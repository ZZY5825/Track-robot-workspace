#!/usr/bin/env python3

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform
import sys
import time

import numpy as np
import torch

from track_robot_perception.dinov3_runtime import (
    extract_features,
    load_model,
    patch_grid,
    preprocess_bgr_aspect_preserving,
)
from track_robot_semantic_search.benchmarking import (
    latency_summary,
    unavailable_candidate,
)
from track_robot_semantic_search.manifest import write_json_atomic
from track_robot_semantic_search.model_selection import select_candidate


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def synchronize(device):
    if str(device).startswith('cuda'):
        torch.cuda.synchronize()


def parser():
    root = argparse.ArgumentParser(
        description='Benchmark the Phase 1 aspect-preserving DINO path.')
    root.add_argument('--local-repo', type=Path, required=True)
    root.add_argument('--weights', type=Path, required=True)
    root.add_argument('--output', type=Path, required=True)
    root.add_argument('--iterations', type=int, default=30)
    root.add_argument('--warmups', type=int, default=3)
    root.add_argument('--input-size', type=int, default=512)
    root.add_argument(
        '--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    root.add_argument('--software-revision', required=True)
    return root


def main(argv=None):
    arguments = parser().parse_args(argv)
    if arguments.iterations <= 0 or arguments.warmups < 0:
        raise SystemExit('iterations must be positive and warmups non-negative')
    if not arguments.weights.is_file():
        raise SystemExit('weights do not exist: {}'.format(arguments.weights))
    if not (arguments.local_repo / 'hubconf.py').is_file():
        raise SystemExit(
            'local DINO repository is invalid: {}'.format(arguments.local_repo))

    synthetic_image = np.zeros((720, 1280, 3), dtype=np.uint8)
    load_start = time.monotonic()
    model, backend = load_model(
        'local_repo',
        'dinov3_vits16plus',
        arguments.device,
        str(arguments.local_repo),
        str(arguments.weights))
    synchronize(arguments.device)
    model_load_ms = (time.monotonic() - load_start) * 1000.0

    def run_once():
        start = time.monotonic()
        tensor, transform = preprocess_bgr_aspect_preserving(
            synthetic_image, arguments.input_size)
        tensor = tensor.to(arguments.device)
        cls_token, tokens, _ = extract_features(model, tensor, backend)
        synchronize(arguments.device)
        grid = patch_grid(tokens, transform.grid_height, transform.grid_width)
        _ = cls_token[0].detach().float().cpu().numpy()
        _ = grid.astype(np.float32, copy=False)
        synchronize(arguments.device)
        return (time.monotonic() - start) * 1000.0, transform

    cold_ms, transform = run_once()
    for _ in range(arguments.warmups):
        run_once()
    if arguments.device.startswith('cuda'):
        torch.cuda.reset_peak_memory_stats()
    latencies = [run_once()[0] for _ in range(arguments.iterations)]
    summary = latency_summary(latencies)
    cuda = {
        'allocated_peak_mb': 0.0,
        'reserved_mb': 0.0,
    }
    if arguments.device.startswith('cuda'):
        cuda = {
            'allocated_peak_mb': torch.cuda.max_memory_allocated() / 1048576.0,
            'reserved_mb': torch.cuda.memory_reserved() / 1048576.0,
        }

    candidates = [
        unavailable_candidate(
            'siglip2_b',
            'No Python-3.8-compatible runtime or checkpoint is present; '
            'licence/export review cannot be completed without the artifact.'),
        unavailable_candidate(
            'open_clip_vit_b32',
            'OpenCLIP runtime and aligned checkpoint are absent from the '
            'isolated external model path.'),
    ]
    selection = select_candidate(candidates)
    report = {
        'schema_version': '1.0.0',
        'run_id': 'phase1-model-selection-2026-07-15',
        'platform': {
            'python': platform.python_version(),
            'pytorch': torch.__version__,
            'device': str(arguments.device),
            'machine': platform.machine(),
            'jetpack_l4t': 'R35.1',
        },
        'software_revision': arguments.software_revision,
        'dino_runtime': {
            'model': 'dinov3_vits16plus',
            'backend': backend,
            'checkpoint_sha256': sha256_file(arguments.weights),
            'licence': 'DINOv3 License, local LICENSE.md reviewed',
            'redistribution': 'subject to DINOv3 License terms',
            'preprocessing_version': 'aspect_pad_v1',
            'source_size': [1280, 720],
            'model_size': [arguments.input_size, arguments.input_size],
            'resized_size': [
                transform.resized_width, transform.resized_height],
            'padding': [
                transform.padding_left, transform.padding_top,
                transform.padding_right, transform.padding_bottom],
            'valid_patch_count': int(transform.valid_patch_mask.sum()),
            'model_load_ms': model_load_ms,
            'cold_complete_path_ms': cold_ms,
            'complete_path_latency': summary,
            'semantic_output_capacity_hz': 1000.0 / summary['p95_ms'],
            'latency_gate_p95_at_most_150_ms': summary['p95_ms'] <= 150.0,
            'rate_gate_at_least_5_hz': 1000.0 / summary['p95_ms'] >= 5.0,
            'cuda': cuda,
        },
        'candidates': candidates,
        'selection': asdict(selection),
    }
    write_json_atomic(arguments.output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
