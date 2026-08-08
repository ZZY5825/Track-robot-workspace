"""Command line entry point for the offline confidence benchmark."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

import cv2
import numpy as np

from .confidence_benchmark import (
    dataset_completeness,
    load_confidence_dataset,
    load_jsonl,
    run_offline_inference,
    write_confidence_report,
)
from .dino_crop_descriptors import DinoCropDescriptorBackend
from .model_adapters import OpenAIClipAdapter
from .yolo_world_backend import GroundedDetection, YoloWorldBackend


def _workspace_root():
    return Path(os.environ.get(
        'TRACK_ROBOT_WS', '/home/track-robot/track_robot_ws')).resolve()


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _git_commit():
    try:
        return subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'],
            cwd=str(Path(__file__).resolve().parent),
            stderr=subprocess.DEVNULL, text=True, timeout=5.0).strip()
    except (OSError, subprocess.SubprocessError):
        return 'unknown'


def _normalise(vector):
    value = np.asarray(vector, dtype=np.float32).reshape(-1)
    norm = float(np.linalg.norm(value))
    if not np.isfinite(value).all() or not np.isfinite(norm) or norm <= 1e-12:
        raise ValueError('semantic descriptor is invalid')
    return value / norm


class _ClipRoiScorer:
    def __init__(self, runtime_path, checkpoint, positive_prompt,
                 hard_negative_prompts, device):
        self._adapter = OpenAIClipAdapter(
            model_name='ViT-B/32', checkpoint_path=str(checkpoint),
            runtime_path=str(runtime_path), device=device, grid_size=1,
            window_strategy='grid_only')
        self._positive = _normalise(
            self._adapter.encode_text(positive_prompt))
        self._negatives = tuple(
            _normalise(self._adapter.encode_text(prompt))
            for prompt in hard_negative_prompts)

    def __call__(self, crop):
        encoded = self._adapter.encode_image_grid(crop)
        image = _normalise(encoded.embeddings[0, 0])
        positive = float(np.dot(image, self._positive))
        hardest = max(float(np.dot(image, value))
                      for value in self._negatives)
        return {
            'clip_positive_similarity': positive,
            'clip_hard_negative_max_similarity': hardest,
            'clip_margin': positive - hardest,
            'clip_inference_ms': float(encoded.inference_ms),
        }


class _DinoReferenceScorer:
    def __init__(self, backend, reference_crop):
        self._backend = backend
        self._reference = self._encode(reference_crop)[0]

    def _encode(self, crop):
        height, width = crop.shape[:2]
        detection = GroundedDetection(
            x1=0.0, y1=0.0, x2=float(width), y2=float(height),
            score=1.0, label='reference')
        started = time.monotonic()
        descriptors = self._backend.encode(crop, (detection,))
        elapsed_ms = (time.monotonic() - started) * 1000.0
        if len(descriptors) != 1:
            raise ValueError('DINOv3 did not return one ROI descriptor')
        return _normalise(descriptors[0].values), elapsed_ms

    def __call__(self, crop):
        descriptor, elapsed_ms = self._encode(crop)
        return {
            'dino_similarity': float(np.dot(descriptor, self._reference)),
            'dino_inference_ms': elapsed_ms,
        }


def _reference_crop(dataset_path, sample_id):
    dataset_path = Path(dataset_path)
    dataset = load_confidence_dataset(dataset_path, verify_files=True)
    for trial in dataset['trials']:
        if trial['ground_truth_kind'] != 'target':
            continue
        for sample in trial['samples']:
            if sample['sample_id'] != sample_id:
                continue
            image = cv2.imread(str(
                dataset_path.parent / sample['image_relative_path']),
                cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('DINOv3 reference image cannot be decoded')
            x, y, width, height = trial['ground_truth_bbox_xywh']
            left, top = int(np.floor(x)), int(np.floor(y))
            right, bottom = int(np.ceil(x + width)), int(np.ceil(y + height))
            return image[top:bottom, left:right].copy()
    raise ValueError(
        'DINOv3 reference sample must name a labelled target sample')


def _add_runtime_arguments(parser):
    root = _workspace_root()
    parser.add_argument(
        '--runtime-path', default=str(root / 'models/r0c_runtime/python'))
    parser.add_argument(
        '--clip-runtime-path',
        default=str(root / 'models/phase1_runtime/python'))
    parser.add_argument(
        '--world-checkpoint',
        default=str(root / 'models/r0c/yolov8s-worldv2.pt'))
    parser.add_argument(
        '--clip-checkpoint',
        default=str(root / 'models/phase1/ViT-B-32.pt'))
    parser.add_argument('--device', type=int, default=0)
    parser.add_argument('--full-precision', action='store_true')
    parser.add_argument('--enable-clip-verifier', action='store_true')
    parser.add_argument('--clip-positive-prompt', default='green bottle')
    parser.add_argument(
        '--clip-hard-negative-prompt', action='append',
        dest='clip_negative_prompts')
    parser.add_argument('--enable-dino-verifier', action='store_true')
    parser.add_argument('--dino-reference-sample-id')
    parser.add_argument(
        '--dino-repo',
        default=str(root / 'src/track_robot_core/third_party/dinov3_py38'))
    parser.add_argument(
        '--dino-checkpoint',
        default=str(root / 'models/dinov3_vits16plus_pretrain_lvd1689m.pth'))


def _parser():
    parser = argparse.ArgumentParser(
        description='Offline YOLO-World confidence calibration.')
    commands = parser.add_subparsers(dest='command', required=True)
    status = commands.add_parser('status', help='check dataset completeness')
    status.add_argument('--dataset', required=True)

    infer = commands.add_parser(
        'infer', help='run shared production YOLO backend on frozen frames')
    infer.add_argument('--dataset', required=True)
    infer.add_argument('--output-dir', required=True)
    _add_runtime_arguments(infer)

    report = commands.add_parser('report', help='produce metrics and plots')
    report.add_argument('--dataset', required=True)
    report.add_argument('--run-dir', required=True)
    report.add_argument('--output-dir')
    return parser


def _infer(args):
    yolo = YoloWorldBackend.from_local_model(
        runtime_path=args.runtime_path,
        clip_runtime_path=args.clip_runtime_path,
        world_checkpoint=args.world_checkpoint,
        clip_checkpoint=args.clip_checkpoint,
        confidence_floor=0.05, iou_threshold=0.70, input_size=640,
        max_detections=64, device=args.device,
        half=not args.full_precision)
    clip_scorer = None
    if args.enable_clip_verifier:
        prompts = args.clip_negative_prompts or [
            'green box', 'green tissue box', 'yellow cylindrical object',
            'unrelated object']
        clip_scorer = _ClipRoiScorer(
            args.clip_runtime_path, args.clip_checkpoint,
            args.clip_positive_prompt, prompts,
            'cuda:{}'.format(args.device))
    dino_scorer = None
    if args.enable_dino_verifier:
        if not args.dino_reference_sample_id:
            raise SystemExit(
                '--dino-reference-sample-id is required when DINOv3 '
                'is enabled')
        backend = DinoCropDescriptorBackend.from_local_model(
            args.dino_repo, args.dino_checkpoint,
            device='cuda:{}'.format(args.device))
        dino_scorer = _DinoReferenceScorer(
            backend,
            _reference_crop(args.dataset, args.dino_reference_sample_id))
    torch = yolo._dependencies.torch
    try:
        device_name = str(torch.cuda.get_device_name(args.device))
    except (AttributeError, RuntimeError, TypeError, ValueError):
        device_name = 'unavailable'
    version = getattr(torch, 'version', None)
    provenance = {
        'git_commit': _git_commit(),
        'backend': 'YoloWorldBackend',
        'runtime_path': str(Path(args.runtime_path).resolve()),
        'clip_runtime_path': str(Path(args.clip_runtime_path).resolve()),
        'world_checkpoint': str(Path(args.world_checkpoint).resolve()),
        'world_checkpoint_sha256': _sha256(args.world_checkpoint),
        'clip_checkpoint': str(Path(args.clip_checkpoint).resolve()),
        'clip_checkpoint_sha256': _sha256(args.clip_checkpoint),
        'input_size': 640,
        'confidence_floor': 0.05,
        'iou_threshold': 0.70,
        'max_detections': 64,
        'device': args.device,
        'device_name': device_name,
        'half': not args.full_precision,
        'pytorch': str(getattr(torch, '__version__', 'unknown')),
        'cuda': str(getattr(version, 'cuda', 'unknown')),
        'clip_enabled': bool(args.enable_clip_verifier),
        'dino_enabled': bool(args.enable_dino_verifier),
    }
    if args.enable_dino_verifier:
        provenance.update({
            'dino_repo': str(Path(args.dino_repo).resolve()),
            'dino_checkpoint': str(Path(args.dino_checkpoint).resolve()),
            'dino_checkpoint_sha256': _sha256(args.dino_checkpoint),
            'dino_reference_sample_id': args.dino_reference_sample_id,
        })
    return run_offline_inference(
        args.dataset, args.output_dir, yolo,
        clip_scorer=clip_scorer, dino_scorer=dino_scorer,
        run_provenance=provenance)


def main(argv=None):
    args = _parser().parse_args(argv)
    dataset = load_confidence_dataset(args.dataset, verify_files=True)
    completeness = dataset_completeness(dataset)
    if args.command == 'status':
        result = completeness
    elif args.command == 'infer':
        result = _infer(args)
    else:
        run_dir = Path(args.run_dir).resolve()
        result = write_confidence_report(
            load_jsonl(run_dir / 'frames.jsonl'),
            load_jsonl(run_dir / 'candidates.jsonl'),
            run_dir,
            Path(args.output_dir).resolve() if args.output_dir else run_dir,
            completeness)
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))


if __name__ == '__main__':
    main()
