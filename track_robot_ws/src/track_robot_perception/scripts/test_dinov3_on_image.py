#!/usr/bin/env python3

import argparse
import json
import os
import sys
import time

import torch
import cv2
import numpy as np

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from track_robot_perception.dinov3_runtime import (
    DEFAULT_HF_MODEL_ID,
    extract_features,
    load_model,
    make_feature_heatmap,
    normalize_feature_rows,
    patch_grid,
    preprocess_bgr_aspect_preserving,
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', required=True)
    parser.add_argument('--model-source', default='torch_hub',
                        choices=('torch_hub', 'huggingface', 'local_repo'))
    parser.add_argument('--model-name', default='dinov3_vits16plus')
    parser.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--input-size', type=int, default=512)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--local-repo', default='')
    parser.add_argument('--weights-path', default='')
    parser.add_argument('--hf-model-id', default=DEFAULT_HF_MODEL_ID)
    return parser.parse_args()


def main():
    args = parse_args()
    image = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError('Could not read image: {}'.format(args.image))
    os.makedirs(os.path.expanduser(args.output_dir), exist_ok=True)

    model, backend = load_model(
        args.model_source, args.model_name, args.device, args.local_repo,
        args.weights_path, args.hf_model_id)
    tensor, transform = preprocess_bgr_aspect_preserving(
        image, args.input_size)
    tensor = tensor.to(args.device)

    start = time.monotonic()
    cls_token, patch_tokens, details = extract_features(model, tensor, backend)
    if args.device.startswith('cuda'):
        torch.cuda.synchronize()
    elapsed_ms = (time.monotonic() - start) * 1000.0

    grid = patch_grid(
        patch_tokens, transform.grid_height, transform.grid_width)
    cls_array = normalize_feature_rows(
        cls_token[0].detach().float().cpu().numpy())
    patch_array = normalize_feature_rows(grid)
    heatmap = make_feature_heatmap(image, grid)

    output_dir = os.path.expanduser(args.output_dir)
    np.save(os.path.join(output_dir, 'cls_token.npy'), cls_array)
    np.save(os.path.join(output_dir, 'patch_tokens.npy'), patch_array)
    np.save(
        os.path.join(output_dir, 'valid_patch_mask.npy'),
        transform.valid_patch_mask)
    cv2.imwrite(os.path.join(output_dir, 'feature_heatmap.jpg'), heatmap)
    metadata = {
        'model': args.model_name,
        'source': backend,
        'device': args.device,
        'input_size': args.input_size,
        'original_size': [transform.source_width, transform.source_height],
        'resized_size': [transform.resized_width, transform.resized_height],
        'preprocessing_scale': transform.scale,
        'padding': {
            'left': transform.padding_left,
            'top': transform.padding_top,
            'right': transform.padding_right,
            'bottom': transform.padding_bottom,
        },
        'valid_patch_count': int(transform.valid_patch_mask.sum()),
        'preprocessing_version': 'aspect_pad_v1',
        'cls_token_shape': list(cls_array.shape),
        'patch_tokens_shape': list(patch_array.shape),
        'inference_ms': round(elapsed_ms, 2),
    }
    metadata.update(details)
    with open(os.path.join(output_dir, 'metadata.json'), 'w') as stream:
        json.dump(metadata, stream, indent=2)
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
