#!/usr/bin/env python3

import argparse
import json
import shutil
from pathlib import Path

import numpy as np


SUPPORTED_MODELS = (
    'rangenet',
    'dsnet',
    'panoptic_polarnet',
    'salsanext',
    'cylinder3d',
)


def load_points(path: Path) -> np.ndarray:
    if path.suffix == '.npy':
        points = np.load(path)
    elif path.suffix == '.bin':
        values = np.fromfile(path, dtype='<f4')
        if values.size % 4:
            raise ValueError(
                f'{path} does not contain float32 [x,y,z,intensity] records')
        points = values.reshape(-1, 4)
    else:
        raise ValueError('input frame must be .npy or .bin')

    if points.ndim != 2 or points.shape[1] < 4:
        raise ValueError('input array must have shape [N, >=4]')
    return np.ascontiguousarray(points[:, :4], dtype='<f4')


def main():
    parser = argparse.ArgumentParser(
        description='Prepare an exported LiDAR frame as a SemanticKITTI scan.')
    parser.add_argument('--model-type', choices=SUPPORTED_MODELS, required=True)
    parser.add_argument('--input-frame', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--sequence', default='08')
    parser.add_argument('--frame-index', type=int, default=0)
    args = parser.parse_args()

    points = load_points(args.input_frame)
    frame_name = f'{args.frame_index:06d}.bin'
    velodyne_dir = (
        args.output_dir / 'sequences' / args.sequence / 'velodyne')
    velodyne_dir.mkdir(parents=True, exist_ok=True)
    destination = velodyne_dir / frame_name
    points.tofile(destination)

    source_metadata = args.input_frame.with_suffix('.json')
    if source_metadata.exists():
        shutil.copy2(
            source_metadata,
            velodyne_dir / f'{args.frame_index:06d}.json')

    manifest = {
        'model_type': args.model_type,
        'source_frame': str(args.input_frame.resolve()),
        'dataset_root': str(args.output_dir.resolve()),
        'sequence': args.sequence,
        'prepared_scan': str(destination.resolve()),
        'point_count': int(points.shape[0]),
        'format': 'little-endian float32 [x, y, z, intensity]',
        'note': (
            'All selected model families accept SemanticKITTI-style scans. '
            'Range-image models still perform their own spherical projection.'),
    }
    manifest_path = args.output_dir / 'prepare_manifest.json'
    with open(manifest_path, 'w', encoding='utf-8') as stream:
        json.dump(manifest, stream, indent=2)

    print(f'Prepared {destination}')
    print(f'Points: {points.shape[0]}')
    print(f'Manifest: {manifest_path}')


if __name__ == '__main__':
    main()
