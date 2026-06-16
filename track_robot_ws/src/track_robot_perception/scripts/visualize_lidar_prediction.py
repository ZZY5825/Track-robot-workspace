#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import numpy as np


SEMANTIC_KITTI_NAMES = {
    0: 'unlabeled',
    10: 'car',
    11: 'bicycle',
    15: 'motorcycle',
    18: 'truck',
    20: 'other-vehicle',
    30: 'person',
    31: 'bicyclist',
    32: 'motorcyclist',
    40: 'road',
    44: 'parking',
    48: 'sidewalk',
    49: 'other-ground',
    50: 'building',
    51: 'fence',
    70: 'vegetation',
    71: 'trunk',
    72: 'terrain',
    80: 'pole',
    81: 'traffic-sign',
    252: 'moving-car',
    253: 'moving-bicyclist',
    254: 'moving-person',
    255: 'moving-motorcyclist',
    256: 'moving-on-rails',
    257: 'moving-bus',
    258: 'moving-truck',
    259: 'moving-other-vehicle',
}

HUMAN_LABELS = {30, 31, 32, 253, 254, 255}


def load_points(path: Path) -> np.ndarray:
    if path.suffix == '.npy':
        points = np.load(path)
    else:
        values = np.fromfile(path, dtype='<f4')
        if values.size % 4:
            raise ValueError('point file is not float32 [x,y,z,intensity]')
        points = values.reshape(-1, 4)
    return np.asarray(points[:, :4], dtype=np.float32)


def label_colors(labels: np.ndarray) -> np.ndarray:
    ids = labels.astype(np.uint32)
    colors = np.column_stack((
        (ids * 53 + 80) % 210,
        (ids * 97 + 45) % 210,
        (ids * 193 + 25) % 210,
    )).astype(np.uint8)
    colors[labels < 0] = [100, 100, 100]
    colors[np.isin(labels, list(HUMAN_LABELS))] = [255, 30, 30]
    return colors


def write_ascii_ply(
        path: Path,
        points: np.ndarray,
        labels: np.ndarray,
        instances: np.ndarray):
    colors = label_colors(labels)
    with open(path, 'w', encoding='ascii') as stream:
        stream.write('ply\nformat ascii 1.0\n')
        stream.write(f'element vertex {points.shape[0]}\n')
        stream.write('property float x\nproperty float y\nproperty float z\n')
        stream.write('property float intensity\n')
        stream.write('property uchar red\nproperty uchar green\nproperty uchar blue\n')
        stream.write('property int semantic_label\nproperty int instance_id\n')
        stream.write('end_header\n')
        for point, color, label, instance in zip(
                points, colors, labels, instances):
            stream.write(
                f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} '
                f'{point[3]:.6f} {color[0]} {color[1]} {color[2]} '
                f'{int(label)} {int(instance)}\n')


def main():
    parser = argparse.ArgumentParser(
        description='Convert LiDAR semantic predictions into a colored PLY.')
    parser.add_argument('--input-frame', type=Path, required=True)
    parser.add_argument('--labels', type=Path, required=True)
    parser.add_argument('--instances', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--debug-json', type=Path)
    args = parser.parse_args()

    points = load_points(args.input_frame)
    labels = np.load(args.labels).astype(np.int32).reshape(-1)
    instances = (
        np.load(args.instances).astype(np.int32).reshape(-1)
        if args.instances else np.full(labels.shape[0], -1, dtype=np.int32))
    if labels.shape[0] != points.shape[0]:
        raise ValueError('label count does not match point count')
    if instances.shape[0] != points.shape[0]:
        raise ValueError('instance count does not match point count')

    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_ascii_ply(args.output, points, labels, instances)

    unique, counts = np.unique(labels, return_counts=True)
    summary = {
        'point_count': int(points.shape[0]),
        'human_point_count': int(np.count_nonzero(
            np.isin(labels, list(HUMAN_LABELS)))),
        'label_counts': {
            str(int(label)): {
                'name': SEMANTIC_KITTI_NAMES.get(int(label), 'unknown'),
                'count': int(count),
            }
            for label, count in zip(unique, counts)
        },
    }
    if args.debug_json:
        args.debug_json.parent.mkdir(parents=True, exist_ok=True)
        with open(args.debug_json, 'w', encoding='utf-8') as stream:
            json.dump(summary, stream, indent=2)
    print(f'Wrote {args.output}')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
