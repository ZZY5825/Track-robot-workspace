#!/usr/bin/env python3

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np


SUPPORTED_MODELS = (
    'rangenet',
    'dsnet',
    'panoptic_polarnet',
    'salsanext',
    'cylinder3d',
)

LEARNING_TO_ORIGINAL = np.array([
    0, 10, 11, 15, 18, 20, 30, 31, 32, 40,
    44, 48, 49, 50, 51, 70, 71, 72, 80, 81,
], dtype=np.int32)


def expected_model_files(model_type: str):
    return {
        'rangenet': [],
        'dsnet': [
            'scripts/release/dsnet',
            'network',
        ],
        'panoptic_polarnet': [
            'test_pretrain.py',
            'network',
        ],
        'salsanext': [
            'eval.sh',
            'train',
        ],
        'cylinder3d': [
            'demo_folder.py',
            'network',
        ],
    }[model_type]


def rangenet_entrypoint(model_root: Path) -> Path:
    candidates = [
        model_root / 'train/tasks/semantic/infer.py',
        model_root / 'tasks/semantic/infer.py',
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def verify_model_installation(model_type: str, model_root: Path, checkpoint: Path):
    missing = [
        str(model_root / relative)
        for relative in expected_model_files(model_type)
        if not (model_root / relative).exists()
    ]
    if model_type == 'rangenet' and not rangenet_entrypoint(model_root).exists():
        missing.append(
            'one of: train/tasks/semantic/infer.py, tasks/semantic/infer.py')
    if not checkpoint.exists():
        missing.append(str(checkpoint))
    if missing:
        lines = '\n  '.join(missing)
        raise FileNotFoundError(
            f'{model_type} is not installed/configured. Missing:\n  {lines}\n'
            'No dependency was installed. Clone/configure the selected official '
            'repository separately, then rerun this wrapper.')


def rangenet_command(
        model_root: Path,
        checkpoint: Path,
        dataset_root: Path,
        output_dir: Path):
    return [
        sys.executable,
        str(rangenet_entrypoint(model_root)),
        '-d', str(dataset_root),
        '-l', str(output_dir),
        '-m', str(checkpoint),
        '-s', 'valid',
    ]


def find_prediction_label(output_dir: Path, frame_stem: str) -> Path:
    candidates = sorted(output_dir.rglob(f'{frame_stem}.label'))
    if candidates:
        return candidates[0]

    all_predictions = sorted(output_dir.rglob('*.label'))
    if len(all_predictions) == 1:
        return all_predictions[0]
    raise FileNotFoundError(
        f'Inference finished but no unambiguous prediction for {frame_stem} '
        f'was found below {output_dir}')


def load_semantic_kitti_prediction(
        path: Path,
        label_space: str):
    packed = np.fromfile(path, dtype='<u4')
    semantic = (packed & np.uint32(0xffff)).astype(np.int32)
    instances = (packed >> np.uint32(16)).astype(np.int32)
    if label_space == 'learning':
        if semantic.size and int(semantic.max()) >= LEARNING_TO_ORIGINAL.size:
            raise ValueError('learning label is outside the 0-19 mapping')
        semantic = LEARNING_TO_ORIGINAL[semantic]
    return semantic, instances


def save_outputs(
        output_dir: Path,
        semantic: np.ndarray,
        instances: np.ndarray,
        source_prediction: Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    np.save(output_dir / 'predicted_labels.npy', semantic)
    np.save(output_dir / 'predicted_instances.npy', instances)
    unique, counts = np.unique(semantic, return_counts=True)
    debug = {
        'source_prediction': str(source_prediction.resolve()),
        'point_count': int(semantic.shape[0]),
        'label_counts': {
            str(int(label)): int(count)
            for label, count in zip(unique, counts)
        },
    }
    with open(output_dir / 'debug.json', 'w', encoding='utf-8') as stream:
        json.dump(debug, stream, indent=2)


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Run an externally installed pretrained LiDAR model, or convert an '
            'existing SemanticKITTI .label prediction into NumPy outputs.'))
    parser.add_argument('--model-type', choices=SUPPORTED_MODELS, required=True)
    parser.add_argument('--input-frame', type=Path, required=True)
    parser.add_argument('--dataset-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--model-root', type=Path)
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--prediction-label', type=Path)
    parser.add_argument(
        '--label-space', choices=('original', 'learning'), default='original')
    parser.add_argument(
        '--command',
        nargs=argparse.REMAINDER,
        help=(
            'Explicit external inference command. Use this for DS-Net, '
            'Panoptic-PolarNet, SalsaNext, or Cylinder3D after installation.'))
    args = parser.parse_args()

    if args.prediction_label:
        prediction_path = args.prediction_label
    else:
        if args.model_root is None or args.checkpoint is None:
            parser.error(
                '--model-root and --checkpoint are required unless '
                '--prediction-label is supplied')
        verify_model_installation(
            args.model_type, args.model_root, args.checkpoint)

        if args.command:
            command = args.command
        elif args.model_type == 'rangenet':
            command = rangenet_command(
                args.model_root,
                args.checkpoint,
                args.dataset_root,
                args.output_dir)
        else:
            raise RuntimeError(
                f'{args.model_type} has repository-specific launch arguments. '
                'Pass its official inference invocation after --command. No '
                'dependencies were changed.')

        print('Running:', ' '.join(command))
        subprocess.run(command, cwd=args.model_root, check=True)
        prediction_path = find_prediction_label(
            args.output_dir, args.input_frame.stem)

    semantic, instances = load_semantic_kitti_prediction(
        prediction_path, args.label_space)
    save_outputs(args.output_dir, semantic, instances, prediction_path)
    print(f'Saved {args.output_dir / "predicted_labels.npy"}')
    print(f'Saved {args.output_dir / "predicted_instances.npy"}')
    print(f'Saved {args.output_dir / "debug.json"}')


if __name__ == '__main__':
    main()
