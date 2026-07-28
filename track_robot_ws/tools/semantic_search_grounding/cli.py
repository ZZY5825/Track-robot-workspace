import argparse
import json
import os
import sys
import tempfile
from pathlib import Path

from .contracts import TeacherIdentity
from .environment import probe_environment


_MAX_ERROR_LENGTH = 500


def _parser():
    parser = argparse.ArgumentParser(
        description='R0B desktop-only zero-shot teacher runner')
    commands = parser.add_subparsers(dest='command', required=True)

    probe = commands.add_parser(
        'probe', help='inspect desktop teacher readiness')
    probe.add_argument('--model-dir', type=Path)
    probe.add_argument('--checkpoint-file', type=Path)
    probe.add_argument('--output', type=Path)

    predict = commands.add_parser(
        'predict', help='run local Grounding DINO over an R0A dataset')
    predict.add_argument('--dataset', required=True, type=Path)
    predict.add_argument('--model-dir', required=True, type=Path)
    predict.add_argument('--checkpoint-file', required=True, type=Path)
    predict.add_argument('--model-revision', required=True)
    predict.add_argument('--candidate-id', required=True)
    predict.add_argument('--licence', required=True)
    predict.add_argument('--licence-approved', action='store_true')
    predict.add_argument('--box-threshold', type=float, default=0.05)
    predict.add_argument('--text-threshold', type=float, default=0.05)
    predict.add_argument('--max-detections', type=int, default=256)
    predict.add_argument('--input-width', type=int, default=1333)
    predict.add_argument('--input-height', type=int, default=800)
    predict.add_argument('--output', required=True, type=Path)
    return parser


def _error(stream, value):
    message = ' '.join(str(value).split())
    stream.write('error: {}\n'.format(message[:_MAX_ERROR_LENGTH]))


def _atomic_json_write(path, document, validator=None):
    path = Path(path)
    parent = path.parent
    if parent.is_symlink() or not parent.is_dir():
        raise ValueError('output parent must be an existing directory')
    if path.is_symlink():
        raise ValueError('output must not be a symbolic link')
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
                mode='w',
                encoding='utf-8',
                dir=str(parent),
                prefix='.{}.'.format(path.name),
                suffix='.tmp',
                delete=False) as stream:
            temporary_path = Path(stream.name)
            json.dump(
                dict(document),
                stream,
                ensure_ascii=True,
                indent=2,
                sort_keys=True,
            )
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        if validator is not None:
            validator(temporary_path)
        os.replace(str(temporary_path), str(path))
        temporary_path = None
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass


def _platform_from_probe(report):
    devices = report['gpu']['devices']
    hardware = ', '.join(device['name'] for device in devices)
    return {
        'role': 'r0b_desktop_teacher',
        'hardware': hardware,
        'os': report['os'],
        'python': report['python'],
        'pytorch': report['runtime']['torch']['version'],
        'device': 'cuda',
    }


def _run_probe(arguments, probe_fn, stdout):
    result = probe_fn(
        model_dir=arguments.model_dir,
        checkpoint_file=arguments.checkpoint_file,
    )
    if arguments.output is None:
        json.dump(
            dict(result.report),
            stdout,
            ensure_ascii=True,
            indent=2,
            sort_keys=True,
        )
        stdout.write('\n')
    else:
        _atomic_json_write(arguments.output, result.report)
    return 0 if result.runtime_ready else 2


def _run_predict(
        arguments, probe_fn, dataset_loader, backend_factory, stderr):
    result = probe_fn(
        model_dir=arguments.model_dir,
        checkpoint_file=arguments.checkpoint_file,
    )
    if not result.runtime_ready:
        reasons = ','.join(result.report['reasons']) or 'unknown'
        _error(
            stderr,
            'desktop teacher runtime is not ready ({})'.format(reasons),
        )
        return 2

    if dataset_loader is None:
        from track_robot_semantic_search.grounding_dataset import (
            load_grounding_dataset,
        )
        dataset_loader = load_grounding_dataset
    if backend_factory is None:
        from .huggingface_grounding_dino import HuggingFaceGroundingDino
        backend_factory = HuggingFaceGroundingDino.from_local_model
    from .teacher_runner import build_prediction_document
    from track_robot_semantic_search.grounding_predictions import (
        load_grounding_predictions,
    )

    dataset = dataset_loader(arguments.dataset)
    backend = backend_factory(
        model_dir=arguments.model_dir,
        box_threshold=arguments.box_threshold,
        text_threshold=arguments.text_threshold,
        max_detections=arguments.max_detections,
    )
    identity = TeacherIdentity(
        candidate_id=arguments.candidate_id,
        implementation='huggingface_transformers_grounding_dino',
        code_revision=arguments.model_revision,
        checkpoint_id=str(arguments.checkpoint_file),
        checkpoint_sha256=result.report['model']['checkpoint_sha256'],
        licence=arguments.licence,
        platform=_platform_from_probe(result.report),
        input_size=(arguments.input_width, arguments.input_height),
    )
    document = build_prediction_document(
        dataset=dataset,
        backend=backend,
        identity=identity,
        licence_approved=arguments.licence_approved,
    )

    expected_case_ids = {case.case_id for case in dataset.cases}

    def validate(path):
        parsed = load_grounding_predictions(path)
        if parsed.dataset_id != dataset.dataset_id:
            raise ValueError('prediction dataset ID does not match')
        if set(parsed.predictions) != expected_case_ids:
            raise ValueError('prediction case set does not match dataset')

    _atomic_json_write(arguments.output, document, validator=validate)
    return 0


def run(
        argv=None,
        probe_fn=probe_environment,
        dataset_loader=None,
        backend_factory=None,
        stdout=None,
        stderr=None):
    stdout = stdout or sys.stdout
    stderr = stderr or sys.stderr
    try:
        arguments = _parser().parse_args(argv)
        if arguments.command == 'probe':
            return _run_probe(arguments, probe_fn, stdout)
        return _run_predict(
            arguments,
            probe_fn,
            dataset_loader,
            backend_factory,
            stderr,
        )
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        _error(stderr, error)
        return 2


def main():
    raise SystemExit(run())


if __name__ == '__main__':
    main()
