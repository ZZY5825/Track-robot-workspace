import argparse
import json
from pathlib import Path
import stat
import sys
import time

from .cli import _atomic_json_write
from .contracts import TeacherIdentity
from .orin_environment import probe_orin_environment


_MAX_ERROR_LENGTH = 500
_IMPLEMENTATION = 'ultralytics_yolov8s_worldv2_pytorch'
_CODE_REVISION = 'r0c-v1;ultralytics=8.2.103'
_LICENCE = 'AGPL-3.0'


def _add_runtime_arguments(parser):
    parser.add_argument('--runtime-path', required=True, type=Path)
    parser.add_argument('--clip-runtime-path', required=True, type=Path)
    parser.add_argument('--world-checkpoint', required=True, type=Path)
    parser.add_argument('--clip-checkpoint', required=True, type=Path)


def _add_inference_arguments(parser):
    parser.add_argument('--confidence-floor', type=float, default=0.05)
    parser.add_argument('--iou-threshold', type=float, default=0.70)
    parser.add_argument('--input-size', type=int, default=640)
    parser.add_argument('--max-detections', type=int, default=256)
    parser.add_argument('--device', type=int, default=0)
    parser.add_argument('--fp32', action='store_true')


def _parser():
    parser = argparse.ArgumentParser(
        description='R0C Orin-only zero-shot YOLO-World runner')
    commands = parser.add_subparsers(dest='command', required=True)

    probe = commands.add_parser('probe', help='inspect R0C Orin readiness')
    _add_runtime_arguments(probe)
    probe.add_argument('--output', type=Path)

    smoke = commands.add_parser(
        'smoke', help='run one local image and English query')
    _add_runtime_arguments(smoke)
    _add_inference_arguments(smoke)
    smoke.add_argument('--image', required=True, type=Path)
    smoke.add_argument('--query', required=True)
    smoke.add_argument('--licence-approved', action='store_true')
    smoke.add_argument('--output', required=True, type=Path)

    predict = commands.add_parser(
        'predict', help='run every case in an R0A grounding dataset')
    _add_runtime_arguments(predict)
    _add_inference_arguments(predict)
    predict.add_argument('--dataset', required=True, type=Path)
    predict.add_argument('--candidate-id', required=True)
    predict.add_argument('--licence-approved', action='store_true')
    predict.add_argument('--output', required=True, type=Path)
    return parser


def _error(stream, error):
    message = ' '.join(str(error).split())
    stream.write('error: {}\n'.format(message[:_MAX_ERROR_LENGTH]))


def _probe_kwargs(arguments):
    return {
        'runtime_path': arguments.runtime_path,
        'clip_runtime_path': arguments.clip_runtime_path,
        'world_checkpoint': arguments.world_checkpoint,
        'clip_checkpoint': arguments.clip_checkpoint,
    }


def _backend_kwargs(arguments):
    return {
        **_probe_kwargs(arguments),
        'confidence_floor': arguments.confidence_floor,
        'iou_threshold': arguments.iou_threshold,
        'input_size': arguments.input_size,
        'max_detections': arguments.max_detections,
        'device': arguments.device,
        'half': not arguments.fp32,
    }


def _default_backend_factory(**kwargs):
    from .ultralytics_yolo_world import UltralyticsYoloWorld
    return UltralyticsYoloWorld.from_local_model(**kwargs)


def _default_image_probe(path):
    import cv2

    path = Path(path)
    try:
        if path.is_symlink() or not stat.S_ISREG(path.lstat().st_mode):
            raise ValueError('smoke image must be a regular non-symlink file')
    except OSError as error:
        raise ValueError(
            'smoke image must be a regular non-symlink file') from error
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('smoke image could not be decoded')
    height, width = image.shape[:2]
    return int(width), int(height)


def _normalize_query(value):
    from track_robot_semantic_search.grounding_query import (
        normalize_grounding_query,
    )
    return normalize_grounding_query(value).normalized_text


def _run_probe(arguments, probe_fn, stdout):
    result = probe_fn(**_probe_kwargs(arguments))
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


def _require_ready(arguments, probe_fn, stderr):
    result = probe_fn(**_probe_kwargs(arguments))
    if result.runtime_ready:
        return result
    reasons = ','.join(result.report['reasons']) or 'unknown'
    _error(stderr, 'Orin YOLO-World runtime is not ready ({})'.format(
        reasons))
    return None


def _detection_document(detections):
    return [{
        'box_xywh': [
            value.x1,
            value.y1,
            value.x2 - value.x1,
            value.y2 - value.y1,
        ],
        'score': value.score,
        'label': value.label,
    } for value in detections]


def _run_smoke(
        arguments, probe_fn, backend_factory, image_probe, clock_ns, stderr):
    result = _require_ready(arguments, probe_fn, stderr)
    if result is None:
        return 2
    query = _normalize_query(arguments.query)
    probe_image = image_probe or _default_image_probe
    width, height = probe_image(arguments.image)
    factory = backend_factory or _default_backend_factory
    backend = factory(**_backend_kwargs(arguments))
    clock = clock_ns or time.perf_counter_ns
    backend.synchronize()
    started_ns = clock()
    detections = backend.predict(arguments.image, query)
    backend.synchronize()
    finished_ns = clock()
    elapsed_ns = finished_ns - started_ns
    if elapsed_ns < 0:
        raise ValueError('smoke complete-path time must be non-negative')
    memory = backend.incremental_cuda_reserved_mib()
    if memory < 0:
        raise ValueError('smoke CUDA memory must be non-negative')
    report = result.report
    document = {
        'schema_version': 'r0c_smoke/1.0.0',
        'candidate_id': (
            'yolov8s-worldv2-fp{}-{}-c{:03d}-i{:03d}'.format(
                '32' if arguments.fp32 else '16',
                arguments.input_size,
                round(arguments.confidence_floor * 100),
                round(arguments.iou_threshold * 100))),
        'query': query,
        'image': arguments.image.name,
        'image_width': width,
        'image_height': height,
        'complete_path_ms': float(elapsed_ns) / 1_000_000.0,
        'incremental_cuda_reserved_mib': float(memory),
        'model_identity': {
            'implementation': _IMPLEMENTATION,
            'code_revision': _CODE_REVISION,
            'world_checkpoint': report['models']['world'],
            'clip_checkpoint': report['models']['clip'],
            'composite_sha256': report['models']['composite_sha256'],
            'licence': _LICENCE,
        },
        'platform': {
            'role': report['host_role'],
            'hardware': report['runtime']['torch']['device_name'],
            'os': report['os'],
            'python': report['python'],
            'pytorch': report['runtime']['torch']['version'],
            'device': 'cuda:{}'.format(arguments.device),
        },
        'release_evidence': {
            'runtime_available': True,
            'platform_compatible': True,
            'licence_approved': arguments.licence_approved,
        },
        'detections': _detection_document(detections),
    }
    _atomic_json_write(arguments.output, document)
    return 0


def _identity(arguments, report):
    world_name = report['models']['world']['filename']
    clip_name = report['models']['clip']['filename']
    return TeacherIdentity(
        candidate_id=arguments.candidate_id,
        implementation=_IMPLEMENTATION,
        code_revision=_CODE_REVISION,
        checkpoint_id='{}+{}'.format(world_name, clip_name),
        checkpoint_sha256=report['models']['composite_sha256'],
        licence=_LICENCE,
        platform={
            'role': report['host_role'],
            'hardware': report['runtime']['torch']['device_name'],
            'os': report['os'],
            'python': report['python'],
            'pytorch': report['runtime']['torch']['version'],
            'device': 'cuda:{}'.format(arguments.device),
        },
        input_size=(arguments.input_size, arguments.input_size),
    )


def _run_predict(
        arguments, probe_fn, dataset_loader, backend_factory, stderr):
    result = _require_ready(arguments, probe_fn, stderr)
    if result is None:
        return 2
    if dataset_loader is None:
        from track_robot_semantic_search.grounding_dataset import (
            load_grounding_dataset,
        )
        dataset_loader = load_grounding_dataset
    from .teacher_runner import build_prediction_document
    from track_robot_semantic_search.grounding_predictions import (
        load_grounding_predictions,
    )

    dataset = dataset_loader(arguments.dataset)
    factory = backend_factory or _default_backend_factory
    backend = factory(**_backend_kwargs(arguments))
    document = build_prediction_document(
        dataset,
        backend,
        _identity(arguments, result.report),
        licence_approved=arguments.licence_approved,
        platform_compatible=True,
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
        probe_fn=probe_orin_environment,
        dataset_loader=None,
        backend_factory=None,
        image_probe=None,
        clock_ns=None,
        stdout=None,
        stderr=None):
    stdout = stdout or sys.stdout
    stderr = stderr or sys.stderr
    try:
        arguments = _parser().parse_args(argv)
        if arguments.command == 'probe':
            return _run_probe(arguments, probe_fn, stdout)
        if arguments.command == 'smoke':
            return _run_smoke(
                arguments,
                probe_fn,
                backend_factory,
                image_probe,
                clock_ns,
                stderr,
            )
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
