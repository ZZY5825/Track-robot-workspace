import math
import time
from typing import Callable, Mapping, Optional

from track_robot_semantic_search.grounding_dataset import GroundingDataset

from .contracts import TeacherDetection, TeacherIdentity


SCHEMA_VERSION = '1.0.0'
_MAX_DETECTIONS = 256
_PLATFORM_KEYS = {
    'role', 'hardware', 'os', 'python', 'pytorch', 'device',
}


def _finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float)) and not isinstance(value, bool) and
        math.isfinite(value)
    )


def _validate_detection(value: TeacherDetection) -> TeacherDetection:
    if not isinstance(value, TeacherDetection):
        raise ValueError('teacher detection must use TeacherDetection')
    if not all(_finite_number(number) for number in (
            value.x1, value.y1, value.x2, value.y2, value.score)):
        raise ValueError('teacher detection values must be finite')
    x1 = float(value.x1)
    y1 = float(value.y1)
    x2 = float(value.x2)
    y2 = float(value.y2)
    score = float(value.score)
    if x1 < 0.0 or y1 < 0.0 or x2 <= x1 or y2 <= y1:
        raise ValueError('teacher detection box is invalid')
    if score < 0.0 or score > 1.0:
        raise ValueError('teacher detection score must be in [0, 1]')
    if not isinstance(value.label, str) or not value.label:
        raise ValueError('teacher detection label must be non-empty')
    return TeacherDetection(x1, y1, x2, y2, score, value.label)


def _detection_sort_key(value: TeacherDetection):
    return (
        -value.score,
        value.x1,
        value.y1,
        value.x2,
        value.y2,
        value.label,
    )


def _validate_identity(identity: TeacherIdentity) -> None:
    if not isinstance(identity, TeacherIdentity):
        raise ValueError('teacher identity is invalid')
    for name in (
            'candidate_id', 'implementation', 'code_revision',
            'checkpoint_id', 'checkpoint_sha256', 'licence'):
        value = getattr(identity, name)
        if not isinstance(value, str) or not value:
            raise ValueError('teacher identity {} is invalid'.format(name))
    if len(identity.checkpoint_sha256) != 64 or any(
            character not in '0123456789abcdef'
            for character in identity.checkpoint_sha256):
        raise ValueError('teacher identity checkpoint sha256 is invalid')
    if set(identity.platform) != _PLATFORM_KEYS or any(
            not isinstance(value, str) or not value
            for value in identity.platform.values()):
        raise ValueError('teacher identity platform is invalid')
    if (
            not isinstance(identity.input_size, tuple) or
            len(identity.input_size) != 2 or
            any(isinstance(value, bool) or not isinstance(value, int) or
                value <= 0 for value in identity.input_size)):
        raise ValueError('teacher identity input size is invalid')


def _prediction_record(case, detections, elapsed_ns):
    if (
            isinstance(elapsed_ns, bool) or
            not isinstance(elapsed_ns, (int, float)) or
            not math.isfinite(elapsed_ns) or elapsed_ns < 0):
        raise ValueError('complete path time must be non-negative')
    values = sorted(
        (_validate_detection(value) for value in detections),
        key=_detection_sort_key,
    )[:_MAX_DETECTIONS]
    return {
        'case_id': case.case_id,
        'complete_path_ms': float(elapsed_ns) / 1_000_000.0,
        'detections': [{
            'box_xywh': [
                value.x1,
                value.y1,
                value.x2 - value.x1,
                value.y2 - value.y1,
            ],
            'score': value.score,
            'label': value.label,
        } for value in values],
    }


def build_prediction_document(
        dataset: GroundingDataset,
        backend,
        identity: TeacherIdentity,
        licence_approved: bool,
        clock_ns: Optional[Callable[[], int]] = None,
        platform_compatible: bool = False,
        ) -> Mapping[str, object]:
    if not isinstance(dataset, GroundingDataset):
        raise ValueError('grounding dataset is invalid')
    if not isinstance(licence_approved, bool):
        raise ValueError('licence_approved must be boolean')
    if not isinstance(platform_compatible, bool):
        raise ValueError('platform_compatible must be boolean')
    _validate_identity(identity)
    clock = clock_ns or time.perf_counter_ns

    predictions = []
    for case in dataset.cases:
        backend.synchronize()
        started_ns = clock()
        detections = backend.predict(
            case.image_path, case.query.normalized_text)
        backend.synchronize()
        finished_ns = clock()
        predictions.append(_prediction_record(
            case, detections, finished_ns - started_ns))

    memory = backend.incremental_cuda_reserved_mib()
    if not _finite_number(memory) or memory < 0.0:
        raise ValueError(
            'incremental CUDA reserved memory must be non-negative')
    return {
        'schema_version': SCHEMA_VERSION,
        'dataset_id': dataset.dataset_id,
        'candidate_id': identity.candidate_id,
        'model_identity': {
            'implementation': identity.implementation,
            'code_revision': identity.code_revision,
            'checkpoint_id': identity.checkpoint_id,
            'checkpoint_sha256': identity.checkpoint_sha256,
            'licence': identity.licence,
        },
        'platform': dict(identity.platform),
        'input_size': list(identity.input_size),
        'incremental_cuda_reserved_mib': float(memory),
        'release_evidence': {
            'runtime_available': True,
            'platform_compatible': platform_compatible,
            'licence_approved': licence_approved,
        },
        'predictions': predictions,
    }
