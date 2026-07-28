import json
import math
import re
from dataclasses import dataclass
from pathlib import Path, PurePath, PureWindowsPath
from types import MappingProxyType
from typing import Any, Mapping, Tuple

from .grounding_dataset import GroundingBox


SCHEMA_VERSION = '1.0.0'
_ROOT_KEYS = {
    'schema_version', 'dataset_id', 'candidate_id', 'model_identity',
    'platform', 'input_size', 'incremental_cuda_reserved_mib',
    'release_evidence', 'predictions',
}
_MODEL_IDENTITY_KEYS = {
    'implementation', 'code_revision', 'checkpoint_id',
    'checkpoint_sha256', 'licence',
}
_PLATFORM_KEYS = {'role', 'hardware', 'os', 'python', 'pytorch', 'device'}
_RELEASE_EVIDENCE_KEYS = {
    'runtime_available', 'platform_compatible', 'licence_approved',
}
_PREDICTION_KEYS = {'case_id', 'complete_path_ms', 'detections'}
_DETECTION_KEYS = {'box_xywh', 'score', 'label'}
_SHA256 = re.compile(r'^[0-9a-f]{64}$')
_MAX_TEXT_LENGTH = 4096
_IDENTIFIER_LENGTH = 256
_CHECKPOINT_ID_LENGTH = 1024


@dataclass(frozen=True)
class GroundingDetection:
    box: GroundingBox
    score: float
    label: str


@dataclass(frozen=True)
class GroundingPrediction:
    case_id: str
    complete_path_ms: float
    detections: Tuple[GroundingDetection, ...]


@dataclass(frozen=True)
class GroundingPredictionSet:
    dataset_id: str
    candidate_id: str
    model_identity: Mapping[str, str]
    platform: Mapping[str, str]
    input_size: Tuple[int, int]
    incremental_cuda_reserved_mib: float
    release_evidence: Mapping[str, bool]
    predictions: Mapping[str, GroundingPrediction]


def load_grounding_predictions(path: Path) -> GroundingPredictionSet:
    path = Path(path)
    try:
        document = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError('invalid grounding predictions document: {}'.format(
            error)) from error

    root = _require_mapping(document, 'grounding predictions')
    _require_exact_keys(root, _ROOT_KEYS, 'grounding predictions')
    if root['schema_version'] != SCHEMA_VERSION:
        raise ValueError('unsupported schema_version')

    model_identity = _parse_model_identity(root['model_identity'])
    platform = _parse_string_mapping(root['platform'], _PLATFORM_KEYS, 'platform')
    input_size = _parse_input_size(root['input_size'])
    release_evidence = _parse_release_evidence(root['release_evidence'])
    memory = _nonnegative_finite_number(
        root['incremental_cuda_reserved_mib'], 'incremental_cuda_reserved_mib')
    prediction_values = root['predictions']
    if not isinstance(prediction_values, list) or len(prediction_values) > 100000:
        raise ValueError('predictions must be an array with at most 100000 entries')

    predictions = {}
    for index, value in enumerate(prediction_values):
        prediction = _parse_prediction(value, index)
        if prediction.case_id in predictions:
            raise ValueError('duplicate case_id')
        predictions[prediction.case_id] = prediction
    return GroundingPredictionSet(
        dataset_id=_nonempty_string(
            root['dataset_id'], 'dataset_id', _IDENTIFIER_LENGTH),
        candidate_id=_nonempty_string(
            root['candidate_id'], 'candidate_id', _IDENTIFIER_LENGTH),
        model_identity=MappingProxyType(model_identity),
        platform=MappingProxyType(platform),
        input_size=input_size,
        incremental_cuda_reserved_mib=memory,
        release_evidence=MappingProxyType(release_evidence),
        predictions=MappingProxyType(predictions),
    )


def _parse_model_identity(value: object) -> Mapping[str, str]:
    identity = _parse_string_mapping(
        value, _MODEL_IDENTITY_KEYS, 'model_identity', {
            'implementation': _IDENTIFIER_LENGTH,
            'code_revision': _MAX_TEXT_LENGTH,
            'checkpoint_id': _CHECKPOINT_ID_LENGTH,
            'checkpoint_sha256': 64,
            'licence': _IDENTIFIER_LENGTH,
        })
    checkpoint_id = identity['checkpoint_id']
    if (checkpoint_id.startswith(('/', '\\')) or
            PurePath(checkpoint_id).is_absolute() or
            PureWindowsPath(checkpoint_id).is_absolute()):
        raise ValueError('checkpoint_id must not be an absolute path')
    if not _SHA256.fullmatch(identity['checkpoint_sha256']):
        raise ValueError('checkpoint_sha256 must be lowercase sha256')
    return identity


def _parse_string_mapping(value: object, keys, name: str,
                          max_lengths=None) -> Mapping[str, str]:
    mapping = _require_mapping(value, name)
    _require_exact_keys(mapping, keys, name)
    return {
        key: _nonempty_string(
            mapping[key], '{}.{}'.format(name, key),
            max_lengths[key] if max_lengths else _IDENTIFIER_LENGTH)
        for key in keys
    }


def _parse_input_size(value: object) -> Tuple[int, int]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError('input_size must contain width and height')
    width = _positive_integer(value[0], 'input_size width')
    height = _positive_integer(value[1], 'input_size height')
    if width > 16384 or height > 16384:
        raise ValueError('input_size dimensions must not exceed 16384')
    return (width, height)


def _parse_release_evidence(value: object) -> Mapping[str, bool]:
    evidence = _require_mapping(value, 'release_evidence')
    _require_exact_keys(evidence, _RELEASE_EVIDENCE_KEYS, 'release_evidence')
    for key in _RELEASE_EVIDENCE_KEYS:
        if not isinstance(evidence[key], bool):
            raise ValueError('release_evidence.{} must be a boolean'.format(key))
    return {key: evidence[key] for key in _RELEASE_EVIDENCE_KEYS}


def _parse_prediction(value: object, index: int) -> GroundingPrediction:
    name = 'predictions[{}]'.format(index)
    prediction = _require_mapping(value, name)
    _require_exact_keys(prediction, _PREDICTION_KEYS, name)
    detections_value = prediction['detections']
    if not isinstance(detections_value, list) or len(detections_value) > 256:
        raise ValueError('detections must be an array with at most 256 entries')
    return GroundingPrediction(
        case_id=_nonempty_string(
            prediction['case_id'], 'case_id', _IDENTIFIER_LENGTH),
        complete_path_ms=_nonnegative_finite_number(
            prediction['complete_path_ms'], 'complete_path_ms'),
        detections=tuple(
            _parse_detection(detection, index, detection_index)
            for detection_index, detection in enumerate(detections_value)),
    )


def _parse_detection(value: object, prediction_index: int,
                     detection_index: int) -> GroundingDetection:
    name = 'predictions[{}].detections[{}]'.format(
        prediction_index, detection_index)
    detection = _require_mapping(value, name)
    _require_exact_keys(detection, _DETECTION_KEYS, name)
    box_value = detection['box_xywh']
    if not isinstance(box_value, list) or len(box_value) != 4:
        raise ValueError('box_xywh must contain four values')
    if not all(_is_finite_number(item) for item in box_value):
        raise ValueError('box_xywh values must be finite numbers')
    x, y, width, height = (float(item) for item in box_value)
    if width <= 0 or height <= 0:
        raise ValueError('box_xywh dimensions must be positive')
    score = _nonnegative_finite_number(detection['score'], 'score')
    if score > 1:
        raise ValueError('score must be in [0, 1]')
    return GroundingDetection(
        box=GroundingBox(x=x, y=y, width=width, height=height),
        score=score,
        label=_nonempty_string(detection['label'], 'label', _MAX_TEXT_LENGTH),
    )


def _require_mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError('{} must be an object'.format(name))
    return value


def _require_exact_keys(value: Mapping[str, Any], keys, name: str) -> None:
    missing = sorted(set(keys) - set(value))
    unknown = sorted(set(value) - set(keys))
    if missing:
        raise ValueError('{} missing {}'.format(name, ', '.join(missing)))
    if unknown:
        raise ValueError('{} contains unknown fields'.format(name))


def _nonempty_string(value: object, name: str,
                     max_length: int = _MAX_TEXT_LENGTH) -> str:
    if (not isinstance(value, str) or not value or
            len(value) > max_length):
        raise ValueError('{} must be a bounded non-empty string'.format(name))
    return value


def _positive_integer(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError('{} must be a positive integer'.format(name))
    return value


def _nonnegative_finite_number(value: object, name: str) -> float:
    if not _is_finite_number(value) or value < 0:
        raise ValueError('{} must be a non-negative finite number'.format(name))
    return float(value)


def _is_finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float)) and not isinstance(value, bool) and
        math.isfinite(value))
