import hashlib
import json
import math
import re
import stat
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence, Tuple

import cv2

from .grounding_query import GroundingQuery, normalize_grounding_query


SCHEMA_VERSION = '1.0.0'
_SPLITS = {'train', 'validation', 'test'}
_REVIEW_STATUSES = {'human_authored', 'human_verified'}
_ROOT_KEYS = {'schema_version', 'dataset_id', 'cases'}
_CASE_KEYS = {
    'case_id', 'split', 'image_relative_path', 'image_sha256',
    'image_width', 'image_height', 'session_id', 'physical_object_id',
    'query_text', 'target_present', 'ground_truth_boxes_xywh',
    'scenario_tags', 'label_review_status',
}
_SAFE_RELATIVE_PATH = re.compile(
    r'^(?!/)(?![A-Za-z]:)(?!.*\\)(?!.*(?:^|/)\.\.(?:/|$)).+$')
_SHA256 = re.compile(r'^[0-9a-f]{64}$')


@dataclass(frozen=True)
class GroundingBox:
    x: float
    y: float
    width: float
    height: float


@dataclass(frozen=True)
class GroundingCase:
    case_id: str
    split: str
    image_path: Path
    image_sha256: str
    session_id: str
    physical_object_id: str
    query: GroundingQuery
    target_present: bool
    boxes: Tuple[GroundingBox, ...]
    scenario_tags: Tuple[str, ...]
    label_review_status: str


@dataclass(frozen=True)
class GroundingDataset:
    dataset_id: str
    cases: Tuple[GroundingCase, ...]


def _safe_relative_path(value: object) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError('image_relative_path must be a safe relative path')
    path = PurePosixPath(value)
    if (not _SAFE_RELATIVE_PATH.fullmatch(value) or path.is_absolute() or
            '..' in path.parts):
        raise ValueError('image_relative_path must be a safe relative path')
    return value


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _box_from_xywh(value: object, width: int, height: int) -> GroundingBox:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError('ground_truth_boxes_xywh entries must have four values')
    if any(not _is_finite_number(item) for item in value):
        raise ValueError('ground_truth_boxes_xywh values must be finite numbers')
    x, y, box_width, box_height = (float(item) for item in value)
    if x < 0 or y < 0 or box_width <= 0 or box_height <= 0:
        raise ValueError('ground_truth_boxes_xywh values must be non-negative')
    if x + box_width > width or y + box_height > height:
        raise ValueError('ground truth box exceeds image bounds')
    return GroundingBox(x=x, y=y, width=box_width, height=box_height)


def _reject_split_leakage(cases: Sequence[GroundingCase]) -> None:
    indexes = {
        'session_id': {},
        'physical_object_id': {},
        'image_sha256': {},
    }
    for case in cases:
        values = {
            'session_id': case.session_id,
            'image_sha256': case.image_sha256,
        }
        if case.physical_object_id:
            values['physical_object_id'] = case.physical_object_id
        for field, value in values.items():
            splits = indexes[field].setdefault(value, set())
            splits.add(case.split)
            if len(splits) > 1:
                raise ValueError('split leakage for {}'.format(field))


def load_grounding_dataset(
        document_path: Path, verify_images: bool = True) -> GroundingDataset:
    document_path = Path(document_path)
    try:
        document = json.loads(document_path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError('invalid grounding dataset document: {}'.format(
            error)) from error

    root = _require_mapping(document, 'grounding dataset')
    _require_exact_keys(root, _ROOT_KEYS, 'grounding dataset')
    if root['schema_version'] != SCHEMA_VERSION:
        raise ValueError('unsupported schema_version')
    dataset_id = _nonempty_string(root['dataset_id'], 'dataset_id')
    case_values = root['cases']
    if not isinstance(case_values, list) or len(case_values) > 100000:
        raise ValueError('cases must be an array with at most 100000 entries')

    cases = []
    case_ids = set()
    for index, value in enumerate(case_values):
        case = _parse_case(value, document_path.parent, verify_images, index)
        if case.case_id in case_ids:
            raise ValueError('duplicate case_id')
        case_ids.add(case.case_id)
        cases.append(case)
    _reject_split_leakage(cases)
    return GroundingDataset(dataset_id=dataset_id, cases=tuple(cases))


def _parse_case(value: object, document_dir: Path,
                verify_images: bool, index: int) -> GroundingCase:
    name = 'cases[{}]'.format(index)
    case = _require_mapping(value, name)
    _require_exact_keys(case, _CASE_KEYS, name)
    case_id = _nonempty_string(case['case_id'], 'case_id')
    split = case['split']
    if not isinstance(split, str) or split not in _SPLITS:
        raise ValueError('split must be train, validation, or test')
    relative_path = _safe_relative_path(case['image_relative_path'])
    image_sha256 = case['image_sha256']
    if not isinstance(image_sha256, str) or not _SHA256.fullmatch(image_sha256):
        raise ValueError('image_sha256 must be lowercase sha256')
    image_width = _positive_integer(case['image_width'], 'image_width')
    image_height = _positive_integer(case['image_height'], 'image_height')
    session_id = _nonempty_string(case['session_id'], 'session_id')
    physical_object_id = case['physical_object_id']
    if not isinstance(physical_object_id, str):
        raise ValueError('physical_object_id must be a string')
    query = normalize_grounding_query(case['query_text'])
    target_present = case['target_present']
    if not isinstance(target_present, bool):
        raise ValueError('target_present must be a boolean')
    if target_present and not physical_object_id:
        raise ValueError(
            'physical_object_id is required when target_present is true')
    if not target_present and physical_object_id:
        raise ValueError(
            'physical_object_id must be empty when target_present is false')
    box_values = case['ground_truth_boxes_xywh']
    if not isinstance(box_values, list):
        raise ValueError('ground_truth_boxes_xywh must be an array')
    if target_present != bool(box_values):
        raise ValueError(
            'target_present must match ground_truth_boxes_xywh presence')
    boxes = tuple(
        _box_from_xywh(box, image_width, image_height)
        for box in box_values)
    scenario_tags = _scenario_tags(case['scenario_tags'])
    label_review_status = case['label_review_status']
    if (not isinstance(label_review_status, str) or
            label_review_status not in _REVIEW_STATUSES):
        raise ValueError('unsupported label_review_status')

    image_path = document_dir / relative_path
    if verify_images:
        _verify_image(image_path, image_sha256, image_width, image_height)
    return GroundingCase(
        case_id=case_id,
        split=split,
        image_path=image_path,
        image_sha256=image_sha256,
        session_id=session_id,
        physical_object_id=physical_object_id,
        query=query,
        target_present=target_present,
        boxes=boxes,
        scenario_tags=scenario_tags,
        label_review_status=label_review_status,
    )


def _verify_image(path: Path, expected_sha256: str,
                  width: int, height: int) -> None:
    try:
        if path.is_symlink() or not stat.S_ISREG(path.lstat().st_mode):
            raise ValueError('image must be a regular non-symlink file')
        if _sha256_file(path) != expected_sha256:
            raise ValueError('image sha256 does not match')
        image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    except OSError as error:
        raise ValueError('image must be a regular non-symlink file') from error
    if image is None:
        raise ValueError('image could not be decoded')
    decoded_height, decoded_width = image.shape[:2]
    if decoded_width != width or decoded_height != height:
        raise ValueError('decoded image dimensions do not match')


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


def _nonempty_string(value: object, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError('{} must be a non-empty string'.format(name))
    return value


def _positive_integer(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError('{} must be a positive integer'.format(name))
    return value


def _scenario_tags(value: object) -> Tuple[str, ...]:
    if not isinstance(value, list) or len(value) > 16:
        raise ValueError('scenario_tags must contain at most 16 entries')
    if (any(not isinstance(tag, str) or not tag for tag in value) or
            len(set(value)) != len(value)):
        raise ValueError('scenario_tags must be unique non-empty strings')
    return tuple(value)


def _is_finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float)) and not isinstance(value, bool) and
        math.isfinite(value))
