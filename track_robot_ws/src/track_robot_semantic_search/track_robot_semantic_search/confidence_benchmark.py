"""Offline YOLO-World distance and semantic-confusion calibration tools."""

import csv
import hashlib
import json
import math
import os
import platform
from pathlib import Path, PurePosixPath
import tempfile
import time

import cv2
import numpy as np

from .manifest import write_json_atomic


DATASET_SCHEMA_VERSION = 'semantic_confidence_dataset/1.0.0'
MATCH_IOU_THRESHOLD = 0.30
_KINDS = {'target', 'distractor', 'background'}
_REVIEW_STATUSES = {'human_authored', 'human_verified'}
_DISTANCES = (1.0, 2.0, 3.0, 4.0, 5.0)


def _sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _finite_number(value):
    return (
        isinstance(value, (int, float)) and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _safe_relative_path(value, name):
    if not isinstance(value, str) or not value:
        raise ValueError('{} must be a non-empty relative path'.format(name))
    path = PurePosixPath(value)
    if path.is_absolute() or '..' in path.parts or '\\' in value:
        raise ValueError('{} must be a safe relative path'.format(name))
    return value


def _bbox(value, width=None, height=None, name='bbox'):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError('{} must contain x, y, width, height'.format(name))
    if not all(_finite_number(item) for item in value):
        raise ValueError('{} values must be finite'.format(name))
    x, y, box_width, box_height = (float(item) for item in value)
    if x < 0.0 or y < 0.0 or box_width <= 0.0 or box_height <= 0.0:
        raise ValueError('{} dimensions are invalid'.format(name))
    if width is not None and height is not None and (
            x + box_width > width or y + box_height > height):
        raise ValueError('{} exceeds image bounds'.format(name))
    return [x, y, box_width, box_height]


def _validate_sample(sample, root, verify_files):
    required = {
        'sample_id', 'stamp_ns', 'image_relative_path', 'image_sha256',
        'depth_relative_path', 'depth_sha256', 'image_width', 'image_height',
        'image_frame_id', 'depth_frame_id',
    }
    if not isinstance(sample, dict) or not required.issubset(sample):
        raise ValueError('sample is missing required fields')
    if not isinstance(sample['sample_id'], str) or not sample['sample_id']:
        raise ValueError('sample_id must be non-empty')
    if (isinstance(sample['stamp_ns'], bool) or
            not isinstance(sample['stamp_ns'], int) or
            sample['stamp_ns'] <= 0):
        raise ValueError('sample stamp_ns must be positive')
    width = sample['image_width']
    height = sample['image_height']
    if (isinstance(width, bool) or not isinstance(width, int) or width <= 0 or
            isinstance(height, bool) or not isinstance(height, int) or
            height <= 0):
        raise ValueError('sample image dimensions are invalid')
    for key in ('image_frame_id', 'depth_frame_id'):
        if not isinstance(sample[key], str) or not sample[key]:
            raise ValueError('{} must be non-empty'.format(key))
    image_relative = _safe_relative_path(
        sample['image_relative_path'], 'image_relative_path')
    depth_relative = _safe_relative_path(
        sample['depth_relative_path'], 'depth_relative_path')
    for key in ('image_sha256', 'depth_sha256'):
        value = sample[key]
        if (not isinstance(value, str) or len(value) != 64 or any(
                character not in '0123456789abcdef'
                for character in value)):
            raise ValueError('{} must be lowercase sha256'.format(key))
    if not verify_files:
        return
    image_path = root / image_relative
    depth_path = root / depth_relative
    if image_path.is_symlink() or not image_path.is_file():
        raise ValueError('image must be a regular file')
    if _sha256_file(image_path) != sample['image_sha256']:
        raise ValueError('image sha256 does not match')
    if depth_path.is_symlink() or not depth_path.is_file():
        raise ValueError('depth must be a regular file')
    if _sha256_file(depth_path) != sample['depth_sha256']:
        raise ValueError('depth sha256 does not match')
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (height, width):
        raise ValueError('image dimensions do not match metadata')
    try:
        depth = np.load(str(depth_path), allow_pickle=False)
    except (OSError, ValueError) as error:
        raise ValueError('depth file is invalid') from error
    if depth.ndim != 2 or depth.shape != (height, width):
        raise ValueError('depth dimensions do not match image')


def _validate_trial(trial, root, verify_files):
    required = {
        'trial_id', 'ground_truth_kind', 'ground_truth_label',
        'nominal_distance_m', 'ground_truth_bbox_xywh',
        'label_review_status', 'notes', 'samples',
    }
    if not isinstance(trial, dict) or set(trial) != required:
        raise ValueError('trial fields do not match the dataset contract')
    if not isinstance(trial['trial_id'], str) or not trial['trial_id']:
        raise ValueError('trial_id must be non-empty')
    kind = trial['ground_truth_kind']
    if kind not in _KINDS:
        raise ValueError('ground_truth_kind is unsupported')
    if (not isinstance(trial['ground_truth_label'], str) or
            not trial['ground_truth_label']):
        raise ValueError('ground_truth_label must be non-empty')
    distance = trial['nominal_distance_m']
    if not _finite_number(distance) or not 0.2 <= float(distance) <= 20.0:
        raise ValueError('nominal_distance_m is invalid')
    if trial['label_review_status'] not in _REVIEW_STATUSES:
        raise ValueError(
            'label_review_status must be human authored or verified')
    if not isinstance(trial['notes'], str):
        raise ValueError('notes must be a string')
    samples = trial['samples']
    if not isinstance(samples, list) or not 1 <= len(samples) <= 1000:
        raise ValueError('trial samples must contain 1 to 1000 entries')
    sample_ids = set()
    for sample in samples:
        _validate_sample(sample, root, verify_files)
        if sample['sample_id'] in sample_ids:
            raise ValueError('duplicate sample_id in trial')
        sample_ids.add(sample['sample_id'])
    first = samples[0]
    _bbox(
        trial['ground_truth_bbox_xywh'],
        first['image_width'], first['image_height'],
        'ground_truth_bbox_xywh')


def _validate_dataset(document, root, verify_files):
    required = {
        'schema_version', 'dataset_id', 'query_text', 'created_at_utc',
        'provenance', 'trials',
    }
    if not isinstance(document, dict) or set(document) != required:
        raise ValueError('dataset fields do not match the contract')
    if document['schema_version'] != DATASET_SCHEMA_VERSION:
        raise ValueError('unsupported confidence dataset schema_version')
    for key in ('dataset_id', 'query_text', 'created_at_utc'):
        if not isinstance(document[key], str) or not document[key]:
            raise ValueError('{} must be non-empty'.format(key))
    provenance = document['provenance']
    provenance_keys = {
        'git_commit', 'ros_domain_id', 'image_topic', 'depth_topic'}
    if not isinstance(provenance, dict) or set(provenance) != provenance_keys:
        raise ValueError('dataset provenance fields do not match the contract')
    if (not isinstance(provenance['git_commit'], str) or
            not provenance['git_commit']):
        raise ValueError('git_commit must be non-empty')
    if (isinstance(provenance['ros_domain_id'], bool) or
            not isinstance(provenance['ros_domain_id'], int) or
            not 0 <= provenance['ros_domain_id'] <= 232):
        raise ValueError('ros_domain_id is invalid')
    for key in ('image_topic', 'depth_topic'):
        if (not isinstance(provenance[key], str) or
                not provenance[key].startswith('/')):
            raise ValueError('{} must be an absolute ROS topic'.format(key))
    trials = document['trials']
    if not isinstance(trials, list) or len(trials) > 10000:
        raise ValueError('trials must be a bounded array')
    trial_ids = set()
    sample_ids = set()
    for trial in trials:
        _validate_trial(trial, root, verify_files)
        if trial['trial_id'] in trial_ids:
            raise ValueError('duplicate trial_id')
        trial_ids.add(trial['trial_id'])
        for sample in trial['samples']:
            if sample['sample_id'] in sample_ids:
                raise ValueError('duplicate sample_id')
            sample_ids.add(sample['sample_id'])
    return document


def load_confidence_dataset(path, verify_files=True):
    """Load and strictly verify a controlled confidence dataset."""
    path = Path(path)
    try:
        document = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError('confidence dataset is invalid') from error
    return _validate_dataset(document, path.parent, bool(verify_files))


def append_trial(dataset_path, base_document, trial):
    """Atomically append one human-labelled static trial."""
    dataset_path = Path(dataset_path)
    if dataset_path.exists():
        document = load_confidence_dataset(dataset_path, verify_files=False)
    else:
        document = json.loads(json.dumps(base_document))
    if any(value['trial_id'] == trial.get('trial_id')
           for value in document.get('trials', ())):
        raise ValueError('duplicate trial_id')
    document['trials'].append(json.loads(json.dumps(trial)))
    _validate_dataset(document, dataset_path.parent, verify_files=True)
    dataset_path.parent.mkdir(parents=True, exist_ok=True)
    write_json_atomic(dataset_path, document)
    return document


def _iou(first, second):
    first = _bbox(first)
    second = _bbox(second)
    left = max(first[0], second[0])
    top = max(first[1], second[1])
    right = min(first[0] + first[2], second[0] + second[2])
    bottom = min(first[1] + first[3], second[1] + second[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    union = first[2] * first[3] + second[2] * second[3] - intersection
    return 0.0 if union <= 0.0 else intersection / union


def classify_candidate(trial_kind, ground_truth_bbox, candidate_bbox,
                       iou_threshold=MATCH_IOU_THRESHOLD):
    """Classify a candidate only through overlap with the human ROI."""
    if trial_kind not in _KINDS:
        raise ValueError('trial_kind is unsupported')
    overlap = _iou(ground_truth_bbox, candidate_bbox)
    if overlap < float(iou_threshold):
        return 'background', overlap
    if trial_kind == 'target':
        return 'target', overlap
    if trial_kind == 'distractor':
        return 'distractor', overlap
    return 'background', overlap


def estimate_candidate_depth(depth, bbox, inner_fraction=0.5,
                             minimum_samples=20,
                             minimum_depth_m=0.2,
                             maximum_depth_m=10.0):
    """Return robust median registered depth and valid-pixel fraction."""
    image = np.asarray(depth)
    if image.ndim != 2:
        raise ValueError('depth image must be two-dimensional')
    if (not _finite_number(inner_fraction) or
            not 0.0 < float(inner_fraction) <= 1.0 or
            isinstance(minimum_samples, bool) or
            not isinstance(minimum_samples, int) or minimum_samples <= 0):
        raise ValueError('depth sampling configuration is invalid')
    x, y, width, height = _bbox(bbox)
    center_x = x + 0.5 * width
    center_y = y + 0.5 * height
    inner_width = max(1, int(round(width * float(inner_fraction))))
    inner_height = max(1, int(round(height * float(inner_fraction))))
    left = max(0, int(math.floor(center_x - 0.5 * inner_width)))
    top = max(0, int(math.floor(center_y - 0.5 * inner_height)))
    right = min(image.shape[1], left + inner_width)
    bottom = min(image.shape[0], top + inner_height)
    if right <= left or bottom <= top:
        return None, 0.0
    values = image[top:bottom, left:right].astype(np.float64, copy=False)
    valid = (
        np.isfinite(values) & (values >= float(minimum_depth_m))
        & (values <= float(maximum_depth_m))
    )
    retained = values[valid]
    quality = float(retained.size) / float(values.size)
    if retained.size < minimum_samples:
        return None, quality
    return float(np.median(retained)), quality


def dataset_completeness(dataset, minimum_samples_per_cell=5,
                         minimum_distractor_classes=3):
    """Check the minimum 1-5 m target/hard-negative matrix."""
    cells = {}
    distractor_labels = set()
    for trial in dataset.get('trials', ()):
        kind = trial.get('ground_truth_kind')
        label = trial.get('ground_truth_label')
        distance = trial.get('nominal_distance_m')
        if not _finite_number(distance):
            continue
        rounded = float(round(float(distance)))
        if rounded not in _DISTANCES or abs(float(distance) - rounded) > 0.1:
            continue
        cells[(kind, label, rounded)] = cells.get(
            (kind, label, rounded), 0) + len(trial.get('samples', ()))
        if kind == 'distractor':
            distractor_labels.add(label)
    target_labels = {
        key[1] for key in cells if key[0] == 'target'}
    target_complete = any(all(
        cells.get(('target', label, distance), 0) >= minimum_samples_per_cell
        for distance in _DISTANCES) for label in target_labels)
    complete_distractors = sorted(
        label for label in distractor_labels if all(
            cells.get(('distractor', label, distance), 0) >=
            minimum_samples_per_cell for distance in _DISTANCES))
    complete = (
        target_complete and
        len(complete_distractors) >= minimum_distractor_classes)
    return {
        'status': (
            'COMPLETE' if complete
            else 'NOT_EVALUATED_INCOMPLETE_MATRIX'),
        'target_complete': target_complete,
        'complete_distractor_labels': complete_distractors,
        'minimum_samples_per_cell': minimum_samples_per_cell,
        'minimum_distractor_classes': minimum_distractor_classes,
    }


def _percentile(values, quantile):
    return (
        None if not values
        else float(np.percentile(np.asarray(values, dtype=np.float64),
                                 quantile * 100.0))
    )


def _candidate_groups(candidates):
    groups = {}
    for candidate in candidates:
        groups.setdefault(candidate['sample_id'], []).append(candidate)
    return groups


def _metrics_at(frames, groups, yolo_threshold, margin_threshold=None):
    target_total = 0
    target_accepts = 0
    distractor_total = 0
    distractor_accepts = 0
    far_total = 0
    far_accepts = 0
    for frame in frames:
        kind = frame['trial_ground_truth_kind']
        if kind not in ('target', 'distractor'):
            continue
        relevant_kind = 'target' if kind == 'target' else 'distractor'
        accepted = False
        for candidate in groups.get(frame['sample_id'], ()):
            if candidate.get('ground_truth_kind') != relevant_kind:
                continue
            if float(candidate['yolo_confidence']) < yolo_threshold:
                continue
            if margin_threshold is not None:
                margin = candidate.get('clip_margin')
                if margin is None or float(margin) < margin_threshold:
                    continue
            accepted = True
            break
        if kind == 'target':
            target_total += 1
            target_accepts += int(accepted)
            if float(frame['nominal_distance_m']) >= 4.0:
                far_total += 1
                far_accepts += int(accepted)
        else:
            distractor_total += 1
            distractor_accepts += int(accepted)
    precision_denominator = target_accepts + distractor_accepts
    return {
        'target_recall': (
            target_accepts / target_total if target_total else None),
        'hard_negative_false_accept_rate': (
            distractor_accepts / distractor_total
            if distractor_total else None),
        'precision': (
            target_accepts / precision_denominator
            if precision_denominator else None),
        'far_target_recall': (
            far_accepts / far_total if far_total else None),
        'target_frame_count': target_total,
        'distractor_frame_count': distractor_total,
    }


def _best_yolo_operating_point(frames, groups):
    points = []
    selected = None
    for index in range(101):
        threshold = index / 100.0
        metrics = _metrics_at(frames, groups, threshold)
        point = dict(metrics, threshold=threshold)
        points.append(point)
        if (metrics['target_recall'] is not None and
                metrics['hard_negative_false_accept_rate'] is not None and
                metrics['target_recall'] >= 0.90 and
                metrics['hard_negative_false_accept_rate'] <= 0.05):
            selected = point
    baseline_candidates = [
        point for point in points
        if point['far_target_recall'] is not None
        and point['far_target_recall'] >= 0.90]
    baseline = max(
        baseline_candidates,
        key=lambda value: (
            -1.0 if value['precision'] is None else value['precision'],
            value['threshold']),
        default=max(
            points,
            key=lambda value: (
                -1.0 if value['target_recall'] is None
                else value['target_recall'],
                -1.0 if value['precision'] is None else value['precision']),
        ),
    )
    return selected, baseline, points


def _best_clip_operating_point(frames, groups, baseline):
    margins = [
        float(candidate['clip_margin']) for values in groups.values()
        for candidate in values
        if candidate.get('ground_truth_kind') in ('target', 'distractor')
        and candidate.get('clip_margin') is not None
    ]
    if not margins:
        return {'available': False, 'reason': 'no_clip_margin_records'}
    far_gate = baseline.get('far_target_recall')
    far_gate = 0.0 if far_gate is None else float(far_gate)
    best = None
    margin_grid = sorted(set(
        [-1.0, 1.0] + [round(value, 4) for value in margins]))
    for yolo_index in range(101):
        yolo_threshold = yolo_index / 100.0
        for margin_threshold in margin_grid:
            metrics = _metrics_at(
                frames, groups, yolo_threshold, margin_threshold)
            if (metrics['far_target_recall'] is None or
                    metrics['far_target_recall'] + 1e-12 < far_gate):
                continue
            point = dict(
                metrics,
                yolo_threshold=yolo_threshold,
                clip_margin_threshold=margin_threshold,
            )
            if best is None or (
                    -1.0 if point['precision'] is None else point['precision'],
                    -point['hard_negative_false_accept_rate'],
                    point['target_recall'],
                    point['yolo_threshold'],
                    point['clip_margin_threshold'],
            ) > (
                    -1.0 if best['precision'] is None else best['precision'],
                    -best['hard_negative_false_accept_rate'],
                    best['target_recall'],
                    best['yolo_threshold'],
                    best['clip_margin_threshold'],
            ):
                best = point
    if best is None:
        return {
            'available': True,
            'improves_precision': False,
            'reason': 'no_operating_point_preserves_far_target_recall',
        }
    baseline_precision = baseline.get('precision')
    improves = (
        best['precision'] is not None and
        (baseline_precision is None or best['precision'] > baseline_precision)
    )
    return dict(best, available=True, improves_precision=improves)


def _rank(values):
    array = np.asarray(values, dtype=np.float64)
    order = np.argsort(array, kind='mergesort')
    ranks = np.empty(array.size, dtype=np.float64)
    start = 0
    while start < array.size:
        end = start + 1
        while end < array.size and array[order[end]] == array[order[start]]:
            end += 1
        ranks[order[start:end]] = 0.5 * (start + end - 1) + 1.0
        start = end
    return ranks


def _spearman(first, second):
    if len(first) < 2 or len(first) != len(second):
        return None
    first_rank = _rank(first)
    second_rank = _rank(second)
    if np.std(first_rank) <= 1e-12 or np.std(second_rank) <= 1e-12:
        return None
    return float(np.corrcoef(first_rank, second_rank)[0, 1])


def analyze_confidence_records(frames, candidates):
    """Compute descriptive separation and ROI-verification diagnostics."""
    frames = tuple(frames)
    candidates = tuple(candidates)
    groups = _candidate_groups(candidates)
    target_scores = [
        float(value['yolo_confidence']) for value in candidates
        if value.get('ground_truth_kind') == 'target']
    negative_scores = [
        float(value['yolo_confidence']) for value in candidates
        if value.get('ground_truth_kind') == 'distractor']
    overlap_lower = (
        max(min(target_scores), min(negative_scores))
        if target_scores and negative_scores else None)
    overlap_upper = (
        min(max(target_scores), max(negative_scores))
        if target_scores and negative_scores else None)
    selected, baseline, points = _best_yolo_operating_point(frames, groups)
    target_candidates = [
        value for value in candidates
        if value.get('ground_truth_kind') == 'target']
    distances = [
        float(value['estimated_3d_distance_m'])
        for value in target_candidates
        if value.get('estimated_3d_distance_m') is not None]
    distance_scores = [
        float(value['yolo_confidence'])
        for value in target_candidates
        if value.get('estimated_3d_distance_m') is not None]
    area_values = [
        math.log(max(float(value['bbox_area_px']), 1.0))
        for value in target_candidates]
    area_scores = [
        float(value['yolo_confidence']) for value in target_candidates]
    by_distance = {}
    for distance in _DISTANCES:
        distance_frames = [
            value for value in frames
            if value['trial_ground_truth_kind'] == 'target'
            and abs(float(value['nominal_distance_m']) - distance) <= 0.1]
        distance_candidates = [
            candidate for candidate in target_candidates
            if any(frame['sample_id'] == candidate['sample_id']
                   for frame in distance_frames)]
        scores = [float(value['yolo_confidence'])
                  for value in distance_candidates]
        areas = [float(value['bbox_area_px'])
                 for value in distance_candidates]
        detected_samples = {
            value['sample_id'] for value in distance_candidates}
        by_distance[str(int(distance))] = {
            'frame_count': len(distance_frames),
            'detected_frame_count': len(detected_samples),
            'detection_recall_at_floor': (
                len(detected_samples) / len(distance_frames)
                if distance_frames else None),
            'confidence_p50': _percentile(scores, 0.50),
            'confidence_p95': _percentile(scores, 0.95),
            'bbox_area_p50_px': _percentile(areas, 0.50),
        }
    return {
        'frame_count': len(frames),
        'candidate_count': len(candidates),
        'score_overlap': {
            'overlap': (
                overlap_lower is not None and
                overlap_upper is not None and
                overlap_lower <= overlap_upper),
            'lower': overlap_lower,
            'upper': overlap_upper,
            'target_min': min(target_scores) if target_scores else None,
            'target_max': max(target_scores) if target_scores else None,
            'distractor_min': (
                min(negative_scores) if negative_scores else None),
            'distractor_max': (
                max(negative_scores) if negative_scores else None),
        },
        'single_yolo_threshold': (
            dict(selected, separable=True) if selected is not None
            else {'separable': False, 'threshold': None}),
        'best_descriptive_yolo_point': baseline,
        'roi_verification': _best_clip_operating_point(
            frames, groups, baseline),
        'target_correlations': {
            'confidence_vs_estimated_distance_spearman': _spearman(
                distances, distance_scores),
            'confidence_vs_log_bbox_area_spearman': _spearman(
                area_values, area_scores),
        },
        'target_by_nominal_distance_m': by_distance,
        'threshold_scan': points,
    }


def _crop(image, bbox):
    x, y, width, height = _bbox(
        bbox, image.shape[1], image.shape[0], 'candidate bbox')
    left = max(0, int(math.floor(x)))
    top = max(0, int(math.floor(y)))
    right = min(image.shape[1], int(math.ceil(x + width)))
    bottom = min(image.shape[0], int(math.ceil(y + height)))
    if right <= left or bottom <= top:
        raise ValueError('candidate crop is empty')
    return image[top:bottom, left:right].copy()


def _write_jsonl_atomic(path, records):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=path.name + '.', suffix='.tmp', dir=str(path.parent))
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            for record in records:
                stream.write(json.dumps(
                    record, sort_keys=True, ensure_ascii=True,
                    allow_nan=False) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _optional_score(scorer, crop):
    if scorer is None:
        return {}
    value = scorer(crop)
    if not isinstance(value, dict):
        raise ValueError('ROI scorer must return a mapping')
    return value


def run_offline_inference(dataset_path, output_dir, yolo_backend,
                          clip_scorer=None, dino_scorer=None,
                          run_provenance=None):
    """Run the production-shared YOLO backend on frozen ZED frames."""
    dataset_path = Path(dataset_path)
    output_dir = Path(output_dir)
    dataset = load_confidence_dataset(dataset_path, verify_files=True)
    output_dir.mkdir(parents=True, exist_ok=True)
    crops_dir = output_dir / 'crops'
    truth_dir = output_dir / 'ground_truth_crops'
    crops_dir.mkdir(exist_ok=True)
    truth_dir.mkdir(exist_ok=True)
    frames = []
    candidates = []
    latencies = []
    for trial in dataset['trials']:
        truth_bbox = trial['ground_truth_bbox_xywh']
        for sample in trial['samples']:
            image_path = dataset_path.parent / sample['image_relative_path']
            depth_path = dataset_path.parent / sample['depth_relative_path']
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('sample image could not be decoded')
            depth = np.load(str(depth_path), allow_pickle=False)
            truth_crop = _crop(image, truth_bbox)
            truth_relative = Path('ground_truth_crops') / '{}.png'.format(
                sample['sample_id'])
            if not cv2.imwrite(str(output_dir / truth_relative), truth_crop):
                raise OSError('unable to write ground-truth crop')
            truth_depth, truth_quality = estimate_candidate_depth(
                depth, truth_bbox)
            started = time.monotonic()
            detections = tuple(yolo_backend.predict(
                image, dataset['query_text']))
            synchronize = getattr(yolo_backend, 'synchronize', None)
            if synchronize is not None:
                synchronize()
            yolo_latency_ms = (time.monotonic() - started) * 1000.0
            latencies.append(yolo_latency_ms)
            matched = 0
            for index, detection in enumerate(detections):
                bbox = [
                    float(detection.x1), float(detection.y1),
                    float(detection.x2 - detection.x1),
                    float(detection.y2 - detection.y1),
                ]
                candidate_kind, overlap = classify_candidate(
                    trial['ground_truth_kind'], truth_bbox, bbox)
                matched += int(candidate_kind in ('target', 'distractor'))
                crop = _crop(image, bbox)
                candidate_id = '{}-det{:03d}'.format(
                    sample['sample_id'], index)
                crop_relative = Path('crops') / '{}.png'.format(candidate_id)
                crop_path = output_dir / crop_relative
                if not cv2.imwrite(str(crop_path), crop):
                    raise OSError('unable to write candidate crop')
                distance_m, depth_quality = estimate_candidate_depth(
                    depth, bbox)
                clip_values = _optional_score(clip_scorer, crop)
                dino_values = _optional_score(dino_scorer, crop)
                record = {
                    'sample_id': sample['sample_id'],
                    'trial_id': trial['trial_id'],
                    'candidate_id': candidate_id,
                    'trial_ground_truth_kind': trial['ground_truth_kind'],
                    'ground_truth_kind': candidate_kind,
                    'ground_truth_label': trial['ground_truth_label'],
                    'ground_truth_iou': overlap,
                    'nominal_distance_m': float(trial['nominal_distance_m']),
                    'yolo_confidence': float(detection.score),
                    'bbox_xywh': bbox,
                    'bbox_width_px': bbox[2],
                    'bbox_height_px': bbox[3],
                    'bbox_area_px': bbox[2] * bbox[3],
                    'bbox_area_fraction': (
                        bbox[2] * bbox[3] /
                        float(image.shape[0] * image.shape[1])),
                    'estimated_3d_distance_m': distance_m,
                    'depth_valid_fraction': depth_quality,
                    'roi_crop_relative_path': str(crop_relative),
                    'roi_crop_sha256': _sha256_file(crop_path),
                    'clip_positive_similarity': clip_values.get(
                        'clip_positive_similarity'),
                    'clip_hard_negative_max_similarity': clip_values.get(
                        'clip_hard_negative_max_similarity'),
                    'clip_margin': clip_values.get('clip_margin'),
                    'clip_inference_ms': clip_values.get('clip_inference_ms'),
                    'dino_similarity': dino_values.get('dino_similarity'),
                    'dino_inference_ms': dino_values.get('dino_inference_ms'),
                    'yolo_inference_ms': yolo_latency_ms,
                }
                candidates.append(record)
            frames.append({
                'sample_id': sample['sample_id'],
                'trial_id': trial['trial_id'],
                'trial_ground_truth_kind': trial['ground_truth_kind'],
                'ground_truth_label': trial['ground_truth_label'],
                'nominal_distance_m': float(trial['nominal_distance_m']),
                'ground_truth_bbox_xywh': list(truth_bbox),
                'ground_truth_depth_m': truth_depth,
                'ground_truth_depth_quality': truth_quality,
                'ground_truth_crop_relative_path': str(truth_relative),
                'image_relative_path': sample['image_relative_path'],
                'matched_candidate_count': matched,
                'candidate_count': len(detections),
                'yolo_inference_ms': yolo_latency_ms,
            })
    _write_jsonl_atomic(output_dir / 'frames.jsonl', frames)
    _write_jsonl_atomic(output_dir / 'candidates.jsonl', candidates)
    memory = getattr(yolo_backend, 'incremental_cuda_reserved_mib', None)
    if run_provenance is None:
        run_provenance = {}
    if not isinstance(run_provenance, dict):
        raise ValueError('run_provenance must be a mapping')
    try:
        provenance = json.loads(json.dumps(
            run_provenance, sort_keys=True, allow_nan=False))
    except (TypeError, ValueError) as error:
        raise ValueError('run_provenance must be finite JSON data') from error
    run = {
        'schema_version': 'semantic_confidence_run/1.0.0',
        'dataset_path': str(dataset_path.resolve()),
        'dataset_sha256': _sha256_file(dataset_path),
        'query_text': dataset['query_text'],
        'frame_count': len(frames),
        'candidate_count': len(candidates),
        'yolo_latency_p50_ms': _percentile(latencies, 0.50),
        'yolo_latency_p95_ms': _percentile(latencies, 0.95),
        'incremental_cuda_reserved_mib': (
            float(memory()) if memory is not None else None),
        'clip_status': 'enabled' if clip_scorer is not None else 'disabled',
        'dino_status': 'enabled' if dino_scorer is not None else 'disabled',
        'provenance': provenance,
        'environment': {
            'python': platform.python_version(),
            'numpy': np.__version__,
            'opencv': cv2.__version__,
        },
    }
    write_json_atomic(output_dir / 'run.json', run)
    return run


def load_jsonl(path):
    records = []
    with Path(path).open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as error:
                raise ValueError(
                    'invalid JSONL at line {}'.format(line_number)) from error
    return records


def _plot_records(frames, candidates, source_root, output_dir, summary):
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/track_robot_matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    colours = {'target': '#2ca02c', 'distractor': '#d62728'}
    relevant = [
        value for value in candidates
        if value.get('ground_truth_kind') in colours]

    figure, axis = plt.subplots(figsize=(8, 5))
    for kind in ('target', 'distractor'):
        values = [value for value in relevant
                  if value['ground_truth_kind'] == kind]
        axis.scatter(
            [value.get('estimated_3d_distance_m') or
             value['nominal_distance_m'] for value in values],
            [value['yolo_confidence'] for value in values],
            label=kind, alpha=0.7, color=colours[kind])
    axis.set_xlabel('Distance (m)')
    axis.set_ylabel('YOLO-World confidence')
    axis.set_ylim(0.0, 1.0)
    axis.grid(True, alpha=0.25)
    axis.legend(loc='best')
    figure.tight_layout()
    figure.savefig(output_dir / 'confidence_vs_distance.png', dpi=140)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(8, 5))
    for kind in ('target', 'distractor'):
        values = [value for value in relevant
                  if value['ground_truth_kind'] == kind]
        axis.scatter(
            [value['bbox_area_px'] for value in values],
            [value['yolo_confidence'] for value in values],
            label=kind, alpha=0.7, color=colours[kind])
    axis.set_xscale('log')
    axis.set_xlabel('Bounding-box area (pixels, log scale)')
    axis.set_ylabel('YOLO-World confidence')
    axis.set_ylim(0.0, 1.0)
    axis.grid(True, alpha=0.25)
    axis.legend(loc='best')
    figure.tight_layout()
    figure.savefig(output_dir / 'confidence_vs_bbox_area.png', dpi=140)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(8, 5))
    bins = np.linspace(0.0, 1.0, 31)
    for kind in ('target', 'distractor'):
        scores = [value['yolo_confidence'] for value in relevant
                  if value['ground_truth_kind'] == kind]
        axis.hist(scores, bins=bins, alpha=0.55, label=kind,
                  color=colours[kind])
    axis.set_xlabel('YOLO-World confidence')
    axis.set_ylabel('Candidate count')
    axis.legend(loc='best')
    figure.tight_layout()
    figure.savefig(output_dir / 'score_distributions.png', dpi=140)
    plt.close(figure)

    threshold = summary['single_yolo_threshold'].get('threshold')
    if threshold is None:
        threshold = summary['best_descriptive_yolo_point'].get(
            'threshold', 0.0)
    failures = [
        value for value in relevant
        if ((value['ground_truth_kind'] == 'distractor' and
             value['yolo_confidence'] >= threshold) or
            (value['ground_truth_kind'] == 'target' and
             value['yolo_confidence'] < threshold))]
    failures.sort(key=lambda value: (
        value['ground_truth_kind'] != 'distractor',
        -value['yolo_confidence']))
    missing_ids = {
        frame['sample_id'] for frame in frames
        if frame['trial_ground_truth_kind'] == 'target'
        and not frame.get('matched_candidate_count')}
    failure_images = []
    for value in failures[:6]:
        path = source_root / value.get('roi_crop_relative_path', '')
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is not None:
            failure_images.append((image, '{} {:.3f}'.format(
                value['ground_truth_kind'], value['yolo_confidence'])))
    for frame in frames:
        if frame['sample_id'] not in missing_ids:
            continue
        path = source_root / frame.get('ground_truth_crop_relative_path', '')
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is not None:
            failure_images.append((image, 'target MISS'))
        if len(failure_images) >= 8:
            break
    figure, axes = plt.subplots(2, 4, figsize=(12, 6))
    for axis in axes.flat:
        axis.axis('off')
    for axis, (image, title) in zip(axes.flat, failure_images):
        axis.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        axis.set_title(title, fontsize=9)
        axis.axis('off')
    figure.suptitle('Diagnostic failure cases at threshold {:.2f}'.format(
        threshold))
    figure.tight_layout()
    figure.savefig(output_dir / 'failure_cases.png', dpi=140)
    plt.close(figure)

    margins_available = any(
        value.get('clip_margin') is not None for value in relevant)
    if margins_available:
        figure, axis = plt.subplots(figsize=(8, 5))
        for kind in ('target', 'distractor'):
            margins = [value['clip_margin'] for value in relevant
                       if value['ground_truth_kind'] == kind
                       and value.get('clip_margin') is not None]
            axis.hist(margins, bins=24, alpha=0.55, label=kind,
                      color=colours[kind])
        axis.set_xlabel('CLIP positive - hardest-negative similarity margin')
        axis.set_ylabel('Candidate count')
        axis.legend(loc='best')
        figure.tight_layout()
        figure.savefig(
            output_dir / 'roi_margin_distributions.png', dpi=140)
        plt.close(figure)


def _markdown_report(summary, completeness):
    overlap = summary['score_overlap']
    threshold = summary['single_yolo_threshold']
    lines = [
        '# YOLO-World `green bottle` 受控置信度实验',
        '',
        '评估状态：**{}**'.format(summary['evaluation_status']),
        '',
        '这是离线受控诊断，不修改生产阈值或感知管线。',
        '',
        '## 数据完整性',
        '',
        '- 矩阵状态：`{}`'.format(completeness['status']),
        '- 帧数：{}'.format(summary['frame_count']),
        '- 检测候选数：{}'.format(summary['candidate_count']),
        '',
        '## 主要诊断结果',
        '',
        '- 目标/困难负样本分数是否重叠：`{}`'.format(
            overlap['overlap']),
        '- 重叠区间：`{}` 至 `{}`'.format(
            overlap['lower'], overlap['upper']),
        '- 是否有单一 YOLO 阈值满足诊断分离门槛：`{}`'.format(
            threshold['separable']),
        '- 诊断阈值：`{}`'.format(
            threshold.get('threshold')),
        '- ROI 语义验证是否有数据：`{}`'.format(
            summary['roi_verification']['available']),
        '- ROI 验证在保持远距离召回时是否提高 precision：`{}`'.format(
            summary['roi_verification'].get('improves_precision')),
        '',
        '## 距离分组',
        '',
        '| 名义距离 | 帧数 | 检出帧数 | floor 下召回 | confidence P50 | '
        'bbox area P50 |',
        '|---:|---:|---:|---:|---:|---:|',
    ]
    for distance, values in summary[
            'target_by_nominal_distance_m'].items():
        lines.append(
            '| {} m | {} | {} | {} | {} | {} |'.format(
                distance, values['frame_count'],
                values['detected_frame_count'],
                values['detection_recall_at_floor'],
                values['confidence_p50'], values['bbox_area_p50_px']))
    lines.extend([
        '',
        '## 相关性',
        '',
        '- confidence 对估计距离 Spearman：`{}`'.format(
            summary['target_correlations'][
                'confidence_vs_estimated_distance_spearman']),
        '- confidence 对 log(bbox area) Spearman：`{}`'.format(
            summary['target_correlations'][
                'confidence_vs_log_bbox_area_spearman']),
        '',
        '## 解释边界',
        '',
        '同一静态 burst 内的帧高度相关。只有完整矩阵才可判断距离特征损失与语义'
        '混淆；生产参数必须用独立 session 再验证。',
        '',
    ])
    return '\n'.join(lines)


def write_confidence_report(frames, candidates, source_root, output_dir,
                            completeness):
    """Write metrics, plots, crop review and a short report."""
    output_dir = Path(output_dir)
    source_root = Path(source_root)
    output_dir.mkdir(parents=True, exist_ok=True)
    summary = analyze_confidence_records(frames, candidates)
    summary['dataset_completeness'] = dict(completeness)
    summary['evaluation_status'] = (
        'EVALUATED_CONTROLLED_DIAGNOSTIC'
        if completeness.get('status') == 'COMPLETE'
        else 'NOT_EVALUATED_INCOMPLETE_MATRIX')

    field_names = sorted(set(
        key for value in candidates for key in value.keys())) or [
            'sample_id', 'ground_truth_kind', 'yolo_confidence']
    with (output_dir / 'samples.csv').open(
            'w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=field_names)
        writer.writeheader()
        for value in candidates:
            writer.writerow({
                key: (json.dumps(item, ensure_ascii=True)
                      if isinstance(item, (list, dict)) else item)
                for key, item in value.items()})
    _plot_records(
        tuple(frames), tuple(candidates), source_root, output_dir, summary)
    write_json_atomic(output_dir / 'summary.json', summary)
    (output_dir / 'report.md').write_text(
        _markdown_report(summary, completeness), encoding='utf-8')
    return summary
