import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from track_robot_semantic_search.confidence_benchmark import (
    append_trial,
    analyze_confidence_records,
    classify_candidate,
    dataset_completeness,
    estimate_candidate_depth,
    load_confidence_dataset,
    run_offline_inference,
    write_confidence_report,
)
from track_robot_semantic_search.yolo_world_backend import GroundedDetection


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _sample_files(root, sample_id, colour=(0, 160, 0)):
    image_path = root / 'images' / '{}.png'.format(sample_id)
    depth_path = root / 'depth' / '{}.npy'.format(sample_id)
    image_path.parent.mkdir(parents=True, exist_ok=True)
    depth_path.parent.mkdir(parents=True, exist_ok=True)
    image = np.zeros((80, 120, 3), dtype=np.uint8)
    image[20:60, 45:75] = colour
    depth = np.full((80, 120), 2.0, dtype=np.float32)
    assert cv2.imwrite(str(image_path), image)
    np.save(str(depth_path), depth, allow_pickle=False)
    return {
        'sample_id': sample_id,
        'stamp_ns': 1_000_000_000,
        'image_relative_path': str(image_path.relative_to(root)),
        'image_sha256': _sha256(image_path),
        'depth_relative_path': str(depth_path.relative_to(root)),
        'depth_sha256': _sha256(depth_path),
        'image_width': 120,
        'image_height': 80,
        'image_frame_id': 'zed_left_camera_optical_frame',
        'depth_frame_id': 'zed_left_camera_optical_frame',
    }


def _new_dataset(root):
    return {
        'schema_version': 'semantic_confidence_dataset/1.0.0',
        'dataset_id': 'green-bottle-controlled',
        'query_text': 'green bottle',
        'created_at_utc': '2026-08-08T10:00:00Z',
        'provenance': {
            'git_commit': 'a' * 40,
            'ros_domain_id': 20,
            'image_topic': '/zed/zed_node/left/image_rect_color',
            'depth_topic': '/zed/zed_node/depth/depth_registered',
        },
        'trials': [],
    }


def _trial(root, trial_id, kind='target', label='green_bottle', distance=2.0):
    return {
        'trial_id': trial_id,
        'ground_truth_kind': kind,
        'ground_truth_label': label,
        'nominal_distance_m': distance,
        'ground_truth_bbox_xywh': [45.0, 20.0, 30.0, 40.0],
        'label_review_status': 'human_authored',
        'notes': 'static controlled fixture',
        'samples': [_sample_files(root, '{}-000'.format(trial_id))],
    }


def test_dataset_append_and_load_verify_rgb_depth_hashes(tmp_path):
    dataset_path = tmp_path / 'dataset.json'
    append_trial(
        dataset_path, _new_dataset(tmp_path),
        _trial(tmp_path, 'target-2m'))

    dataset = load_confidence_dataset(dataset_path, verify_files=True)

    assert dataset['query_text'] == 'green bottle'
    assert dataset['trials'][0]['nominal_distance_m'] == 2.0
    depth_path = tmp_path / dataset['trials'][0]['samples'][0][
        'depth_relative_path']
    depth_path.write_bytes(b'changed')
    with pytest.raises(ValueError, match='depth sha256'):
        load_confidence_dataset(dataset_path, verify_files=True)


def test_append_rejects_duplicate_trial_and_nonhuman_roi(tmp_path):
    dataset_path = tmp_path / 'dataset.json'
    base = _new_dataset(tmp_path)
    trial = _trial(tmp_path, 'target-2m')
    append_trial(dataset_path, base, trial)

    with pytest.raises(ValueError, match='duplicate trial_id'):
        append_trial(dataset_path, base, trial)
    trial['label_review_status'] = 'model_generated'
    with pytest.raises(ValueError, match='label_review_status'):
        append_trial(tmp_path / 'other.json', _new_dataset(tmp_path), trial)


def test_candidate_classification_uses_human_roi_iou():
    gt = [10.0, 10.0, 20.0, 20.0]

    assert classify_candidate('target', gt, [11.0, 11.0, 18.0, 18.0]) == (
        'target', pytest.approx(0.81))
    kind, iou = classify_candidate(
        'distractor', gt, [11.0, 11.0, 18.0, 18.0])
    assert kind == 'distractor'
    assert iou == pytest.approx(0.81)
    assert classify_candidate(
        'target', gt, [60.0, 60.0, 10.0, 10.0])[0] == 'background'


def test_depth_estimate_uses_inner_roi_median_and_quality():
    depth = np.zeros((20, 20), dtype=np.float32)
    depth[7:13, 7:13] = 3.25

    distance, quality = estimate_candidate_depth(
        depth, [4.0, 4.0, 12.0, 12.0], inner_fraction=0.5,
        minimum_samples=20)

    assert distance == pytest.approx(3.25)
    assert quality == pytest.approx(1.0)
    assert estimate_candidate_depth(
        np.zeros((20, 20), dtype=np.float32),
        [4.0, 4.0, 12.0, 12.0], minimum_samples=20) == (None, 0.0)


def test_completeness_requires_distance_matrix_and_hard_negatives():
    trials = []
    for distance in (1, 2, 3, 4, 5):
        trials.append({
            'trial_id': 'target-{}'.format(distance),
            'ground_truth_kind': 'target',
            'ground_truth_label': 'green_bottle',
            'nominal_distance_m': float(distance),
            'samples': [{}] * 5,
        })
        for label in ('green_box', 'green_tissue_box', 'yellow_cylinder'):
            trials.append({
                'trial_id': '{}-{}'.format(label, distance),
                'ground_truth_kind': 'distractor',
                'ground_truth_label': label,
                'nominal_distance_m': float(distance),
                'samples': [{}] * 5,
            })

    complete = dataset_completeness({'trials': trials})
    assert complete['status'] == 'COMPLETE'
    incomplete = dataset_completeness({'trials': trials[:-1]})
    assert incomplete['status'] == 'NOT_EVALUATED_INCOMPLETE_MATRIX'


def _frame(sample_id, kind, distance, matched):
    return {
        'sample_id': sample_id,
        'trial_ground_truth_kind': kind,
        'ground_truth_label': (
            'green_bottle' if kind == 'target' else 'green_box'),
        'nominal_distance_m': float(distance),
        'matched_candidate_count': int(matched),
        'ground_truth_depth_m': float(distance),
    }


def _candidate(sample_id, kind, score, area, distance, margin=None):
    return {
        'sample_id': sample_id,
        'ground_truth_kind': kind,
        'yolo_confidence': float(score),
        'bbox_area_px': float(area),
        'estimated_3d_distance_m': float(distance),
        'clip_margin': margin,
        'roi_crop_relative_path': 'crops/{}.png'.format(sample_id),
    }


def test_threshold_analysis_reports_overlap_and_no_single_separator():
    frames = [
        _frame('t4', 'target', 4, True),
        _frame('t5', 'target', 5, True),
        _frame('n1', 'distractor', 1, True),
        _frame('n2', 'distractor', 2, True),
    ]
    candidates = [
        _candidate('t4', 'target', 0.25, 450, 4),
        _candidate('t5', 'target', 0.30, 300, 5),
        _candidate('n1', 'distractor', 0.28, 500, 1),
        _candidate('n2', 'distractor', 0.29, 400, 2),
    ]

    summary = analyze_confidence_records(frames, candidates)

    assert summary['score_overlap']['overlap'] is True
    assert summary['single_yolo_threshold']['separable'] is False
    assert summary['single_yolo_threshold']['threshold'] is None


def test_clip_margin_scan_preserves_far_recall_and_rejects_distractors():
    frames = [
        _frame('t4', 'target', 4, True),
        _frame('t5', 'target', 5, True),
        _frame('n1', 'distractor', 1, True),
        _frame('n2', 'distractor', 2, True),
    ]
    candidates = [
        _candidate('t4', 'target', 0.25, 450, 4, 0.18),
        _candidate('t5', 'target', 0.30, 300, 5, 0.12),
        _candidate('n1', 'distractor', 0.28, 500, 1, -0.05),
        _candidate('n2', 'distractor', 0.29, 400, 2, 0.01),
    ]

    summary = analyze_confidence_records(frames, candidates)

    assert summary['roi_verification']['available'] is True
    assert summary['roi_verification']['improves_precision'] is True
    assert summary['roi_verification'][
        'far_target_recall'] == pytest.approx(1.0)


def test_clip_margin_report_requires_hard_negative_frames():
    frames = [
        _frame('t4', 'target', 4, True),
        _frame('t5', 'target', 5, True),
    ]
    candidates = [
        _candidate('t4', 'target', 0.25, 450, 4, 0.18),
        _candidate('t5', 'target', 0.30, 300, 5, 0.12),
    ]

    summary = analyze_confidence_records(frames, candidates)

    assert summary['roi_verification'] == {
        'available': False,
        'reason': 'no_hard_negative_frames',
    }


class _FakeYolo:
    def predict(self, image, query):
        assert query == 'green bottle'
        if 'miss' in str(image[0, 0, 0]):
            return ()
        return (GroundedDetection(
            x1=45.0, y1=20.0, x2=75.0, y2=60.0,
            score=0.27, label=query),)

    def synchronize(self):
        return None

    def incremental_cuda_reserved_mib(self):
        return 12.5


def test_offline_inference_records_candidate_geometry_depth_and_crop(tmp_path):
    dataset_path = tmp_path / 'dataset.json'
    append_trial(
        dataset_path, _new_dataset(tmp_path),
        _trial(tmp_path, 'target-2m'))
    output = tmp_path / 'run'

    result = run_offline_inference(
        dataset_path, output, yolo_backend=_FakeYolo(),
        run_provenance={
            'git_commit': 'b' * 40,
            'world_checkpoint_sha256': 'c' * 64,
            'input_size': 640,
        })

    assert result['frame_count'] == 1
    assert result['candidate_count'] == 1
    assert result['provenance']['git_commit'] == 'b' * 40
    assert result['environment']['opencv'] == cv2.__version__
    candidate = json.loads((output / 'candidates.jsonl').read_text().strip())
    assert candidate['yolo_confidence'] == pytest.approx(0.27)
    assert candidate['bbox_width_px'] == pytest.approx(30.0)
    assert candidate['bbox_height_px'] == pytest.approx(40.0)
    assert candidate['bbox_area_px'] == pytest.approx(1200.0)
    assert candidate['estimated_3d_distance_m'] == pytest.approx(2.0)
    crop = output / candidate['roi_crop_relative_path']
    assert crop.is_file()
    assert cv2.imread(str(crop)).shape[:2] == (40, 30)


def test_report_writes_required_plots_csv_json_and_markdown(tmp_path):
    image_path = tmp_path / 'crop.png'
    assert cv2.imwrite(str(image_path), np.zeros((20, 20, 3), np.uint8))
    frames = [
        _frame('t4', 'target', 4, True),
        _frame('n1', 'distractor', 1, True),
    ]
    candidates = [
        dict(_candidate('t4', 'target', 0.25, 400, 4, 0.1),
             roi_crop_relative_path='crop.png'),
        dict(_candidate('n1', 'distractor', 0.28, 500, 1, -0.1),
             roi_crop_relative_path='crop.png'),
    ]
    output = tmp_path / 'report'

    summary = write_confidence_report(
        frames, candidates, source_root=tmp_path, output_dir=output,
        completeness={'status': 'NOT_EVALUATED_INCOMPLETE_MATRIX'})

    assert summary['evaluation_status'] == 'NOT_EVALUATED_INCOMPLETE_MATRIX'
    for name in (
            'confidence_vs_distance.png',
            'confidence_vs_bbox_area.png',
            'score_distributions.png',
            'failure_cases.png',
            'roi_margin_distributions.png',
            'samples.csv', 'summary.json', 'report.md'):
        assert (output / name).is_file(), name
    assert 'NOT_EVALUATED_INCOMPLETE_MATRIX' in (
        output / 'report.md').read_text(encoding='utf-8')
