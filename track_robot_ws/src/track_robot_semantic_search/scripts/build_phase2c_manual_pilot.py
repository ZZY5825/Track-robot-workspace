#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path


# Human-reviewed image boxes in source-image pixels. Each selected LiDAR
# tracklet is the one whose projected returns belong to that person. Frames
# with no person, ambiguous projection, or duplicate source time are omitted.
MANUAL_SELECTIONS = {
    'primary': {
        2: (13, [590, 70, 710, 660]),
        6: (21, [600, 40, 790, 650]),
        7: (13, [560, 80, 720, 650]),
        10: (13, [560, 80, 730, 650]),
        11: (28, [760, 0, 950, 650]),
        12: (28, [980, 0, 1280, 720]),
        13: (28, [980, 0, 1280, 680]),
        14: (28, [720, 0, 930, 650]),
        16: (54, [980, 0, 1280, 680]),
        26: (82, [990, 40, 1230, 660]),
        27: (82, [740, 70, 900, 650]),
        28: (13, [590, 60, 700, 660]),
        29: (13, [590, 60, 700, 660]),
    },
    'q50': {
        3: (13, [590, 60, 720, 650]),
        4: (13, [560, 40, 720, 650]),
        8: (13, [620, 40, 760, 650]),
        9: (28, [850, 0, 1220, 700]),
        10: (28, [970, 0, 1280, 720]),
        12: (13, [560, 0, 720, 680]),
        13: (54, [1000, 0, 1280, 700]),
        21: (82, [850, 70, 1040, 650]),
        22: (13, [560, 60, 700, 660]),
    },
}

CALIBRATED_SIZE_RATIO_MAXIMUM = 40.0


def read_jsonl(path):
    with Path(path).open('r', encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def area(box):
    return max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])


def intersection(first, second):
    return [
        max(first[0], second[0]), max(first[1], second[1]),
        min(first[2], second[2]), min(first[3], second[3])]


def center_distance(first, second):
    return math.hypot(
        0.5 * (first[0] + first[2] - second[0] - second[2]),
        0.5 * (first[1] + first[3] - second[1] - second[3]))


def geometry_values(projected, manual):
    overlap = area(intersection(projected, manual))
    projected_area = area(projected)
    manual_area = area(manual)
    union = projected_area + manual_area - overlap
    ratio = manual_area / projected_area
    return {
        'projected_centroid': center_distance(projected, manual),
        'inside_fraction': overlap / projected_area,
        'projected_iou': overlap / union if union > 0.0 else 0.0,
        'extent_consistency': min(ratio, 1.0 / ratio),
        'size_ratio': ratio,
    }


def update_term(term, values):
    name = term['name']
    if name not in values:
        return
    raw = float(values[name])
    term['raw_value'] = raw
    term['valid'] = True
    if name == 'size_ratio':
        passed = 0.25 <= raw <= CALIBRATED_SIZE_RATIO_MAXIMUM
        term['gate_passed'] = passed
        term['normalized_value'] = max(
            0.0, min(1.0, (raw - 0.25) /
                         (CALIBRATED_SIZE_RATIO_MAXIMUM - 0.25)))
        term['contribution'] = 0.0
        return
    if name == 'projected_centroid':
        normalized = 1.0 - max(0.0, min(1.0, raw / 200.0))
    else:
        normalized = max(0.0, min(1.0, raw))
    term['gate_passed'] = True
    term['normalized_value'] = normalized
    term['contribution'] = normalized * float(term['weight'])


def build_source(name, debug_path, manifest_path):
    debug = read_jsonl(debug_path)
    by_pair = {
        (int(row['visual_candidate_id']), int(row['lidar_tracklet_id'])): row
        for row in debug
    }
    manifest = json.loads(Path(manifest_path).read_text(encoding='utf-8'))
    candidates = {
        int(row['visual_candidate_id']): row
        for row in manifest['candidates']
    }
    samples = []
    annotations = []
    decisions = []
    for candidate_id, (human_tracklet_id, manual_box) in sorted(
            MANUAL_SELECTIONS[name].items()):
        candidate = candidates[candidate_id]
        projected = {
            int(row['tracklet_id']): row
            for row in candidate['projected_tracklets']
        }
        if human_tracklet_id not in projected:
            raise ValueError(
                '{} candidate {} is missing selected human tracklet {}'.format(
                    name, candidate_id, human_tracklet_id))
        positive_pair_id = None
        for tracklet_id, projection in sorted(projected.items()):
            original = by_pair.get((candidate_id, tracklet_id))
            if original is None:
                raise ValueError(
                    '{} candidate {} tracklet {} lacks debug evidence'.format(
                        name, candidate_id, tracklet_id))
            row = json.loads(json.dumps(original))
            row['pair_id'] = 'manual_roi_v1:{}:{}'.format(name, row['pair_id'])
            values = geometry_values(
                projection['projected_box'], manual_box)
            for term in row['terms']:
                update_term(term, values)
            row['total_score'] = sum(
                float(term['contribution']) for term in row['terms']
                if term['valid'] and not term['hard_gate'] and
                term['contribution'] is not None)
            size_term = next(
                term for term in row['terms'] if term['name'] == 'size_ratio')
            hard_gates_passed = all(
                term['gate_passed'] for term in row['terms']
                if term['valid'] and term['hard_gate'])
            row['decision'] = (
                'unmatched' if hard_gates_passed else 'rejected_gate')
            annotation = (
                'positive' if tracklet_id == human_tracklet_id else 'negative')
            if annotation == 'positive':
                positive_pair_id = row['pair_id']
            samples.append(row)
            annotations.append({
                'pair_id': row['pair_id'], 'annotation': annotation})
            decisions.append({
                'source': name,
                'visual_candidate_id': candidate_id,
                'visual_stamp_ns': int(row['visual_stamp_ns']),
                'manual_human_box_xyxy': manual_box,
                'lidar_tracklet_id': tracklet_id,
                'annotation': annotation,
                'projected_box_xyxy': projection['projected_box'],
                'hard_gates_passed': hard_gates_passed,
                'size_ratio_gate_passed': bool(size_term['gate_passed']),
                'total_score': row['total_score'],
                'review_basis': (
                    'human-visible image ROI and source-time LiDAR projection'),
            })
        if positive_pair_id is None:
            raise ValueError('selection did not create a positive pair')
    return samples, annotations, decisions


def write_jsonl(path, rows):
    Path(path).write_text(''.join(
        json.dumps(row, sort_keys=True, allow_nan=False) + '\n'
        for row in rows), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(
        description='Build the reviewed Stage 2C manual-ROI pilot dataset.')
    parser.add_argument('--primary-debug', required=True, type=Path)
    parser.add_argument('--primary-manifest', required=True, type=Path)
    parser.add_argument('--q50-debug', required=True, type=Path)
    parser.add_argument('--q50-manifest', required=True, type=Path)
    parser.add_argument('--output-debug', required=True, type=Path)
    parser.add_argument('--output-annotations', required=True, type=Path)
    parser.add_argument('--output-review', required=True, type=Path)
    arguments = parser.parse_args()

    primary = build_source(
        'primary', arguments.primary_debug, arguments.primary_manifest)
    q50 = build_source('q50', arguments.q50_debug, arguments.q50_manifest)
    samples = primary[0] + q50[0]
    annotations = primary[1] + q50[1]
    decisions = primary[2] + q50[2]
    stamps = [
        row['visual_stamp_ns'] for row in decisions
        if row['annotation'] == 'positive']
    if len(stamps) != len(set(stamps)):
        raise ValueError('positive manual annotations must use unique source frames')
    if len(stamps) < 20:
        raise ValueError('manual pilot requires at least 20 positive source frames')

    write_jsonl(arguments.output_debug, samples)
    write_jsonl(arguments.output_annotations, annotations)
    review = {
        'schema_version': '1.0.0',
        'review_method': 'manual_image_bbox_and_source_time_lidar_projection',
        'calibrated_size_ratio_maximum': CALIBRATED_SIZE_RATIO_MAXIMUM,
        'positive_source_frame_count': len(stamps),
        'negative_pair_count': sum(
            row['annotation'] == 'negative' for row in decisions),
        'decisions': decisions,
    }
    Path(arguments.output_review).write_text(
        json.dumps(review, indent=2, sort_keys=True, allow_nan=False) + '\n',
        encoding='utf-8')


if __name__ == '__main__':
    main()
