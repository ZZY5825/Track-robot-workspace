import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest


SCHEMA_PATH = (
    Path(__file__).resolve().parents[1] /
    'schemas' / 'grounding_dataset.schema.json')


def _sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def split_cases():
    return [
        {
            'case_id': 'session-a-frame-0001-query-1',
            'split': 'train',
            'session_id': 'session-a',
            'physical_object_id': 'blue-container-01',
            'target_present': True,
            'ground_truth_boxes_xywh': [[1.0, 2.0, 3.0, 4.0]],
        },
        {
            'case_id': 'session-b-frame-0001-query-1',
            'split': 'validation',
            'session_id': 'session-b',
            'physical_object_id': '',
            'target_present': False,
            'ground_truth_boxes_xywh': [],
        },
        {
            'case_id': 'session-c-frame-0001-query-1',
            'split': 'test',
            'session_id': 'session-c',
            'physical_object_id': 'blue-container-03',
            'target_present': True,
            'ground_truth_boxes_xywh': [[0.0, 0.0, 8.0, 8.0]],
        },
    ]


def write_dataset_fixture(tmp_path, cases, mutation=None):
    images_dir = tmp_path / 'images'
    images_dir.mkdir()
    payload_cases = []
    for index, base_case in enumerate(cases):
        image_path = images_dir / ('case-{}.png'.format(index))
        assert cv2.imwrite(
            str(image_path), np.full((8, 8, 3), index, dtype=np.uint8))
        case = {
            'case_id': base_case['case_id'],
            'split': base_case['split'],
            'image_relative_path': 'images/{}'.format(image_path.name),
            'image_sha256': _sha256(image_path),
            'image_width': 8,
            'image_height': 8,
            'session_id': base_case['session_id'],
            'physical_object_id': base_case['physical_object_id'],
            'query_text': 'a tall blue cylindrical container',
            'target_present': base_case['target_present'],
            'ground_truth_boxes_xywh': base_case['ground_truth_boxes_xywh'],
            'scenario_tags': ['cluttered', 'distance_1_to_2m'],
            'label_review_status': 'human_verified',
        }
        payload_cases.append(case)

    if mutation == 'positive_without_box':
        payload_cases[0]['ground_truth_boxes_xywh'] = []
    elif mutation == 'negative_with_box':
        payload_cases[1]['ground_truth_boxes_xywh'] = [[1.0, 2.0, 3.0, 4.0]]
    elif mutation == 'box_outside_image':
        payload_cases[0]['ground_truth_boxes_xywh'] = [[6.0, 2.0, 3.0, 4.0]]
    elif mutation == 'wrong_digest':
        payload_cases[0]['image_sha256'] = '0' * 64
    elif mutation == 'duplicate_case_id':
        payload_cases[1]['case_id'] = payload_cases[0]['case_id']
    elif mutation == 'unsafe_relative_path':
        payload_cases[0]['image_relative_path'] = '../case-0.png'
    elif mutation is not None:
        raise ValueError('unknown mutation')

    document_path = tmp_path / 'grounding_dataset.json'
    document_path.write_text(json.dumps({
        'schema_version': '1.0.0',
        'dataset_id': 'grounding-r0',
        'cases': payload_cases,
    }), encoding='utf-8')
    return document_path


def write_leaking_fixture(tmp_path, leakage):
    cases = split_cases()
    if leakage == 'image_sha256':
        document_path = write_dataset_fixture(tmp_path, cases)
        payload = json.loads(document_path.read_text(encoding='utf-8'))
        payload['cases'][1]['image_sha256'] = payload['cases'][0]['image_sha256']
        second_image = tmp_path / payload['cases'][1]['image_relative_path']
        first_image = tmp_path / payload['cases'][0]['image_relative_path']
        second_image.write_bytes(first_image.read_bytes())
        document_path.write_text(json.dumps(payload), encoding='utf-8')
        return document_path
    target_index = 2 if leakage == 'physical_object_id' else 1
    cases[target_index][leakage] = cases[0][leakage]
    return write_dataset_fixture(tmp_path, cases)


def test_schema_has_versioned_closed_case_contract():
    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
    assert schema['properties']['schema_version']['const'] == '1.0.0'
    assert schema['additionalProperties'] is False
    assert set(schema['properties']['cases']['items']['required']) == {
        'case_id', 'split', 'image_relative_path', 'image_sha256',
        'image_width', 'image_height', 'session_id', 'physical_object_id',
        'query_text', 'target_present', 'ground_truth_boxes_xywh',
        'scenario_tags', 'label_review_status',
    }


def test_loads_verified_positive_and_negative_cases(tmp_path):
    from track_robot_semantic_search.grounding_dataset import (
        GroundingBox,
        load_grounding_dataset,
    )

    dataset = load_grounding_dataset(
        write_dataset_fixture(tmp_path, split_cases()))

    assert dataset.dataset_id == 'grounding-r0'
    assert [case.split for case in dataset.cases] == [
        'train', 'validation', 'test']
    assert dataset.cases[0].boxes[0] == GroundingBox(1.0, 2.0, 3.0, 4.0)
    assert dataset.cases[1].physical_object_id == ''
    assert dataset.cases[1].boxes == ()


@pytest.mark.parametrize('review_status', [
    'human_authored',
    'human_verified',
])
def test_preserves_label_review_status(tmp_path, review_status):
    from track_robot_semantic_search.grounding_dataset import (
        load_grounding_dataset,
    )

    document_path = write_dataset_fixture(tmp_path, split_cases())
    payload = json.loads(document_path.read_text(encoding='utf-8'))
    payload['cases'][0]['label_review_status'] = review_status
    document_path.write_text(json.dumps(payload), encoding='utf-8')

    dataset = load_grounding_dataset(document_path)

    assert dataset.cases[0].label_review_status == review_status


def test_rejects_present_target_without_physical_object_id(tmp_path):
    from track_robot_semantic_search.grounding_dataset import (
        load_grounding_dataset,
    )

    document_path = write_dataset_fixture(tmp_path, split_cases())
    payload = json.loads(document_path.read_text(encoding='utf-8'))
    payload['cases'][0]['physical_object_id'] = ''
    document_path.write_text(json.dumps(payload), encoding='utf-8')

    with pytest.raises(ValueError, match='physical_object_id'):
        load_grounding_dataset(document_path)


def test_rejects_absent_target_with_physical_object_id(tmp_path):
    from track_robot_semantic_search.grounding_dataset import (
        load_grounding_dataset,
    )

    document_path = write_dataset_fixture(tmp_path, split_cases())
    payload = json.loads(document_path.read_text(encoding='utf-8'))
    payload['cases'][1]['physical_object_id'] = 'unexpected-object'
    document_path.write_text(json.dumps(payload), encoding='utf-8')

    with pytest.raises(ValueError, match='physical_object_id'):
        load_grounding_dataset(document_path)


@pytest.mark.parametrize('field,value', [
    ('split', []),
    ('split', {}),
    ('label_review_status', []),
    ('label_review_status', {}),
])
def test_rejects_non_string_enum_fields_as_value_error(
        tmp_path, field, value):
    from track_robot_semantic_search.grounding_dataset import (
        load_grounding_dataset,
    )

    document_path = write_dataset_fixture(tmp_path, split_cases())
    payload = json.loads(document_path.read_text(encoding='utf-8'))
    payload['cases'][0][field] = value
    document_path.write_text(json.dumps(payload), encoding='utf-8')

    with pytest.raises(ValueError, match=field):
        load_grounding_dataset(document_path)


@pytest.mark.parametrize('mutation,reason', [
    ('positive_without_box', 'target_present'),
    ('negative_with_box', 'target_present'),
    ('box_outside_image', 'image bounds'),
    ('wrong_digest', 'sha256'),
    ('duplicate_case_id', 'case_id'),
    ('unsafe_relative_path', 'relative path'),
])
def test_rejects_invalid_dataset_case(tmp_path, mutation, reason):
    from track_robot_semantic_search.grounding_dataset import (
        load_grounding_dataset,
    )

    with pytest.raises(ValueError, match=reason):
        load_grounding_dataset(
            write_dataset_fixture(tmp_path, split_cases(), mutation))


@pytest.mark.parametrize('leakage', [
    'session_id', 'physical_object_id', 'image_sha256'])
def test_rejects_train_validation_test_leakage(tmp_path, leakage):
    from track_robot_semantic_search.grounding_dataset import (
        load_grounding_dataset,
    )

    with pytest.raises(ValueError, match='split leakage'):
        load_grounding_dataset(write_leaking_fixture(tmp_path, leakage))
