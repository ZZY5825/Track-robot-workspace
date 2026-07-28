import json
from pathlib import Path

import pytest


SCHEMA_PATH = (
    Path(__file__).resolve().parents[1] /
    'schemas' / 'grounding_predictions.schema.json')


def prediction_document():
    return {
        'schema_version': '1.0.0',
        'dataset_id': 'grounding-r0',
        'candidate_id': 'yolo_world_s_1280',
        'model_identity': {
            'implementation': 'yolo-world',
            'code_revision': 'full-revision-string',
            'checkpoint_id': 'yolo_world_v2_s.pth',
            'checkpoint_sha256': 'a' * 64,
            'licence': 'GPL-3.0',
        },
        'platform': {
            'role': 'jetson_candidate',
            'hardware': 'Jetson AGX Orin',
            'os': 'Ubuntu 20.04',
            'python': '3.8.10',
            'pytorch': '1.13.0',
            'device': 'cuda',
        },
        'input_size': [1280, 1280],
        'incremental_cuda_reserved_mib': 1024.0,
        'release_evidence': {
            'runtime_available': True,
            'platform_compatible': True,
            'licence_approved': True,
        },
        'predictions': [{
            'case_id': 'test-1',
            'complete_path_ms': 80.0,
            'detections': [{
                'box_xywh': [600.0, 320.0, 30.0, 110.0],
                'score': 0.91,
                'label': 'a tall blue cylindrical container',
            }],
        }],
    }


def write_prediction_document(tmp_path, mutation=None):
    document = prediction_document()
    if mutation == 'duplicate_case_id':
        document['predictions'].append(document['predictions'][0].copy())
    elif mutation == 'missing_case_id':
        del document['predictions'][0]['case_id']
    elif mutation == 'too_many_detections':
        document['predictions'][0]['detections'] *= 257
    elif mutation == 'nonfinite_latency':
        document['predictions'][0]['complete_path_ms'] = float('inf')
    elif mutation == 'nonfinite_score':
        document['predictions'][0]['detections'][0]['score'] = float('nan')
    elif mutation == 'score_out_of_range':
        document['predictions'][0]['detections'][0]['score'] = 1.01
    elif mutation == 'invalid_identity':
        document['model_identity']['checkpoint_sha256'] = 'not-a-checksum'
    elif mutation == 'empty_identity':
        document['model_identity']['implementation'] = ''
    elif mutation == 'absolute_checkpoint_path':
        document['model_identity']['checkpoint_id'] = '/models/yolo.pth'
    elif mutation == 'negative_memory':
        document['incremental_cuda_reserved_mib'] = -1.0
    elif mutation == 'nonfinite_memory':
        document['incremental_cuda_reserved_mib'] = float('inf')
    elif mutation == 'nonboolean_runtime_evidence':
        document['release_evidence']['runtime_available'] = 'yes'
    elif mutation == 'nonboolean_platform_evidence':
        document['release_evidence']['platform_compatible'] = 'yes'
    elif mutation == 'nonboolean_licence_evidence':
        document['release_evidence']['licence_approved'] = 'yes'
    elif mutation == 'malformed_input_size':
        document['input_size'] = [1280]
    elif mutation == 'invalid_box':
        document['predictions'][0]['detections'][0]['box_xywh'][2] = 0.0
    elif mutation == 'unknown_field':
        document['platform']['unexpected'] = 'field'
    elif mutation is not None:
        raise ValueError('unknown mutation')

    path = tmp_path / 'grounding_predictions.json'
    path.write_text(json.dumps(document), encoding='utf-8')
    return path


def test_schema_has_closed_versioned_prediction_contract():
    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))

    assert schema['properties']['schema_version']['const'] == '1.0.0'
    assert schema['additionalProperties'] is False
    assert set(schema['required']) == {
        'schema_version', 'dataset_id', 'candidate_id', 'model_identity',
        'platform', 'input_size', 'incremental_cuda_reserved_mib',
        'release_evidence', 'predictions',
    }
    assert schema['properties']['predictions']['maxItems'] == 100000
    assert (schema['properties']['predictions']['items']['properties']
            ['detections']['maxItems'] == 256)


def test_loads_valid_model_independent_predictions(tmp_path):
    from track_robot_semantic_search.grounding_predictions import (
        load_grounding_predictions,
    )

    result = load_grounding_predictions(write_prediction_document(tmp_path))

    assert result.candidate_id == 'yolo_world_s_1280'
    assert result.input_size == (1280, 1280)
    assert result.predictions['test-1'].detections[0].score == 0.91


@pytest.mark.parametrize('mutation,reason', [
    ('duplicate_case_id', 'duplicate case_id'),
    ('missing_case_id', 'missing case_id'),
    ('too_many_detections', 'detections'),
    ('nonfinite_latency', 'complete_path_ms'),
    ('nonfinite_score', 'score'),
    ('score_out_of_range', 'score'),
    ('invalid_identity', 'checkpoint_sha256'),
    ('empty_identity', 'implementation'),
    ('absolute_checkpoint_path', 'checkpoint_id'),
    ('negative_memory', 'incremental_cuda_reserved_mib'),
    ('nonfinite_memory', 'incremental_cuda_reserved_mib'),
    ('nonboolean_runtime_evidence', 'runtime_available'),
    ('nonboolean_platform_evidence', 'platform_compatible'),
    ('nonboolean_licence_evidence', 'licence_approved'),
    ('malformed_input_size', 'input_size'),
    ('invalid_box', 'box_xywh'),
    ('unknown_field', 'unknown fields'),
])
def test_rejects_invalid_prediction_document(tmp_path, mutation, reason):
    from track_robot_semantic_search.grounding_predictions import (
        load_grounding_predictions,
    )

    with pytest.raises(ValueError, match=reason):
        load_grounding_predictions(write_prediction_document(tmp_path, mutation))
