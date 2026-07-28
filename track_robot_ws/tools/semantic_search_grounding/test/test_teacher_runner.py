import json
from pathlib import Path

import pytest

from track_robot_semantic_search.grounding_dataset import (
    GroundingCase,
    GroundingDataset,
)
from track_robot_semantic_search.grounding_query import GroundingQuery


def _case(case_id, normalized_query):
    return GroundingCase(
        case_id=case_id,
        split='validation',
        image_path=Path('/unused/{}.png'.format(case_id)),
        image_sha256='a' * 64,
        session_id='session-{}'.format(case_id),
        physical_object_id='object-{}'.format(case_id),
        query=GroundingQuery(
            raw_text=normalized_query.title(),
            normalized_text=normalized_query,
        ),
        target_present=True,
        boxes=(),
        scenario_tags=(),
        label_review_status='human_verified',
    )


def _dataset():
    return GroundingDataset(
        dataset_id='r0b-corpus',
        cases=(
            _case('case-b', 'a blue bag'),
            _case('case-a', 'a green cup'),
        ),
    )


def _identity():
    from tools.semantic_search_grounding.contracts import TeacherIdentity

    return TeacherIdentity(
        candidate_id='grounding-dino-tiny-hf-b005-t005',
        implementation='huggingface_transformers_grounding_dino',
        code_revision='runner-v1;model-revision=abc123',
        checkpoint_id='model.safetensors',
        checkpoint_sha256='b' * 64,
        licence='Apache-2.0',
        platform={
            'role': 'desktop_teacher',
            'hardware': 'NVIDIA RTX 4090',
            'os': 'Ubuntu 22.04',
            'python': '3.11.9',
            'pytorch': '2.4.1',
            'device': 'cuda:0',
        },
        input_size=(1333, 800),
    )


class FakeBackend:
    def __init__(self, detections=None):
        self.calls = []
        self.synchronizations = 0
        self.detections = detections or []

    def synchronize(self):
        self.synchronizations += 1

    def predict(self, image_path, normalized_query):
        self.calls.append((image_path, normalized_query))
        return tuple(self.detections)

    def incremental_cuda_reserved_mib(self):
        return 321.5


def _clock(values):
    iterator = iter(values)
    return lambda: next(iterator)


def test_processes_every_case_and_times_complete_path():
    from tools.semantic_search_grounding.teacher_runner import (
        build_prediction_document,
    )

    backend = FakeBackend()
    document = build_prediction_document(
        _dataset(),
        backend,
        _identity(),
        licence_approved=True,
        clock_ns=_clock([0, 10_000_000, 20_000_000, 35_000_000]),
    )

    assert [value['case_id'] for value in document['predictions']] == [
        'case-b', 'case-a']
    assert [value['complete_path_ms']
            for value in document['predictions']] == [10.0, 15.0]
    assert backend.calls == [
        (Path('/unused/case-b.png'), 'a blue bag'),
        (Path('/unused/case-a.png'), 'a green cup'),
    ]
    assert backend.synchronizations == 4


def test_sorts_detections_by_score_geometry_and_label():
    from tools.semantic_search_grounding.contracts import TeacherDetection
    from tools.semantic_search_grounding.teacher_runner import (
        build_prediction_document,
    )

    backend = FakeBackend([
        TeacherDetection(10, 5, 20, 15, 0.8, 'z-label'),
        TeacherDetection(1, 2, 8, 9, 0.9, 'first'),
        TeacherDetection(10, 5, 20, 15, 0.8, 'a-label'),
    ])
    document = build_prediction_document(
        GroundingDataset('one-case', (_dataset().cases[0],)),
        backend,
        _identity(),
        licence_approved=False,
        clock_ns=_clock([0, 1_000_000]),
    )

    assert document['predictions'][0]['detections'] == [
        {'box_xywh': [1.0, 2.0, 7.0, 7.0],
         'score': 0.9, 'label': 'first'},
        {'box_xywh': [10.0, 5.0, 10.0, 10.0],
         'score': 0.8, 'label': 'a-label'},
        {'box_xywh': [10.0, 5.0, 10.0, 10.0],
         'score': 0.8, 'label': 'z-label'},
    ]


def test_truncates_to_r0a_detection_limit():
    from tools.semantic_search_grounding.contracts import TeacherDetection
    from tools.semantic_search_grounding.teacher_runner import (
        build_prediction_document,
    )

    backend = FakeBackend([
        TeacherDetection(index, 0, index + 1, 1, index / 300.0, 'object')
        for index in range(300)
    ])
    document = build_prediction_document(
        GroundingDataset('one-case', (_dataset().cases[0],)),
        backend,
        _identity(),
        licence_approved=False,
        clock_ns=_clock([0, 1_000_000]),
    )

    detections = document['predictions'][0]['detections']
    assert len(detections) == 256
    assert detections[0]['score'] == pytest.approx(299 / 300.0)


def test_emits_r0a_loadable_desktop_teacher_artifact(tmp_path):
    from tools.semantic_search_grounding.teacher_runner import (
        build_prediction_document,
    )
    from track_robot_semantic_search.grounding_predictions import (
        load_grounding_predictions,
    )

    document = build_prediction_document(
        _dataset(),
        FakeBackend(),
        _identity(),
        licence_approved=True,
        clock_ns=_clock([0, 1, 2, 3]),
    )
    path = tmp_path / 'predictions.json'
    path.write_text(json.dumps(document), encoding='utf-8')
    parsed = load_grounding_predictions(path)

    assert parsed.dataset_id == 'r0b-corpus'
    assert parsed.release_evidence == {
        'runtime_available': True,
        'platform_compatible': False,
        'licence_approved': True,
    }
    assert parsed.incremental_cuda_reserved_mib == 321.5
    assert set(parsed.predictions) == {'case-a', 'case-b'}


def test_emits_platform_compatible_orin_candidate_evidence():
    from tools.semantic_search_grounding.teacher_runner import (
        build_prediction_document,
    )

    document = build_prediction_document(
        GroundingDataset('one-case', (_dataset().cases[0],)),
        FakeBackend(),
        _identity(),
        licence_approved=False,
        platform_compatible=True,
        clock_ns=_clock([0, 1]),
    )

    assert document['release_evidence'] == {
        'runtime_available': True,
        'platform_compatible': True,
        'licence_approved': False,
    }


def test_rejects_non_boolean_platform_compatibility():
    from tools.semantic_search_grounding.teacher_runner import (
        build_prediction_document,
    )

    with pytest.raises(ValueError, match='platform_compatible'):
        build_prediction_document(
            GroundingDataset('one-case', (_dataset().cases[0],)),
            FakeBackend(),
            _identity(),
            licence_approved=False,
            platform_compatible=1,
            clock_ns=_clock([0, 1]),
        )


@pytest.mark.parametrize('detection', [
    (0, 0, 1, 1, float('nan'), 'object'),
    (0, 0, float('inf'), 1, 0.5, 'object'),
    (0, 0, 0, 1, 0.5, 'object'),
    (-1, 0, 1, 1, 0.5, 'object'),
    (0, 0, 1, 1, 1.1, 'object'),
    (0, 0, 1, 1, 0.5, ''),
])
def test_rejects_malformed_backend_detections(detection):
    from tools.semantic_search_grounding.contracts import TeacherDetection
    from tools.semantic_search_grounding.teacher_runner import (
        build_prediction_document,
    )

    backend = FakeBackend([TeacherDetection(*detection)])
    with pytest.raises(ValueError, match='teacher detection'):
        build_prediction_document(
            GroundingDataset('one-case', (_dataset().cases[0],)),
            backend,
            _identity(),
            licence_approved=False,
            clock_ns=_clock([0, 1]),
        )
