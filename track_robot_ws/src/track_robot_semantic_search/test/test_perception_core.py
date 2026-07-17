import numpy as np
import pytest

from track_robot_semantic_search.perception_core import (
    PassivePerceptionCore,
    SourceTimestampScheduler,
)
from track_robot_semantic_search.model_adapters import ImageGridEncoding
from track_robot_semantic_search.region_scoring import ImageGeometry
from track_robot_semantic_search.visual_candidates import CandidateProposal


class FakeAlignedEncoder:
    encoder_id = 'aligned:fake'
    checkpoint_id = 'sha256:fake'

    def __init__(self):
        self.text_calls = []
        self.image_calls = 0
        self.bad_dimension = False

    def encode_text(self, text):
        self.text_calls.append(text)
        return np.asarray([1.0, 0.0], dtype=np.float32)

    def encode_image_grid(self, image_bgr):
        self.image_calls += 1
        dimension = 3 if self.bad_dimension else 2
        embeddings = np.zeros((2, 2, dimension), dtype=np.float32)
        embeddings[0, 0, 0] = 1.0
        return ImageGridEncoding(
            embeddings=embeddings,
            valid_patch_mask=np.ones((2, 2), dtype=bool),
            geometry=ImageGeometry(
                source_width=image_bgr.shape[1],
                source_height=image_bgr.shape[0],
                model_width=32,
                model_height=32,
                resized_width=32,
                resized_height=32,
                scale=32.0 / image_bgr.shape[1],
                padding_left=0,
                padding_top=0,
                patch_size=16),
            inference_ms=12.5)


def test_core_reuses_cached_text_and_increments_observation_id():
    encoder = FakeAlignedEncoder()
    core = PassivePerceptionCore(encoder, threshold=0.9)
    image = np.zeros((32, 32, 3), dtype=np.uint8)

    first = core.process(image, 'fallen   branch', 5, 7, 100)
    second = core.process(image, 'fallen branch', 5, 7, 200)

    assert encoder.text_calls == ['fallen branch']
    assert encoder.image_calls == 2
    assert first.observation_id == 1
    assert second.observation_id == 2
    assert len(first.regions) == 1
    assert first.query_id == 5
    assert first.query_version == 7
    assert first.image_encoder_id == 'aligned:fake'
    assert first.text_encoder_id == 'aligned:fake'
    assert first.checkpoint_id == 'sha256:fake'
    assert first.inference_ms == pytest.approx(12.5)
    assert len(first.visual_candidates) == 1
    assert first.visual_candidates[0].visual_candidate_id == 1
    assert first.visual_candidates[0].proposal_source == 2
    assert np.linalg.norm(first.visual_candidates[0].descriptor) == (
        pytest.approx(1.0))
    assert np.linalg.norm(first.task_descriptor) == pytest.approx(1.0)


def test_core_reencodes_when_query_version_changes():
    encoder = FakeAlignedEncoder()
    core = PassivePerceptionCore(encoder, threshold=0.9)
    image = np.zeros((32, 32, 3), dtype=np.uint8)

    core.process(image, 'branch', 1, 1, 100)
    core.process(image, 'branch', 1, 2, 200)

    assert encoder.text_calls == ['branch', 'branch']


def test_core_returns_empty_candidates_without_fabricating_full_frame():
    encoder = FakeAlignedEncoder()
    core = PassivePerceptionCore(encoder, threshold=1.1)

    result = core.process(
        np.zeros((32, 32, 3), dtype=np.uint8), 'branch', 1, 1, 100)

    assert result.regions == []
    assert result.visual_candidates == []


def test_core_processes_external_proposal_without_active_query():
    encoder = FakeAlignedEncoder()
    core = PassivePerceptionCore(encoder)
    proposal = CandidateProposal(
        producer_epoch_id=9,
        proposal_id=10,
        roi=(0, 0, 16, 16),
        proposal_source=6,
        detector_confidence=0.75,
    )

    result = core.process_frame(
        np.zeros((32, 32, 3), dtype=np.uint8),
        stamp_ns=100,
        proposals=[proposal],
    )

    assert result.query_id == 0
    assert result.regions == []
    assert result.task_descriptor.size == 0
    assert len(result.visual_candidates) == 1
    candidate = result.visual_candidates[0]
    assert candidate.upstream_proposal_id == 10
    assert candidate.detector_confidence == pytest.approx(0.75)
    assert candidate.visual_candidate_id == 1
    assert np.linalg.norm(candidate.descriptor) == pytest.approx(1.0)


def test_core_resets_candidate_ids_only_at_a_new_producer_epoch():
    core = PassivePerceptionCore(FakeAlignedEncoder())
    proposal = CandidateProposal(10, 1, (0, 0, 16, 16), 6, 0.9)
    image = np.zeros((32, 32, 3), dtype=np.uint8)

    first = core.process_frame(image, stamp_ns=1, proposals=[proposal])
    second = core.process_frame(image, stamp_ns=2, proposals=[proposal])
    core.start_producer_epoch()
    after_epoch_change = core.process_frame(
        image, stamp_ns=3, proposals=[proposal])

    assert first.visual_candidates[0].visual_candidate_id == 1
    assert second.visual_candidates[0].visual_candidate_id == 2
    assert after_epoch_change.visual_candidates[0].visual_candidate_id == 1


def test_core_rejects_image_text_dimension_mismatch():
    encoder = FakeAlignedEncoder()
    encoder.bad_dimension = True
    core = PassivePerceptionCore(encoder)

    with pytest.raises(ValueError, match='same embedding dimension'):
        core.process(
            np.zeros((32, 32, 3), dtype=np.uint8), 'branch', 1, 1, 100)


def test_core_rejects_invalid_image_and_identifiers():
    core = PassivePerceptionCore(FakeAlignedEncoder())
    with pytest.raises(ValueError, match='H,W,3'):
        core.process(np.zeros((2, 2)), 'branch', 1, 1, 100)
    with pytest.raises(ValueError, match='query_id'):
        core.process(np.zeros((32, 32, 3)), 'branch', -1, 1, 100)


def test_source_timestamp_scheduler_targets_five_hz():
    scheduler = SourceTimestampScheduler(5.0)

    assert scheduler.should_process(1_000_000_000)
    assert not scheduler.should_process(1_100_000_000)
    assert scheduler.should_process(1_200_000_000)


def test_source_timestamp_rollback_resets_scheduler():
    scheduler = SourceTimestampScheduler(5.0)
    assert scheduler.should_process(2_000_000_000)

    assert not scheduler.should_process(1_000_000_000)
    assert scheduler.rollback_count == 1
    assert scheduler.should_process(1_200_000_000)


@pytest.mark.parametrize('rate', [0.0, -1.0, float('nan')])
def test_scheduler_rejects_invalid_rate(rate):
    with pytest.raises(ValueError, match='target_rate_hz'):
        SourceTimestampScheduler(rate)
