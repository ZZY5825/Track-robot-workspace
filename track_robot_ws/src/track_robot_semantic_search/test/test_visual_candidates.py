import numpy as np
import pytest

from track_robot_semantic_search.region_scoring import ImageGeometry
from track_robot_semantic_search.visual_candidates import (
    CandidateProposal,
    ProducerIdentity,
    normalize_descriptor,
    pool_roi_descriptor,
)


def geometry():
    return ImageGeometry(
        source_width=32,
        source_height=32,
        model_width=32,
        model_height=32,
        resized_width=32,
        resized_height=32,
        scale=1.0,
        padding_left=0,
        padding_top=0,
        patch_size=16,
    )


def test_normalize_descriptor_returns_unit_finite_vector():
    normalized = normalize_descriptor(np.asarray([3.0, 4.0]))

    assert normalized.dtype == np.float32
    assert np.linalg.norm(normalized) == pytest.approx(1.0)
    assert normalized.flags.writeable is False


@pytest.mark.parametrize('vector', [
    np.asarray([0.0, 0.0]),
    np.asarray([np.nan, 1.0]),
    np.asarray([[1.0]]),
])
def test_normalize_descriptor_rejects_invalid_vector(vector):
    with pytest.raises(ValueError):
        normalize_descriptor(vector)


def test_roi_pooling_uses_overlap_weights_and_valid_mask():
    embeddings = np.asarray([
        [[1.0, 0.0], [0.0, 1.0]],
        [[-1.0, 0.0], [0.0, -1.0]],
    ], dtype=np.float32)
    valid = np.asarray([[True, True], [False, True]])

    descriptor, quality = pool_roi_descriptor(
        embeddings, valid, geometry(), (0, 0, 32, 16))

    assert descriptor == pytest.approx(
        np.asarray([1.0, 1.0]) / np.sqrt(2.0))
    assert quality == pytest.approx(1.0)


def test_roi_pooling_reports_partial_coverage_and_rejects_empty_roi():
    embeddings = np.ones((2, 2, 2), dtype=np.float32)
    valid = np.asarray([[True, False], [False, False]])

    descriptor, quality = pool_roi_descriptor(
        embeddings, valid, geometry(), (0, 0, 32, 32))

    assert np.linalg.norm(descriptor) == pytest.approx(1.0)
    assert quality == pytest.approx(0.25)
    with pytest.raises(ValueError, match='positive'):
        pool_roi_descriptor(embeddings, valid, geometry(), (0, 0, 0, 1))


def test_producer_identity_is_deterministic_and_prevents_id_reuse_per_epoch():
    identity = ProducerIdentity(101)

    assert identity.epoch_id == 101
    assert identity.next_candidate_id() == 1
    assert identity.next_candidate_id() == 2
    identity.advance_epoch()
    assert identity.epoch_id == 102
    assert identity.next_candidate_id() == 1


def test_candidate_proposal_validates_roi_and_confidence():
    proposal = CandidateProposal(
        producer_epoch_id=1,
        proposal_id=2,
        roi=(1, 2, 3, 4),
        proposal_source=6,
        detector_confidence=0.8,
    )
    assert proposal.roi == (1, 2, 3, 4)

    with pytest.raises(ValueError, match='confidence'):
        CandidateProposal(1, 2, (1, 2, 3, 4), 6, 1.1)
