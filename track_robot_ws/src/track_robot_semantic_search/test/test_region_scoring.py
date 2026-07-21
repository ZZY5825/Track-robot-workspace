import numpy as np
import pytest

from track_robot_semantic_search.multiscale_windows import WindowEncoding
from track_robot_semantic_search.region_scoring import (
    ImageGeometry,
    RegionCandidate,
    score_multiscale_regions,
    score_regions,
    suppress_duplicate_regions,
)


def geometry(width=4, height=3, patch_size=16):
    return ImageGeometry(
        source_width=width * patch_size,
        source_height=height * patch_size,
        model_width=width * patch_size,
        model_height=height * patch_size,
        resized_width=width * patch_size,
        resized_height=height * patch_size,
        scale=1.0,
        padding_left=0,
        padding_top=0,
        patch_size=patch_size,
    )


def cosine_vector(score):
    return np.asarray(
        [score, np.sqrt(max(0.0, 1.0 - score ** 2))],
        dtype=np.float32)


def candidate(roi, score, peak=None):
    return RegionCandidate(
        x=roi[0], y=roi[1], width=roi[2], height=roi[3],
        token_area=1, score=score,
        peak_score=score if peak is None else peak,
        descriptor=np.asarray([1.0, 0.0], dtype=np.float32),
        descriptor_quality=1.0)


def test_cosine_score_creates_region_for_matching_tokens():
    embeddings = np.zeros((3, 4, 2), dtype=np.float32)
    embeddings[1, 1] = [1.0, 0.0]
    embeddings[1, 2] = [1.0, 0.0]
    valid = np.ones((3, 4), dtype=bool)

    regions = score_regions(
        embeddings, np.asarray([1.0, 0.0]), valid, geometry(),
        threshold=0.9, min_area=2)

    assert len(regions) == 1
    assert regions[0].roi == (16, 16, 32, 16)
    assert regions[0].token_area == 2
    assert regions[0].score == pytest.approx(1.0)
    assert regions[0].peak_score == pytest.approx(1.0)
    assert regions[0].descriptor == pytest.approx([1.0, 0.0])
    assert np.linalg.norm(regions[0].descriptor) == pytest.approx(1.0)
    assert regions[0].descriptor_quality == pytest.approx(1.0)


def test_padding_mask_excludes_high_scoring_tokens():
    embeddings = np.zeros((3, 4, 2), dtype=np.float32)
    embeddings[0, 0] = [1.0, 0.0]
    valid = np.ones((3, 4), dtype=bool)
    valid[0] = False

    regions = score_regions(
        embeddings, np.asarray([1.0, 0.0]), valid, geometry(),
        threshold=0.9)

    assert regions == []


def test_components_use_four_connectivity():
    embeddings = np.zeros((2, 2, 2), dtype=np.float32)
    embeddings[0, 0] = [1.0, 0.0]
    embeddings[1, 1] = [1.0, 0.0]

    regions = score_regions(
        embeddings, np.asarray([1.0, 0.0]),
        np.ones((2, 2), dtype=bool), geometry(2, 2), threshold=0.9)

    assert len(regions) == 2
    assert [region.roi for region in regions] == [
        (0, 0, 16, 16),
        (16, 16, 16, 16),
    ]


def test_minimum_area_and_max_regions_are_enforced():
    embeddings = np.zeros((3, 5, 2), dtype=np.float32)
    embeddings[0, 0] = [1.0, 0.0]
    embeddings[1, 1:3] = [1.0, 0.0]
    embeddings[2, 3:5] = [0.8, 0.2]

    regions = score_regions(
        embeddings, np.asarray([1.0, 0.0]),
        np.ones((3, 5), dtype=bool), geometry(5, 3),
        threshold=0.7, min_area=2, max_regions=1)

    assert len(regions) == 1
    assert regions[0].roi == (16, 16, 32, 16)


def test_quantile_threshold_uses_valid_scores_only():
    embeddings = np.asarray([[[1.0, 0.0], [0.8, 0.6], [0.0, 1.0]]])
    valid = np.asarray([[True, True, False]])

    regions = score_regions(
        embeddings, np.asarray([1.0, 0.0]), valid, geometry(3, 1),
        threshold_mode='quantile', quantile=0.75)

    assert len(regions) == 1
    assert regions[0].roi == (0, 0, 16, 16)


def test_padded_geometry_maps_component_back_to_source():
    embeddings = np.zeros((4, 4, 2), dtype=np.float32)
    embeddings[1:3, 1:3] = [1.0, 0.0]
    valid = np.zeros((4, 4), dtype=bool)
    valid[1:3] = True
    transform = ImageGeometry(
        source_width=128,
        source_height=64,
        model_width=64,
        model_height=64,
        resized_width=64,
        resized_height=32,
        scale=0.5,
        padding_left=0,
        padding_top=16,
        patch_size=16,
    )

    regions = score_regions(
        embeddings, np.asarray([1.0, 0.0]), valid, transform,
        threshold=0.9)

    assert regions[0].roi == (32, 0, 64, 64)


@pytest.mark.parametrize('bad_part', ['image', 'text'])
def test_nonfinite_embeddings_are_rejected(bad_part):
    embeddings = np.zeros((1, 1, 2), dtype=np.float32)
    text = np.asarray([1.0, 0.0], dtype=np.float32)
    if bad_part == 'image':
        embeddings[0, 0, 0] = np.nan
    else:
        text[0] = np.inf

    with pytest.raises(ValueError, match='finite'):
        score_regions(
            embeddings, text, np.ones((1, 1), dtype=bool), geometry(1, 1))


def test_shape_mismatch_is_rejected():
    with pytest.raises(ValueError, match='same embedding dimension'):
        score_regions(
            np.zeros((1, 1, 2)), np.zeros(3),
            np.ones((1, 1), dtype=bool), geometry(1, 1))


def test_deterministic_ties_use_image_position():
    embeddings = np.zeros((1, 3, 2), dtype=np.float32)
    embeddings[0, 0] = [1.0, 0.0]
    embeddings[0, 2] = [1.0, 0.0]

    regions = score_regions(
        embeddings, np.asarray([1.0, 0.0]),
        np.ones((1, 3), dtype=bool), geometry(3, 1), threshold=0.9)

    assert [region.roi[0] for region in regions] == [0, 32]


def test_empty_valid_mask_returns_no_regions():
    regions = score_regions(
        np.zeros((1, 1, 2)), np.asarray([1.0, 0.0]),
        np.zeros((1, 1), dtype=bool), geometry(1, 1))

    assert regions == []


def test_multiscale_uses_whole_image_only_as_fallback():
    embeddings = np.tile([0.0, 1.0], (2, 2, 1)).astype(np.float32)
    extras = (
        WindowEncoding('global', (0, 0, 100, 100), [1.0, 0.0]),
        WindowEncoding('center', (20, 20, 60, 60), [0.0, 1.0]),
    )

    regions = score_multiscale_regions(
        embeddings, [1.0, 0.0], np.ones((2, 2), dtype=bool),
        geometry(2, 2), extras, threshold=0.5)

    assert [region.roi for region in regions] == [(0, 0, 100, 100)]


def test_multiscale_local_region_hides_passing_global_window():
    embeddings = np.tile([0.0, 1.0], (2, 2, 1)).astype(np.float32)
    embeddings[0, 0] = [1.0, 0.0]
    extras = (
        WindowEncoding('global', (0, 0, 32, 32), [1.0, 0.0]),
    )

    regions = score_multiscale_regions(
        embeddings, [1.0, 0.0], np.ones((2, 2), dtype=bool),
        geometry(2, 2), extras, threshold=0.5)

    assert [region.roi for region in regions] == [(0, 0, 16, 16)]


def test_multiscale_center_window_is_a_local_candidate():
    embeddings = np.tile([0.0, 1.0], (2, 2, 1)).astype(np.float32)
    extras = (
        WindowEncoding('global', (0, 0, 100, 100), [0.0, 1.0]),
        WindowEncoding('center', (20, 20, 60, 60), [1.0, 0.0]),
    )

    regions = score_multiscale_regions(
        embeddings, [1.0, 0.0], np.ones((2, 2), dtype=bool),
        geometry(2, 2), extras, threshold=0.5)

    assert [region.roi for region in regions] == [(20, 20, 60, 60)]


def test_duplicate_suppression_uses_iou_and_keeps_higher_score():
    regions = [
        candidate((0, 0, 10, 10), 0.8),
        candidate((2, 0, 10, 10), 0.9),
    ]

    kept = suppress_duplicate_regions(regions, 0.50, 0.80)

    assert [region.roi for region in kept] == [(2, 0, 10, 10)]


def test_duplicate_suppression_uses_containment():
    regions = [
        candidate((0, 0, 20, 20), 0.8),
        candidate((2, 2, 10, 10), 0.9),
    ]

    kept = suppress_duplicate_regions(regions, 0.50, 0.80)

    assert [region.roi for region in kept] == [(2, 2, 10, 10)]


def test_duplicate_suppression_ties_use_image_position():
    regions = [
        candidate((10, 10, 10, 10), 0.9),
        candidate((0, 0, 20, 20), 0.9),
    ]

    kept = suppress_duplicate_regions(regions, 0.20, 0.20)

    assert [region.roi for region in kept] == [(0, 0, 20, 20)]


def test_multiscale_quantile_cutoff_uses_grid_and_extra_scores():
    embeddings = np.asarray([[
        cosine_vector(0.1), cosine_vector(0.2),
        cosine_vector(0.3), cosine_vector(0.4),
    ]])
    extras = (
        WindowEncoding('global', (0, 0, 64, 16), cosine_vector(0.8)),
        WindowEncoding('center', (16, 0, 32, 16), cosine_vector(0.9)),
    )

    regions = score_multiscale_regions(
        embeddings, [1.0, 0.0], np.ones((1, 4), dtype=bool),
        geometry(4, 1), extras,
        threshold_mode='quantile', quantile=0.5)

    assert [region.roi for region in regions] == [
        (16, 0, 32, 16), (48, 0, 16, 16)]


def test_multiscale_applies_max_regions_after_deterministic_ordering():
    embeddings = np.asarray([[
        cosine_vector(0.9), cosine_vector(0.0),
        cosine_vector(0.8), cosine_vector(0.0), cosine_vector(0.7),
    ]])

    regions = score_multiscale_regions(
        embeddings, [1.0, 0.0], np.ones((1, 5), dtype=bool),
        geometry(5, 1), (), threshold=0.5, max_regions=2)

    assert [region.roi for region in regions] == [
        (0, 0, 16, 16), (32, 0, 16, 16)]
