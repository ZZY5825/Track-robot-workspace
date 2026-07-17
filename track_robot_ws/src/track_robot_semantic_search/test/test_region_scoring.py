import numpy as np
import pytest

from track_robot_semantic_search.region_scoring import (
    ImageGeometry,
    score_regions,
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
