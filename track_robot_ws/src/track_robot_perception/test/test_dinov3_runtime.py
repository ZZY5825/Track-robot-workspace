import numpy as np
import pytest
import torch
from pathlib import Path

from track_robot_perception.dinov3_runtime import (
    map_model_roi_to_source,
    patch_grid,
    preprocess_bgr_aspect_preserving,
)


def test_aspect_preserving_preprocess_centres_16_by_9_image():
    image = np.zeros((720, 1280, 3), dtype=np.uint8)

    tensor, transform = preprocess_bgr_aspect_preserving(image, 512)

    assert tuple(tensor.shape) == (1, 3, 512, 512)
    assert transform.source_width == 1280
    assert transform.source_height == 720
    assert transform.resized_width == 512
    assert transform.resized_height == 288
    assert transform.padding_left == 0
    assert transform.padding_right == 0
    assert transform.padding_top == 112
    assert transform.padding_bottom == 112
    assert transform.scale == pytest.approx(0.4)


def test_aspect_preserving_preprocess_marks_only_image_patches_valid():
    image = np.zeros((720, 1280, 3), dtype=np.uint8)

    _, transform = preprocess_bgr_aspect_preserving(image, 512)

    assert transform.valid_patch_mask.shape == (32, 32)
    assert not transform.valid_patch_mask[:7].any()
    assert transform.valid_patch_mask[7:25].all()
    assert not transform.valid_patch_mask[25:].any()
    assert int(transform.valid_patch_mask.sum()) == 18 * 32


def test_square_image_has_no_padding():
    image = np.zeros((320, 320, 3), dtype=np.uint8)

    _, transform = preprocess_bgr_aspect_preserving(image, 512)

    assert transform.resized_width == 512
    assert transform.resized_height == 512
    assert transform.padding_left == 0
    assert transform.padding_top == 0
    assert transform.valid_patch_mask.all()


def test_preprocess_rejects_invalid_images_and_input_size():
    with pytest.raises(ValueError, match='H,W,3'):
        preprocess_bgr_aspect_preserving(np.zeros((20, 20)), 512)
    with pytest.raises(ValueError, match='positive multiple'):
        preprocess_bgr_aspect_preserving(
            np.zeros((20, 20, 3), dtype=np.uint8), 510)


def test_model_roi_maps_back_to_source_and_clips_padding():
    image = np.zeros((720, 1280, 3), dtype=np.uint8)
    _, transform = preprocess_bgr_aspect_preserving(image, 512)

    roi = map_model_roi_to_source(transform, 64, 80, 256, 208)

    assert roi == (160, 0, 640, 440)


def test_model_roi_entirely_in_padding_is_empty():
    image = np.zeros((720, 1280, 3), dtype=np.uint8)
    _, transform = preprocess_bgr_aspect_preserving(image, 512)

    assert map_model_roi_to_source(transform, 0, 0, 64, 64) == (0, 0, 0, 0)


def test_patch_grid_supports_explicit_rectangular_shape():
    tokens = torch.arange(1 * 18 * 32 * 4).reshape(1, 18 * 32, 4)

    grid = patch_grid(tokens, 18, 32)

    assert grid.shape == (18, 32, 4)
    assert grid[17, 31, 3] == pytest.approx(float(tokens[0, -1, -1]))


def test_patch_grid_rejects_wrong_token_count():
    tokens = torch.zeros((1, 10, 4))

    with pytest.raises(ValueError, match='Expected 12 patch tokens'):
        patch_grid(tokens, 3, 4)


@pytest.mark.parametrize('relative_path', [
    'track_robot_perception/zed_dinov3_feature_node.py',
    'scripts/test_dinov3_on_image.py',
])
def test_phase1_callers_use_aspect_preserving_preprocess(relative_path):
    package_root = Path(__file__).resolve().parents[1]
    source = (package_root / relative_path).read_text(encoding='utf-8')

    assert 'preprocess_bgr_aspect_preserving' in source
    assert 'preprocess_bgr(image' not in source


def test_package_exposes_pytest_extra_for_colcon_discovery():
    package_root = Path(__file__).resolve().parents[1]
    setup_source = (package_root / 'setup.py').read_text(encoding='utf-8')

    assert "extras_require={'test': ['pytest']}" in setup_source
    assert 'tests_require=' not in setup_source
