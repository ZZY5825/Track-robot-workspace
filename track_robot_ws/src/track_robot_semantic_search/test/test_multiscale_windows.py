import numpy as np
import pytest

from track_robot_semantic_search.multiscale_windows import (
    WindowEncoding,
    center_window_roi,
    letterbox_to_square,
    validate_window_strategy,
)


def test_center_window_is_sixty_percent_and_centered():
    assert center_window_roi(1280, 720, 0.60) == (256, 144, 768, 432)


def test_center_window_is_deterministic_for_odd_dimensions():
    x, y, width, height = center_window_roi(1279, 719, 0.60)

    assert (x, y, width, height) == (256, 144, 767, 431)
    assert x + width <= 1279
    assert y + height <= 719


@pytest.mark.parametrize('strategy', ['grid_only', 'multiscale_v1'])
def test_supported_strategy_is_returned(strategy):
    assert validate_window_strategy(strategy, 2, 0.60) == strategy


def test_multiscale_requires_two_by_two_grid():
    with pytest.raises(ValueError, match='grid_size=2'):
        validate_window_strategy('multiscale_v1', 4, 0.60)


@pytest.mark.parametrize('scale', [0.24, 1.01, float('nan')])
def test_center_window_scale_is_bounded(scale):
    with pytest.raises(ValueError, match=r'\[0.25, 1.0\]'):
        validate_window_strategy('multiscale_v1', 2, scale)


def test_letterbox_preserves_every_source_pixel():
    source = np.arange(2 * 4 * 3, dtype=np.uint8).reshape(2, 4, 3)

    output = letterbox_to_square(source)

    assert output.shape == (4, 4, 3)
    np.testing.assert_array_equal(output[1:3], source)


def test_window_encoding_rejects_unknown_kind():
    with pytest.raises(ValueError, match='kind'):
        WindowEncoding('chair', (0, 0, 1, 1), np.ones(2))


def test_window_encoding_normalizes_and_freezes_embedding():
    encoding = WindowEncoding(
        'center', (1, 2, 3, 4), np.asarray([3.0, 4.0], dtype=np.float32))

    np.testing.assert_allclose(encoding.embedding, [0.6, 0.8])
    assert not encoding.embedding.flags.writeable
