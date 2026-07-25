from pathlib import Path

import numpy
import pytest

from track_robot_semantic_search.live_overlay import (
    ExactStampBuffer,
    OverlayRegion,
    render_overlay,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_render_overlay_copies_image_and_draws_all_valid_candidates():
    source = numpy.zeros((120, 160, 3), dtype=numpy.uint8)

    rendered = render_overlay(
        source,
        [
            OverlayRegion(10, 20, 30, 40, 0.8),
            OverlayRegion(80, 30, 20, 25, 0.4),
        ],
        query_id=7,
        query_version=2,
    )

    assert numpy.array_equal(source, numpy.zeros_like(source))
    assert rendered.shape == source.shape
    assert numpy.count_nonzero(rendered) > 0
    assert rendered[20, 10].tolist() == [255, 255, 0]
    assert rendered[30, 80].tolist() == [0, 191, 255]


def test_render_overlay_clips_partial_regions_and_ignores_invalid_regions():
    source = numpy.zeros((40, 60, 3), dtype=numpy.uint8)

    rendered = render_overlay(
        source,
        [
            OverlayRegion(-5, -4, 20, 15, 0.7),
            OverlayRegion(10, 10, 0, 5, 0.9),
            OverlayRegion(10, 10, 5, 5, float('nan')),
            OverlayRegion(100, 100, 5, 5, 0.6),
        ],
        query_id=1,
        query_version=1,
    )

    assert rendered[0, 0].tolist() == [255, 255, 0]
    assert numpy.count_nonzero(rendered) > 0


def test_render_overlay_marks_a_correlated_empty_result():
    source = numpy.zeros((60, 180, 3), dtype=numpy.uint8)

    rendered = render_overlay(
        source, [], query_id=11, query_version=3)

    assert numpy.count_nonzero(rendered) > 0


def test_exact_stamp_buffer_pairs_only_equal_source_stamps():
    buffer = ExactStampBuffer(capacity=2)
    image = object()
    regions = object()

    assert buffer.add_image((1, 2), image) is None
    assert buffer.add_regions((1, 3), object()) is None
    assert buffer.add_regions((1, 2), regions) == (image, regions)


def test_exact_stamp_buffer_pairs_regions_that_arrive_before_image():
    buffer = ExactStampBuffer(capacity=2)
    image = object()
    regions = object()

    assert buffer.add_regions((4, 5), regions) is None
    assert buffer.add_image((4, 5), image) == (image, regions)


def test_exact_stamp_buffer_evicts_oldest_values_and_rejects_bad_capacity():
    with pytest.raises(ValueError, match='capacity'):
        ExactStampBuffer(capacity=0)
    with pytest.raises(ValueError, match='capacity'):
        ExactStampBuffer(capacity=65)

    buffer = ExactStampBuffer(capacity=2)
    buffer.add_image((1, 0), 'old')
    buffer.add_image((2, 0), 'middle')
    buffer.add_image((3, 0), 'new')

    assert buffer.add_regions((1, 0), 'regions') is None
    assert buffer.add_regions((2, 0), 'regions') == ('middle', 'regions')


def test_ros_adapter_contract_is_bounded_exact_stamp_and_passive():
    source = (
        PACKAGE_ROOT
        / 'track_robot_semantic_search'
        / 'live_overlay.py'
    ).read_text(encoding='utf-8')
    setup = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')

    assert "'/zed/zed_node/left/image_rect_color'" in source
    assert "'/semantic_search/regions'" in source
    assert "'/semantic_search/overlay_image'" in source
    assert "'correlation_capacity', 8" in source
    assert 'ExactStampBuffer(capacity=capacity)' in source
    assert 'semantic_search_live_overlay' in setup
    for forbidden in ('cmd_vel', 'SearchMotionIntent', 'Twist'):
        assert forbidden not in source
