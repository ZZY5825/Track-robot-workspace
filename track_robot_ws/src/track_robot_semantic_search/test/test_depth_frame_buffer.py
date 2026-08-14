import numpy as np

from track_robot_semantic_search.depth_frame_buffer import (
    DepthFrame,
    DepthFrameBuffer,
)


def frame(stamp_ns):
    return DepthFrame(stamp_ns, 'zed_left_camera_optical_frame',
                      np.full((2, 2), float(stamp_ns)))


def test_nearest_uses_source_time_and_earlier_tie_break():
    buffer = DepthFrameBuffer(max_frames=4, max_age_ns=1_000)
    buffer.push(frame(100))
    buffer.push(frame(200))
    match = buffer.nearest(150, maximum_delta_ns=50)
    assert match.frame.stamp_ns == 100
    assert match.delta_ns == 50


def test_closest_uses_source_time_and_earlier_tie_break():
    buffer = DepthFrameBuffer(max_frames=4, max_age_ns=1_000)
    buffer.push(frame(100))
    buffer.push(frame(200))

    match = buffer.closest(150)

    assert match.frame.stamp_ns == 100
    assert match.delta_ns == 50


def test_nearest_rejects_frame_outside_maximum_delta():
    buffer = DepthFrameBuffer(max_frames=4, max_age_ns=1_000)
    buffer.push(frame(100))
    assert buffer.nearest(201, maximum_delta_ns=100) is None


def test_push_enforces_count_bound_independently_of_age_bound():
    buffer = DepthFrameBuffer(max_frames=2, max_age_ns=10_000)
    for stamp_ns in (100, 200, 300):
        buffer.push(frame(stamp_ns))
    assert buffer.size == 2
    assert buffer.nearest(100, maximum_delta_ns=1_000).frame.stamp_ns == 200


def test_push_enforces_age_bound_independently_of_count_bound():
    buffer = DepthFrameBuffer(max_frames=10, max_age_ns=150)
    for stamp_ns in (100, 200, 300):
        buffer.push(frame(stamp_ns))
    assert buffer.size == 2
    assert buffer.nearest(100, maximum_delta_ns=1_000).frame.stamp_ns == 200


def test_timestamp_rollback_clears_previous_camera_epoch():
    buffer = DepthFrameBuffer(max_frames=4, max_age_ns=1_000)
    buffer.push(frame(10_000))
    buffer.push(frame(100))
    assert buffer.size == 1
    assert buffer.nearest(100, maximum_delta_ns=0).frame.stamp_ns == 100
