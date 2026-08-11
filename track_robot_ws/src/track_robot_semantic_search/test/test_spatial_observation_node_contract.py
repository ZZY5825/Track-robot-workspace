from pathlib import Path


def test_enricher_matches_observation_stamp_and_uses_exact_depth_tf():
    source = (
        Path(__file__).resolve().parents[1]
        / 'track_robot_semantic_search'
        / 'spatial_observation_node.py'
    ).read_text()
    assert 'DepthFrameBuffer' in source
    assert 'nearest(source_stamp_ns' in source
    assert 'Time(nanoseconds=match.frame.stamp_ns)' in source
    assert 'Time()' not in source
    assert 'DiagnosticArray' in source


def test_enricher_diagnostics_use_only_fixed_reason_counters():
    source = (
        Path(__file__).resolve().parents[1]
        / 'track_robot_semantic_search'
        / 'spatial_observation_node.py'
    ).read_text()
    expected_keys = (
        'matched_depth',
        'no_matching_depth',
        'depth_delta_exceeded',
        'insufficient_depth_samples',
        'depth_out_of_range',
        'tf_unavailable',
        'invalid_transformed_position',
    )

    assert '_COUNTER_KEYS = (' in source
    for key in expected_keys:
        assert "'{}'".format(key) in source
    assert "self._counters[latest_reason] += 1" in source


def test_enricher_configures_bounded_depth_matching():
    config = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'semantic_search_phase4a.yaml'
    ).read_text()

    assert ('diagnostics_topic: '
            '/semantic_search/spatial_observation_diagnostics') in config
    assert 'depth_buffer_frames: 16' in config
    assert 'depth_buffer_max_age_sec: 2.0' in config
    assert 'maximum_depth_delta_sec: 0.20' in config
