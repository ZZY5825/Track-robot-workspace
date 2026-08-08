from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SETUP = PACKAGE_ROOT / 'setup.py'
CAPTURE = (
    PACKAGE_ROOT / 'track_robot_semantic_search' / 'confidence_capture.py')


def test_setup_exposes_capture_and_offline_benchmark_commands():
    source = SETUP.read_text(encoding='utf-8')

    assert 'semantic_search_confidence_capture = ' in source
    assert 'confidence_capture:main' in source
    assert 'semantic_search_confidence_benchmark = ' in source
    assert 'confidence_benchmark_cli:main' in source


def test_capture_is_subscriber_only_and_uses_synchronized_zed_topics():
    source = CAPTURE.read_text(encoding='utf-8')

    assert 'ApproximateTimeSynchronizer' in source
    assert '/zed/zed_node/left/image_rect_color' in source
    assert '/zed/zed_node/depth/depth_registered' in source
    for forbidden in (
            'create_publisher', '/cmd_vel', 'NavigateToPose',
            '/semantic_search/query'):
        assert forbidden not in source
