import ast
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch' / 'semantic_search_phase5a.launch.py'
RVIZ = PACKAGE / 'rviz' / 'semantic_search_phase5a.rviz'


def _source(path):
    assert path.is_file(), 'required Phase 5A file is missing: {}'.format(path)
    return path.read_text(encoding='utf-8')


def test_phase5a_defaults_to_passive_and_cannot_rotate():
    source = _source(LAUNCH)

    assert "'search_mode', default_value='PASSIVE_ONLY'" in source
    assert "'rotation_runtime_mode', default_value='PLANNING_ONLY'" in source
    assert "'enable_rotation_execution', default_value='false'" in source
    assert "'start_base', default_value='false'" in source
    assert "'start_imu': 'false'" in source


def test_phase5a_composes_existing_pipeline_manager_and_bounded_nav2():
    source = _source(LAUNCH)

    assert 'semantic_search_phase4a.launch.py' in source
    assert 'semantic_search_platform.launch.py' in source
    assert 'phase5a_rotation.launch.py' in source
    assert "executable='semantic_search_active_manager'" in source
    assert 'semantic_search_phase5a.yaml' in source
    assert "executable='semantic_search_live_overlay'" in source
    assert 'semantic_search_phase5a.rviz' in source
    assert ast.parse(source)


def test_phase5a_rviz_shows_search_evidence_without_manual_motion_tools():
    source = _source(RVIZ)

    for topic in (
            '/semantic_search/active_search/markers',
            '/semantic_search/phase4/markers',
            '/semantic_search/overlay_image',
            '/local_costmap/costmap',
            '/rslidar_points'):
        assert topic in source
    assert 'Fixed Frame: odom' in source
    assert 'nav2_rviz_plugins/GoalTool' not in source
    assert 'nav2_rviz_plugins/Navigation 2' not in source

