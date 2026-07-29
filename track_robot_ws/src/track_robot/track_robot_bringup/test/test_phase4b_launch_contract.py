import ast
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch' / 'semantic_search_phase4b.launch.py'
RVIZ = PACKAGE / 'rviz' / 'semantic_search_phase4b.rviz'


def _source(path):
    assert path.is_file(), 'required Phase 4B file is missing: {}'.format(path)
    return path.read_text(encoding='utf-8')


def test_phase4b_entrypoint_is_shadow_and_stationary_by_default():
    source = _source(LAUNCH)

    assert "default_value='SEMANTIC_SHADOW'" in source
    assert "'enable_semantic_execution', default_value='false'" in source
    assert "'start_base', default_value='false'" in source
    assert "'start_imu': 'false'" in source
    assert "'start_obstacle_map': 'false'" in source


def test_phase4b_composes_phase4a_nav2_platform_and_rviz():
    source = _source(LAUNCH)

    assert 'semantic_search_phase4a.launch.py' in source
    assert 'semantic_search_platform.launch.py' in source
    assert 'phase4b_navigation.launch.py' in source
    assert 'semantic_search_phase4b.rviz' in source
    assert "'runtime_mode': LaunchConfiguration('runtime_mode')" in source
    assert "'enable_semantic_execution':" in source
    assert "LaunchConfiguration('enable_semantic_execution')" in source
    assert ast.parse(source)


def test_phase4b_rviz_exposes_semantic_and_nav2_evidence():
    source = _source(RVIZ)

    for topic in (
            '/semantic_search/phase4/markers',
            '/semantic_search/phase4/planned_path',
            '/semantic_navigation/shadow_path',
            '/global_costmap/costmap',
            '/local_costmap/costmap',
            '/plan',
            '/rslidar_points'):
        assert topic in source
    assert 'Fixed Frame: odom' in source
    assert 'nav2_rviz_plugins/GoalTool' in source
