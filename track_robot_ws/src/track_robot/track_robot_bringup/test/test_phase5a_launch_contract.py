import ast
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch' / 'semantic_search_phase5a.launch.py'
RVIZ = PACKAGE / 'rviz' / 'semantic_search_phase5a.rviz'
SEARCH_CONFIG = (
    PACKAGE.parents[1]
    / 'track_robot_semantic_search'
    / 'config'
    / 'semantic_search_phase5a.yaml'
)


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


def test_phase5a_composes_pipeline_manager_and_one_full_handoff_nav2():
    source = _source(LAUNCH)

    assert 'semantic_search_phase4a.launch.py' in source
    assert 'semantic_search_platform.launch.py' in source
    assert 'phase4b_navigation.launch.py' in source
    assert 'phase5a_rotation.launch.py' not in source
    assert source.count("executable='search_motion_adapter'") == 1
    assert "name='search_motion_adapter'" in source
    assert 'active_search_motion.yaml' in source
    assert "'enable_semantic_execution':" in source
    assert "LaunchConfiguration('enable_rotation_execution')" in source
    assert "executable='semantic_search_active_manager'" in source
    assert 'semantic_search_phase5a.yaml' in source
    assert "executable='semantic_search_live_overlay'" in source
    assert 'semantic_search_phase5a.rviz' in source
    assert "'base_frame': 'robot_bottom'" in source
    assert ast.parse(source)


def test_phase5a_runtime_mode_gates_have_one_launch_owner_on_foxy():
    tree = ast.parse(_source(LAUNCH))
    manager_node = None
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(
                isinstance(target, ast.Name) and target.id == 'manager'
                for target in node.targets):
            continue
        if isinstance(node.value, ast.Call):
            manager_node = node.value
            break

    assert manager_node is not None
    parameters = next(
        keyword.value for keyword in manager_node.keywords
        if keyword.arg == 'parameters')
    assert isinstance(parameters, ast.List)
    assert len(parameters.elts) == 2
    runtime_parameters = next(
        element for element in parameters.elts
        if isinstance(element, ast.Dict))
    runtime_keys = {
        key.value for key in runtime_parameters.keys
        if isinstance(key, ast.Constant)
    }
    assert 'search_mode' in runtime_keys
    assert 'active_search_execution_enabled' in runtime_keys

    config = yaml.safe_load(_source(SEARCH_CONFIG))[
        'active_search_manager']['ros__parameters']
    assert 'search_mode' not in config
    assert 'active_search_execution_enabled' not in config


def test_phase5a_rviz_shows_search_evidence_without_manual_motion_tools():
    source = _source(RVIZ)

    for topic in (
            '/semantic_search/active_search/markers',
            '/semantic_search/phase4/markers',
            '/semantic_search/phase4/planned_path',
            '/plan',
            '/semantic_search/overlay_image',
            '/local_costmap/costmap',
            '/rslidar_points'):
        assert topic in source
    assert source.count('Class: rviz_default_plugins/Path') >= 2
    assert 'Fixed Frame: odom' in source
    assert 'nav2_rviz_plugins/GoalTool' not in source
    assert 'nav2_rviz_plugins/Navigation 2' not in source
    assert 'Class: rviz_default_plugins/RobotModel' in source
    assert 'Value: /robot_description' in source
