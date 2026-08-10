from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch' / 'semantic_search_phase5a.launch.py'
MANAGER = (
    PACKAGE.parents[1]
    / 'track_robot_semantic_search'
    / 'track_robot_semantic_search'
    / 'active_search_manager_node.py'
)


def test_passive_and_shadow_manager_never_publish_velocity_or_nav2_goals():
    source = MANAGER.read_text(encoding='utf-8')

    assert 'SearchMotionIntent' in source
    assert 'geometry_msgs.msg import Twist' not in source
    assert "'/cmd_vel'" not in source
    assert 'NavigateToPose' not in source
    assert 'ActionClient' not in source


def test_bringup_keeps_rotation_execution_behind_explicit_separate_gate():
    source = LAUNCH.read_text(encoding='utf-8')

    assert "'search_mode': LaunchConfiguration('search_mode')" in source
    assert "'active_search_execution_enabled':" in source
    assert "LaunchConfiguration('enable_rotation_execution')" in source
    assert "'enable_rotation_execution', default_value='false'" in source
    assert "'physical_recovery_enabled', default_value='false'" in source
    assert "'runtime_mode': LaunchConfiguration('rotation_runtime_mode')" in source
    assert "'enable_semantic_execution':" in source
    assert 'phase4b_navigation.launch.py' in source
    assert "'physical_recovery_enabled':" in source
    assert "condition=IfCondition(LaunchConfiguration(\n            'enable_rotation_execution'))" in source
