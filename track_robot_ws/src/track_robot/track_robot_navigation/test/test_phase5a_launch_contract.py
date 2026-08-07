from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE_ROOT / 'launch' / 'phase5a_rotation.launch.py'


def test_phase5a_launch_is_default_off_and_requires_explicit_gate():
    source = LAUNCH.read_text()

    assert "default_value='PLANNING_ONLY'" in source
    assert "'enable_rotation_execution', default_value='false'" in source
    assert 'validate_mode_request(' in source
    assert 'enable_rotation_execution=rotation_enabled' in source


def test_rotation_runtime_has_spin_costmap_and_safety_but_no_approach_stack():
    source = LAUNCH.read_text()

    assert "executable='controller_server'" in source
    assert "executable='recoveries_server'" in source
    assert "executable='search_motion_adapter'" in source
    assert "executable='motion_safety_supervisor_node'" in source
    assert "executable='cmd_vel_gate'" in source
    assert "executable='lifecycle_manager'" in source
    assert "executable='planner_server'" not in source
    assert "executable='bt_navigator'" not in source
    assert "executable='semantic_navigation_supervisor'" not in source


def test_every_nav2_velocity_output_is_remapped_to_raw_safety_input():
    source = LAUNCH.read_text()

    assert "('cmd_vel', '/nav2/cmd_vel_raw')" in source
    assert source.count('remappings=NAV2_CMD_REMAPPINGS') == 2
    assert "'/cmd_vel'" not in source
