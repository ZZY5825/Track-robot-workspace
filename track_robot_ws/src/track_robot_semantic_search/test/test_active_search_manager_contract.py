from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MANAGER_SOURCE = (
    PACKAGE_ROOT
    / 'track_robot_semantic_search'
    / 'active_search_manager_node.py'
)
CONFIG = PACKAGE_ROOT / 'config' / 'semantic_search_phase5a.yaml'
SETUP = PACKAGE_ROOT / 'setup.py'


def test_manager_uses_existing_action_query_and_intent_contracts():
    source = MANAGER_SOURCE.read_text()

    assert 'ActionServer' in source
    assert 'SearchForObject' in source
    assert "'/semantic_search/query'" in source
    assert 'QueryRequest.create' in source
    assert 'SearchMotionIntent' in source
    assert "'/semantic_search/search_motion_intent'" in source


def test_manager_cannot_publish_velocity_or_call_navigation():
    source = MANAGER_SOURCE.read_text()
    forbidden = (
        'geometry_msgs.msg import Twist',
        "'/cmd_vel'",
        'NavigateToPose',
        'ActionClient',
        'nav2_msgs',
    )

    assert all(token not in source for token in forbidden)


def test_manager_observes_authoritative_phase2_and_phase3_outputs():
    source = MANAGER_SOURCE.read_text()

    assert "'/semantic_memory/tasks'" in source
    assert "'/semantic_memory/active_objects'" in source
    assert "'/semantic_memory/diagnostic_ranking'" in source
    assert "'/semantic_search/phase4a/selected_target'" in source
    assert 'on_camera_info' in source
    assert 'ObjectEvidenceKey' in source


def test_phase5a_config_is_passive_bounded_and_fail_closed():
    params = yaml.safe_load(CONFIG.read_text())[
        'active_search_manager']['ros__parameters']

    assert params['search_mode'] == 'PASSIVE_ONLY'
    assert params['active_search_execution_enabled'] is False
    assert params['heading_offsets_deg'] == [45.0, 90.0, 0.0, -45.0, -90.0]
    assert params['maximum_individual_rotation_deg'] == 90.0
    assert params['maximum_cumulative_rotation_deg'] == 270.0
    assert params['maximum_angular_speed_rad_s'] == 0.30
    assert params['settle_duration_sec'] == 0.75
    assert params['settle_angular_speed_rad_s'] == 0.03
    assert params['observation_timeout_sec'] >= 4.5
    assert params['confirmation_snapshots'] == 3
    assert params['evidence_ttl_sec'] == 12.0
    assert params['default_task_timeout_sec'] == 60.0


def test_setup_installs_one_active_search_manager_executable():
    setup = SETUP.read_text()

    assert 'semantic_search_active_manager = ' in setup
    assert 'active_search_manager_node:main' in setup


def test_manager_shutdown_handles_operator_interrupt_cleanly():
    source = MANAGER_SOURCE.read_text()

    assert 'except KeyboardInterrupt:' in source


def test_manager_binds_evidence_even_after_active_object_epoch_arrives():
    source = MANAGER_SOURCE.read_text()

    assert 'if not context.evidence.is_bound:' in source


def test_manager_closes_the_spin_settle_observe_loop_and_stops_at_terminal():
    source = MANAGER_SOURCE.read_text()

    assert "state == 'SPIN_COMPLETED'" in source
    assert 'context.policy.mark_completed(decision)' in source
    assert 'def on_odom(' in source
    assert 'settle_angular_speed_rad_s' in source
    assert "publish_stop_intent('task_terminal')" in source
