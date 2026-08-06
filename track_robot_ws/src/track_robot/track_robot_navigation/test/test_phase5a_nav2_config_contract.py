from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PHASE4_CONFIG = PACKAGE_ROOT / 'config' / 'nav2_phase4b.yaml'
PHASE5_CONFIG = PACKAGE_ROOT / 'config' / 'nav2_phase5a.yaml'


def _load(path):
    return yaml.safe_load(path.read_text())


def test_phase5a_enables_only_bounded_spin_recovery():
    params = _load(PHASE5_CONFIG)
    recoveries = params['recoveries_server']['ros__parameters']

    assert recoveries['recovery_plugins'] == ['spin']
    assert recoveries['spin']['plugin'] == 'nav2_recoveries/Spin'
    assert recoveries['spin']['max_rotational_vel'] == 0.30
    assert 0.0 < recoveries['spin']['min_rotational_vel'] <= 0.10
    assert recoveries['spin']['rotational_acc_lim'] <= 0.50
    assert recoveries['spin']['simulate_ahead_time'] >= 1.0
    assert 'back_up' not in recoveries
    assert 'wait' not in recoveries


def test_phase5a_local_costmap_keeps_physical_footprint_and_live_clearing():
    params = _load(PHASE5_CONFIG)
    local = params['local_costmap']['local_costmap']['ros__parameters']

    assert local['global_frame'] == 'odom'
    assert local['robot_base_frame'] == 'base_link'
    assert local['rolling_window'] is True
    assert local['footprint'] == (
        '[[-0.44,-0.40],[-0.44,0.40],[0.44,0.40],[0.44,-0.40]]')
    assert local['footprint_padding'] == 0.0
    assert local['voxel_layer']['observation_sources'] == (
        'raw_clear filtered_mark')
    for source_name in ('raw_clear', 'filtered_mark'):
        source = local['voxel_layer'][source_name]
        assert source['observation_persistence'] == 0.0
        assert 0.0 < source['expected_update_rate'] <= 0.5


def test_phase5a_controller_exists_only_as_local_costmap_host():
    params = _load(PHASE5_CONFIG)
    controller = params['controller_server']['ros__parameters']

    assert controller['controller_plugins'] == ['FollowPath']
    assert controller['FollowPath']['desired_linear_vel'] == 0.15
    assert controller['FollowPath']['plugin'].endswith(
        'RegulatedPurePursuitController')


def test_phase4b_shared_runtime_supports_wait_and_bounded_spin():
    recoveries = _load(PHASE4_CONFIG)[
        'recoveries_server']['ros__parameters']

    assert recoveries['recovery_plugins'] == ['wait', 'spin']
    assert recoveries['spin']['plugin'] == 'nav2_recoveries/Spin'
    assert recoveries['spin']['max_rotational_vel'] == 0.30
    assert 'back_up' not in recoveries
