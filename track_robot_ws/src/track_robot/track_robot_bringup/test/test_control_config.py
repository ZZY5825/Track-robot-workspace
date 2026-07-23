import pytest

from track_robot_bringup.control_config import (
    HardwareSelection,
    default_workspace_paths,
    managed_environment,
    resolve_stage,
)


def test_phase1_requires_only_camera():
    spec = resolve_stage('phase1')

    assert spec.camera is True
    assert spec.lidar is False
    assert spec.base is False
    assert spec.imu is False


def test_phase2_requires_all_passive_hardware():
    spec = resolve_stage('phase2')

    assert (spec.camera, spec.lidar, spec.base, spec.imu) == (
        True, True, True, True)


def test_managed_environment_uses_domain_20_and_preserves_parent():
    result = managed_environment({'PATH': '/bin'}, dds_profile='/tmp/dds.xml')

    assert result['PATH'] == '/bin'
    assert result['ROS_DOMAIN_ID'] == '20'
    assert result['FASTRTPS_DEFAULT_PROFILES_FILE'] == '/tmp/dds.xml'


def test_managed_environment_rejects_a_domain_other_than_20():
    with pytest.raises(ValueError, match='must be 20'):
        managed_environment({}, domain_id=0)


@pytest.mark.parametrize('base', [{1: 'value'}, {'KEY': 1}])
def test_managed_environment_rejects_non_string_base_entries(base):
    with pytest.raises(ValueError, match='keys and values must be strings'):
        managed_environment(base)


def test_hardware_selection_is_immutable_and_explicit():
    selection = HardwareSelection(camera=True, lidar=False, base=False, imu=False)

    assert (selection.camera, selection.lidar, selection.base, selection.imu) == (
        True, False, False, False)


def test_default_workspace_paths_are_relative_to_the_workspace_root(tmp_path):
    paths = default_workspace_paths(tmp_path)

    assert paths['runtime_path'] == tmp_path / 'models' / 'phase1_runtime' / 'python'
    assert paths['checkpoint_path'] == tmp_path / 'models' / 'phase1' / 'ViT-B-32.pt'
    assert paths['dds_profile'] == (
        tmp_path / 'src' / 'track_robot' / 'track_robot_bringup' / 'config'
        / 'fastdds_semantic_search.xml')
