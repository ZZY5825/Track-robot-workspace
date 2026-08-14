import ast
from pathlib import Path
import xml.etree.ElementTree as ET


SENSOR_PACKAGE = Path(__file__).resolve().parents[1]
TRACK_ROBOT_SOURCE = SENSOR_PACKAGE.parent
WORKSPACE_SOURCE = TRACK_ROBOT_SOURCE.parent
BRINGUP_PACKAGE = TRACK_ROBOT_SOURCE / 'track_robot_bringup'
PERCEPTION_PACKAGE = WORKSPACE_SOURCE / 'track_robot_perception'

SENSOR_LAUNCH = SENSOR_PACKAGE / 'launch' / 'rslidar_with_tf.launch.py'
BRINGUP_LAUNCH = (
    BRINGUP_PACKAGE / 'launch' / 'rslidar_with_tf.launch.py'
)
SEMANTIC_SENSORS_LAUNCH = (
    BRINGUP_PACKAGE / 'launch' / 'semantic_search_sensors.launch.py'
)
SEMANTIC_LIVE_LAUNCH = (
    BRINGUP_PACKAGE / 'launch' / 'semantic_search_live.launch.py'
)
POINT_LIO_LAUNCH = (
    PERCEPTION_PACKAGE / 'launch' / 'point_lio_rshelios.launch.py'
)
FAST_LIO_LAUNCH = (
    PERCEPTION_PACKAGE / 'launch' / 'fast_lio_rshelios.launch.py'
)
POINT_LIO_DOC = PERCEPTION_PACKAGE / 'docs' / 'point_lio_rshelios.md'
FAST_LIO_DOC = PERCEPTION_PACKAGE / 'docs' / 'fast_lio_rshelios.md'

PUBLIC_ARGUMENTS = {
    'configure_network',
    'network_interface',
    'host_ip',
    'host_cidr',
    'driver_start_delay',
    'publish_base_lidar_tf',
    'config_path',
}


def _source(path):
    assert path.is_file(), 'required file is missing: {}'.format(path)
    return path.read_text(encoding='utf-8')


def _exec_dependencies(package):
    manifest = package / 'package.xml'
    assert manifest.is_file(), 'required package manifest is missing: {}'.format(
        manifest)
    return {
        element.text
        for element in ET.parse(str(manifest)).getroot().findall('exec_depend')
    }


def _tree(path):
    return ast.parse(_source(path))


def _declared_arguments(path):
    return {
        call.args[0].value
        for call in ast.walk(_tree(path))
        if (
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == 'DeclareLaunchArgument'
            and call.args
            and isinstance(call.args[0], ast.Constant)
        )
    }


def _forwards_launch_configuration(path, name):
    tree = _tree(path)
    launch_configuration_names = {
        target.id: call.args[0].value
        for assignment in ast.walk(tree)
        if (
            isinstance(assignment, ast.Assign)
            and len(assignment.targets) == 1
            and isinstance(assignment.targets[0], ast.Name)
            and isinstance(assignment.value, ast.Call)
            and isinstance(assignment.value.func, ast.Name)
            and assignment.value.func.id == 'LaunchConfiguration'
            and assignment.value.args
            and isinstance(assignment.value.args[0], ast.Constant)
        )
        for target in assignment.targets
        for call in (assignment.value,)
    }
    for item in ast.walk(tree):
        if not isinstance(item, ast.Dict):
            continue
        for key, value in zip(item.keys, item.values):
            if not isinstance(key, ast.Constant) or key.value != name:
                continue
            if (
                isinstance(value, ast.Call)
                and isinstance(value.func, ast.Name)
                and value.func.id == 'LaunchConfiguration'
                and value.args
                and isinstance(value.args[0], ast.Constant)
                and value.args[0].value == name
            ):
                return True
            if (
                isinstance(value, ast.Name)
                and launch_configuration_names.get(value.id) == name
            ):
                return True
    return False


def test_dependency_graph_has_no_bringup_perception_cycle():
    perception_dependencies = _exec_dependencies(PERCEPTION_PACKAGE)
    bringup_dependencies = _exec_dependencies(BRINGUP_PACKAGE)
    sensor_dependencies = _exec_dependencies(SENSOR_PACKAGE)

    assert 'track_robot_bringup' not in perception_dependencies
    assert 'track_robot_sensor_bringup' in perception_dependencies
    assert 'track_robot_sensor_bringup' in bringup_dependencies
    assert 'rslidar_sdk' not in bringup_dependencies
    assert sensor_dependencies == {
        'launch',
        'launch_ros',
        'rslidar_sdk',
        'tf2_ros',
    }


def test_bringup_launch_is_a_full_compatibility_wrapper():
    source = _source(BRINGUP_LAUNCH)

    assert _declared_arguments(BRINGUP_LAUNCH) == PUBLIC_ARGUMENTS
    assert 'IncludeLaunchDescription' in source
    assert 'rslidar_with_tf.launch.py' in source
    assert 'track_robot_sensor_bringup' in source
    for argument in PUBLIC_ARGUMENTS:
        assert _forwards_launch_configuration(BRINGUP_LAUNCH, argument)


def test_perception_resolves_shared_lidar_assets_from_sensor_package():
    point_lio_source = _source(POINT_LIO_LAUNCH)
    fast_lio_source = _source(FAST_LIO_LAUNCH)

    assert 'track_robot_sensor_bringup' in point_lio_source
    assert 'track_robot_bringup' not in point_lio_source
    assert 'track_robot_sensor_bringup' in fast_lio_source
    assert 'track_robot_bringup' not in fast_lio_source


def test_sensor_package_owns_motion_free_lidar_behavior():
    source = _source(SENSOR_LAUNCH)
    cmake_source = _source(SENSOR_PACKAGE / 'CMakeLists.txt')
    config = _source(
        SENSOR_PACKAGE / 'config' / 'rslidar_track_robot.yaml')

    assert _declared_arguments(SENSOR_LAUNCH) == PUBLIC_ARGUMENTS
    assert 'launch' in cmake_source
    assert 'config' in cmake_source
    assert 'rslidar_sdk_node' in source
    assert 'static_transform_publisher' in source
    assert 'base_link' in source
    assert 'rslidar' in source
    assert 'host_address: 192.168.1.102' in config
    assert 'ros_frame_id: lidar_link' in config
    assert 'track_robot_bringup' not in source
    assert 'track_robot_perception' not in source
    for forbidden in ('controller', '/cmd_vel'):
        assert forbidden not in source


def test_lidar_network_setup_reuses_an_already_configured_interface_without_sudo():
    source = _source(SENSOR_LAUNCH)

    preflight = 'ip -4 addr show dev '
    privileged_reconfigure = 'sudo -n ip addr flush dev '
    assert preflight in source
    assert 'already has expected LiDAR address' in source
    assert source.index(preflight) < source.index(privileged_reconfigure)


def test_sensor_contract_is_registered_with_ament():
    cmake_source = _source(SENSOR_PACKAGE / 'CMakeLists.txt')
    manifest_source = _source(SENSOR_PACKAGE / 'package.xml')

    assert 'ament_cmake_pytest' in manifest_source
    assert 'ament_add_pytest_test' in cmake_source
    assert 'test/test_sensor_bringup_contract.py' in cmake_source


def test_semantic_entrypoints_default_to_sensor_owned_lidar_config():
    for launch_file in (SEMANTIC_SENSORS_LAUNCH, SEMANTIC_LIVE_LAUNCH):
        source = _source(launch_file)
        assert "FindPackageShare('track_robot_sensor_bringup')" in source


def test_legacy_bringup_config_copy_is_removed():
    legacy_config = (
        BRINGUP_PACKAGE / 'config' / 'rslidar_track_robot.yaml')
    assert not legacy_config.exists()


def test_perception_docs_name_the_sensor_launch_owner():
    for doc in (POINT_LIO_DOC, FAST_LIO_DOC):
        source = _source(doc)
        assert 'track_robot_sensor_bringup' in source
        assert 'track_robot_bringup/rslidar_with_tf.launch.py' not in source
        assert (
            'ros2 launch track_robot_bringup rslidar_with_tf.launch.py'
            not in source
        )
