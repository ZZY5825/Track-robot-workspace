import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_ROOT = PACKAGE_ROOT / 'launch'
HARDWARE_LAUNCH = LAUNCH_ROOT / 'track_robot_hardware.launch.py'
SENSORS_LAUNCH = LAUNCH_ROOT / 'semantic_search_sensors.launch.py'
PLATFORM_LAUNCH = LAUNCH_ROOT / 'semantic_search_platform.launch.py'
LIVE_LAUNCH = LAUNCH_ROOT / 'semantic_search_live.launch.py'

PHYSICAL_INCLUDES = (
    ('bunker_pro2', 'description.launch.py', 'start_description'),
    (
        'track_robot_bringup',
        'semantic_search_camera.launch.py',
        'start_camera',
    ),
    (
        'track_robot_bringup',
        'rslidar_with_tf.launch.py',
        'start_lidar',
    ),
    ('bunker_base', 'bunker_base.launch.py', 'start_base'),
    ('track_robot_perception', 'phidget_imu.launch.py', 'start_imu'),
)

SENSORS_ARGUMENT_DEFAULTS = {
    'start_camera': 'true',
    'start_lidar': 'true',
    'start_base': 'true',
    'start_imu': 'true',
    'base_frame': 'base_link',
    'configure_network': 'true',
    'network_interface': 'eth0',
    'host_ip': '192.168.1.102',
    'host_cidr': '24',
    'driver_start_delay': '1.0',
    'publish_base_lidar_tf': 'true',
    'extrinsic_mode': 'none',
    'extrinsic_file': '',
    'allow_degraded': 'false',
    'camera_depth_mode': 'NONE',
}


def _source(path):
    assert path.is_file(), 'required launch file is missing: {}'.format(path)
    return path.read_text(encoding='utf-8')


def _tree(path):
    return ast.parse(_source(path))


def _calls(path, function_name):
    return [
        item
        for item in ast.walk(_tree(path))
        if isinstance(item, ast.Call)
        and isinstance(item.func, ast.Name)
        and item.func.id == function_name
    ]


def _keyword(call, name):
    return next(
        (keyword.value for keyword in call.keywords if keyword.arg == name),
        None,
    )


def _contains_string(node, value):
    return any(
        isinstance(item, ast.Constant) and item.value == value
        for item in ast.walk(node)
    )


def _declared_arguments(path):
    return {
        call.args[0].value
        for call in _calls(path, 'DeclareLaunchArgument')
        if call.args and isinstance(call.args[0], ast.Constant)
    }


def _argument_defaults(path):
    defaults = {}
    for call in _calls(path, 'DeclareLaunchArgument'):
        if not call.args or not isinstance(call.args[0], ast.Constant):
            continue
        default = _keyword(call, 'default_value')
        if isinstance(default, ast.Constant):
            defaults[call.args[0].value] = default.value
    return defaults


def _include_for(path, package, launch_file):
    return [
        call
        for call in _calls(path, 'IncludeLaunchDescription')
        if _contains_string(call, package)
        and _contains_string(call, launch_file)
    ]


def _launch_arguments(include):
    launch_arguments = _keyword(include, 'launch_arguments')
    argument_dict = next(
        (
            item for item in ast.walk(launch_arguments)
            if isinstance(item, ast.Dict)
        ),
        None,
    )
    assert argument_dict is not None
    return {
        key.value: value
        for key, value in zip(argument_dict.keys, argument_dict.values)
        if isinstance(key, ast.Constant)
    }


def _assert_launch_configuration(value, name):
    assert isinstance(value, ast.Call)
    assert isinstance(value.func, ast.Name)
    assert value.func.id == 'LaunchConfiguration'
    assert _contains_string(value, name)


def _assert_if_condition(call, launch_argument):
    condition = _keyword(call, 'condition')
    assert isinstance(condition, ast.Call)
    assert isinstance(condition.func, ast.Name)
    assert condition.func.id == 'IfCondition'
    assert _contains_string(condition, launch_argument)


def test_neutral_hardware_launch_has_one_conditional_include_per_module():
    includes = _calls(HARDWARE_LAUNCH, 'IncludeLaunchDescription')

    assert len(includes) == len(PHYSICAL_INCLUDES)
    for package, launch_file, start_flag in PHYSICAL_INCLUDES:
        matching = _include_for(HARDWARE_LAUNCH, package, launch_file)
        assert len(matching) == 1
        _assert_if_condition(matching[0], start_flag)


def test_neutral_hardware_launch_preserves_safe_defaults_and_tf_ownership():
    expected_arguments = {
        'start_description',
        'start_camera',
        'start_lidar',
        'start_base',
        'start_imu',
        'base_frame',
        'camera_depth_mode',
        'configure_network',
        'network_interface',
        'host_ip',
        'host_cidr',
        'driver_start_delay',
        'publish_base_lidar_tf',
        'lidar_config_path',
        'extrinsic_mode',
        'extrinsic_file',
        'allow_degraded',
    }

    assert _declared_arguments(HARDWARE_LAUNCH) == expected_arguments
    assert _argument_defaults(HARDWARE_LAUNCH) == {
        'start_description': 'true',
        'start_camera': 'true',
        'start_lidar': 'true',
        'start_base': 'true',
        'start_imu': 'true',
        'base_frame': 'base_link',
        'camera_depth_mode': 'NONE',
        'configure_network': 'true',
        'network_interface': 'eth0',
        'host_ip': '192.168.1.102',
        'host_cidr': '24',
        'driver_start_delay': '1.0',
        'publish_base_lidar_tf': 'false',
        'extrinsic_mode': 'robot_description',
        'extrinsic_file': '',
        'allow_degraded': 'false',
    }
    source = _source(HARDWARE_LAUNCH)
    assert "FindPackageShare('track_robot_sensor_bringup')" in source
    assert "'rslidar_track_robot.yaml'" in source

    camera_include = _include_for(
        HARDWARE_LAUNCH,
        'track_robot_bringup',
        'semantic_search_camera.launch.py',
    )[0]
    camera_mode = _launch_arguments(camera_include)['extrinsic_mode']
    assert isinstance(camera_mode, ast.Call)
    assert isinstance(camera_mode.func, ast.Name)
    assert camera_mode.func.id == 'PythonExpression'
    assert _contains_string(camera_mode, 'start_description')
    assert _contains_string(camera_mode, 'robot_description')
    assert _contains_string(camera_mode, 'extrinsic_mode')

    lidar_include = _include_for(
        HARDWARE_LAUNCH,
        'track_robot_bringup',
        'rslidar_with_tf.launch.py',
    )[0]
    publish_tf = _launch_arguments(
        lidar_include)['publish_base_lidar_tf']
    assert isinstance(publish_tf, ast.Call)
    assert isinstance(publish_tf.func, ast.Name)
    assert publish_tf.func.id == 'PythonExpression'
    assert _contains_string(publish_tf, 'start_description')
    assert _contains_string(publish_tf, 'false')
    assert _contains_string(publish_tf, 'publish_base_lidar_tf')


def test_neutral_hardware_launch_contains_no_feature_or_control_actions():
    source = _source(HARDWARE_LAUNCH).lower()

    assert not _calls(HARDWARE_LAUNCH, 'Node')
    assert not _calls(HARDWARE_LAUNCH, 'ExecuteProcess')
    for forbidden in (
            'human_tracking',
            'navigation',
            'controller',
            'planner',
            'safety',
            'cmd_vel'):
        assert forbidden not in source


def test_sensors_wrapper_preserves_public_contract_and_disables_description():
    expected_arguments = set(SENSORS_ARGUMENT_DEFAULTS) | {
        'lidar_config_path',
    }
    includes = _calls(SENSORS_LAUNCH, 'IncludeLaunchDescription')

    assert _declared_arguments(SENSORS_LAUNCH) == expected_arguments
    assert _argument_defaults(SENSORS_LAUNCH) == SENSORS_ARGUMENT_DEFAULTS
    assert len(includes) == 1
    assert len(_include_for(
        SENSORS_LAUNCH,
        'track_robot_bringup',
        'track_robot_hardware.launch.py',
    )) == 1
    arguments = _launch_arguments(includes[0])
    assert isinstance(arguments['start_description'], ast.Constant)
    assert arguments['start_description'].value == 'false'
    for argument in expected_arguments:
        _assert_launch_configuration(arguments[argument], argument)


def test_platform_wrapper_preserves_public_contract_and_selects_base_imu_only():
    includes = _calls(PLATFORM_LAUNCH, 'IncludeLaunchDescription')

    assert _declared_arguments(PLATFORM_LAUNCH) == {
        'start_base',
        'start_imu',
        'base_frame',
    }
    assert _argument_defaults(PLATFORM_LAUNCH) == {
        'start_base': 'true',
        'start_imu': 'true',
        'base_frame': 'base_link',
    }
    assert len(includes) == 1
    assert len(_include_for(
        PLATFORM_LAUNCH,
        'track_robot_bringup',
        'track_robot_hardware.launch.py',
    )) == 1
    arguments = _launch_arguments(includes[0])
    assert set(arguments) == {
        'start_description',
        'start_camera',
        'start_lidar',
        'start_base',
        'start_imu',
        'base_frame',
    }
    for argument in ('start_description', 'start_camera', 'start_lidar'):
        assert isinstance(arguments[argument], ast.Constant)
        assert arguments[argument].value == 'false'
    for argument in ('start_base', 'start_imu', 'base_frame'):
        _assert_launch_configuration(arguments[argument], argument)


def test_wrappers_do_not_duplicate_physical_drivers_in_the_composed_ast():
    physical_launch_files = {
        launch_file for _, launch_file, _ in PHYSICAL_INCLUDES
    }

    for wrapper in (SENSORS_LAUNCH, PLATFORM_LAUNCH):
        source = _source(wrapper)
        assert source.count('track_robot_hardware.launch.py') == 1
        for launch_file in physical_launch_files:
            assert launch_file not in source

    live_source = _source(LIVE_LAUNCH)
    assert live_source.count("'description.launch.py'") == 1
    sensors_include = _calls(SENSORS_LAUNCH, 'IncludeLaunchDescription')[0]
    assert _launch_arguments(sensors_include)['start_description'].value == (
        'false')
