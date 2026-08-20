import ast
import importlib.util
from collections import Counter
from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_ROOT = PACKAGE_ROOT / 'launch'
HARDWARE_LAUNCH = LAUNCH_ROOT / 'track_robot_hardware.launch.py'
SENSORS_LAUNCH = LAUNCH_ROOT / 'semantic_search_sensors.launch.py'
PLATFORM_LAUNCH = LAUNCH_ROOT / 'semantic_search_platform.launch.py'
LIVE_LAUNCH = LAUNCH_ROOT / 'semantic_search_live.launch.py'
PHASE4A_LAUNCH = LAUNCH_ROOT / 'semantic_search_phase4a.launch.py'
PHASE4B_LAUNCH = LAUNCH_ROOT / 'semantic_search_phase4b.launch.py'
PHASE5A_LAUNCH = LAUNCH_ROOT / 'semantic_search_phase5a.launch.py'

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


def _load_launch_module(path):
    module_name = 'shared_hardware_contract_{}'.format(
        path.stem.replace('.', '_').replace('-', '_'))
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _launch_actions(path, overrides=None):
    module = _load_launch_module(path)
    launch_description = module.generate_launch_description()
    context = LaunchContext()
    values = {
        argument: default
        for argument, default in _argument_defaults(path).items()
    }
    values.update(overrides or {})
    for argument in _declared_arguments(path):
        context.launch_configurations[argument] = values.get(
            argument, '__unused_contract_value__')

    actions = []
    for entity in launch_description.entities:
        if isinstance(entity, IncludeLaunchDescription):
            actions.append(entity)
        elif isinstance(entity, OpaqueFunction):
            actions.extend(entity.execute(context))
    return context, actions


def _perform(context, value):
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(value))


def _include_identity(include, context):
    source = include.launch_description_source
    location = source.__dict__['_LaunchDescriptionSource__location']
    assert len(location) == 1
    path_join = location[0]
    parts = path_join.__dict__['_PathJoinSubstitution__substitutions']
    package_substitution = parts[0]
    package = _perform(
        context, package_substitution.__dict__['_FindPackage__package'])
    launch_file = _perform(context, parts[-1])
    return package, launch_file


def _effective_launch_arguments(include, context):
    return {
        name: _perform(context, value)
        for name, value in include.launch_arguments
    }


def _active_physical_includes(path, overrides=None):
    active = []

    def walk(launch_path, launch_overrides):
        context, actions = _launch_actions(launch_path, launch_overrides)
        for action in actions:
            if not isinstance(action, IncludeLaunchDescription):
                continue
            if action.condition is not None and not action.condition.evaluate(
                    context):
                continue

            identity = _include_identity(action, context)
            if identity in {
                    (package, launch_file)
                    for package, launch_file, _ in PHYSICAL_INCLUDES}:
                active.append(identity)
                continue

            package, launch_file = identity
            if package != 'track_robot_bringup':
                continue
            child_launch = LAUNCH_ROOT / launch_file
            if child_launch.is_file():
                walk(child_launch, _effective_launch_arguments(action, context))

    walk(path, overrides or {})
    return Counter(active)


def _active_include_arguments(path, overrides=None, identities=None):
    context, actions = _launch_actions(path, overrides)
    return {
        _include_identity(action, context): _effective_launch_arguments(
            action, context)
        for action in actions
        if isinstance(action, IncludeLaunchDescription)
        and (action.condition is None or action.condition.evaluate(context))
        and (identities is None
             or _include_identity(action, context) in identities)
    }


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


def test_hardware_tf_ownership_expressions_resolve_for_both_description_modes():
    camera = (
        'track_robot_bringup', 'semantic_search_camera.launch.py')
    lidar = ('track_robot_bringup', 'rslidar_with_tf.launch.py')

    description_owned = _active_include_arguments(HARDWARE_LAUNCH, {
        'start_description': 'true',
        'extrinsic_mode': 'measured',
        'publish_base_lidar_tf': 'true',
    }, {camera, lidar})
    assert description_owned[camera]['extrinsic_mode'] == 'robot_description'
    assert description_owned[lidar]['publish_base_lidar_tf'] == 'false'

    standalone_sensors = _active_include_arguments(HARDWARE_LAUNCH, {
        'start_description': 'false',
        'extrinsic_mode': 'measured',
        'publish_base_lidar_tf': 'true',
    }, {camera, lidar})
    assert standalone_sensors[camera]['extrinsic_mode'] == 'measured'
    assert standalone_sensors[lidar]['publish_base_lidar_tf'] == 'true'


def test_composed_entrypoints_activate_each_physical_driver_once_when_enabled():
    expected_physical_topologies = (
        (
            LIVE_LAUNCH,
            {},
            {
                ('bunker_pro2', 'description.launch.py'): 1,
                ('track_robot_bringup', 'semantic_search_camera.launch.py'): 1,
                ('track_robot_bringup', 'rslidar_with_tf.launch.py'): 0,
                ('bunker_base', 'bunker_base.launch.py'): 0,
                ('track_robot_perception', 'phidget_imu.launch.py'): 0,
            },
        ),
        (
            LIVE_LAUNCH,
            {'stage': 'sensors'},
            {
                ('bunker_pro2', 'description.launch.py'): 1,
                ('track_robot_bringup', 'semantic_search_camera.launch.py'): 1,
                ('track_robot_bringup', 'rslidar_with_tf.launch.py'): 1,
                ('bunker_base', 'bunker_base.launch.py'): 1,
                ('track_robot_perception', 'phidget_imu.launch.py'): 1,
            },
        ),
        (
            PHASE4A_LAUNCH,
            {},
            {
                ('bunker_pro2', 'description.launch.py'): 1,
                ('track_robot_bringup', 'semantic_search_camera.launch.py'): 1,
                ('track_robot_bringup', 'rslidar_with_tf.launch.py'): 1,
                ('bunker_base', 'bunker_base.launch.py'): 0,
                ('track_robot_perception', 'phidget_imu.launch.py'): 0,
            },
        ),
        (
            PHASE4B_LAUNCH,
            {},
            {
                ('bunker_pro2', 'description.launch.py'): 1,
                ('track_robot_bringup', 'semantic_search_camera.launch.py'): 1,
                ('track_robot_bringup', 'rslidar_with_tf.launch.py'): 1,
                ('bunker_base', 'bunker_base.launch.py'): 0,
                ('track_robot_perception', 'phidget_imu.launch.py'): 0,
            },
        ),
        (
            PHASE5A_LAUNCH,
            {},
            {
                ('bunker_pro2', 'description.launch.py'): 1,
                ('track_robot_bringup', 'semantic_search_camera.launch.py'): 1,
                ('track_robot_bringup', 'rslidar_with_tf.launch.py'): 1,
                ('bunker_base', 'bunker_base.launch.py'): 0,
                ('track_robot_perception', 'phidget_imu.launch.py'): 0,
            },
        ),
        (
            PHASE4B_LAUNCH,
            {'start_base': 'true'},
            {
                ('bunker_pro2', 'description.launch.py'): 1,
                ('track_robot_bringup', 'semantic_search_camera.launch.py'): 1,
                ('track_robot_bringup', 'rslidar_with_tf.launch.py'): 1,
                ('bunker_base', 'bunker_base.launch.py'): 1,
                ('track_robot_perception', 'phidget_imu.launch.py'): 0,
            },
        ),
        (
            PHASE5A_LAUNCH,
            {'start_base': 'true'},
            {
                ('bunker_pro2', 'description.launch.py'): 1,
                ('track_robot_bringup', 'semantic_search_camera.launch.py'): 1,
                ('track_robot_bringup', 'rslidar_with_tf.launch.py'): 1,
                ('bunker_base', 'bunker_base.launch.py'): 1,
                ('track_robot_perception', 'phidget_imu.launch.py'): 0,
            },
        ),
    )

    for launch, overrides, expected in expected_physical_topologies:
        active = _active_physical_includes(launch, overrides)
        assert {
            (package, launch_file): active[(package, launch_file)]
            for package, launch_file, _ in PHYSICAL_INCLUDES
        } == expected
