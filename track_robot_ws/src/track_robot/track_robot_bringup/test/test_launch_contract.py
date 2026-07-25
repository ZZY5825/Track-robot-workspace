import ast
import importlib.util
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_ROOT = PACKAGE_ROOT / 'launch'
CONFIG_ROOT = PACKAGE_ROOT / 'config'
CAMERA_LAUNCH = LAUNCH_ROOT / 'semantic_search_camera.launch.py'
PLATFORM_LAUNCH = LAUNCH_ROOT / 'semantic_search_platform.launch.py'
SENSORS_LAUNCH = LAUNCH_ROOT / 'semantic_search_sensors.launch.py'
LIVE_LAUNCH = LAUNCH_ROOT / 'semantic_search_live.launch.py'
VISUALIZATION_LAUNCH = (
    LAUNCH_ROOT / 'semantic_search_visualization.launch.py')
RVIZ_ROOT = PACKAGE_ROOT / 'rviz'
PHASE1_RVIZ = RVIZ_ROOT / 'semantic_search_phase1.rviz'
PHASE2_RVIZ = RVIZ_ROOT / 'semantic_search_phase2.rviz'
RSLIDAR_IMPLEMENTATION = (
    PACKAGE_ROOT.parent
    / 'track_robot_sensor_bringup'
    / 'launch'
    / 'rslidar_with_tf.launch.py'
)
EXTRINSIC_CONFIG = CONFIG_ROOT / 'camera_extrinsic.example.yaml'
MODULAR_LAUNCHES = (
    CAMERA_LAUNCH,
    PLATFORM_LAUNCH,
    SENSORS_LAUNCH,
    LIVE_LAUNCH,
)


def _source(path):
    assert path.is_file(), 'required launch file is missing: {}'.format(path)
    return path.read_text(encoding='utf-8')


def _tree(path):
    return ast.parse(_source(path))


def _load_camera_launch_module():
    spec = importlib.util.spec_from_file_location(
        'semantic_search_camera_launch_contract', CAMERA_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _declared_arguments(path):
    return {
        call.args[0].value
        for call in ast.walk(_tree(path))
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'DeclareLaunchArgument'
        and call.args
        and isinstance(call.args[0], ast.Constant)
    }


def _argument_defaults(path):
    defaults = {}
    for call in ast.walk(_tree(path)):
        if (
                not isinstance(call, ast.Call)
                or not isinstance(call.func, ast.Name)
                or call.func.id != 'DeclareLaunchArgument'
                or not call.args
                or not isinstance(call.args[0], ast.Constant)):
            continue
        default = _keyword(call, 'default_value')
        if isinstance(default, ast.Constant):
            defaults[call.args[0].value] = default.value
    return defaults


def _calls(path, function_name):
    return [
        item
        for item in ast.walk(_tree(path))
        if isinstance(item, ast.Call)
        and isinstance(item.func, ast.Name)
        and item.func.id == function_name
    ]


def _included_launch_files_in_source_order(path):
    includes = [
        call
        for call in ast.walk(_tree(path))
        if (
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == '_include'
            and len(call.args) >= 2
            and isinstance(call.args[1], ast.Constant)
        )
    ]
    return [
        call.args[1].value
        for call in sorted(includes, key=lambda call: call.lineno)
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


def _forwards_launch_configuration(path, name):
    for item in ast.walk(_tree(path)):
        if not isinstance(item, ast.Dict):
            continue
        for key, value in zip(item.keys, item.values):
            if not isinstance(key, ast.Constant) or key.value != name:
                continue
            if (
                    isinstance(value, ast.Call)
                    and isinstance(value.func, ast.Name)
                    and value.func.id == 'LaunchConfiguration'
                    and _contains_string(value, name)):
                return True
    return False


def _assert_if_condition(call, launch_argument):
    condition = _keyword(call, 'condition')
    assert isinstance(condition, ast.Call)
    assert isinstance(condition.func, ast.Name)
    assert condition.func.id == 'IfCondition'
    assert _contains_string(condition, launch_argument)


def test_modular_launches_exist_and_remain_passive():
    combined_source = '\n'.join(_source(path) for path in MODULAR_LAUNCHES)

    for forbidden in (
            'cmd_vel', 'controller', 'planner', 'safe_human_following'):
        assert forbidden not in combined_source


def test_live_launch_exposes_public_contract_and_composes_all_phases():
    required_live_arguments = {
        'stage',
        'start_camera',
        'start_lidar',
        'start_base',
        'start_imu',
        'runtime_path',
        'checkpoint_path',
        'extrinsic_mode',
        'extrinsic_file',
        'allow_degraded',
    }
    source = _source(LIVE_LAUNCH)

    assert required_live_arguments <= _declared_arguments(LIVE_LAUNCH)
    assert 'semantic_search_phase1.launch.py' in source
    assert 'semantic_search_phase0.launch.py' in source
    assert 'semantic_memory_lidar_tracklets.launch.py' in source
    assert 'semantic_memory_phase2.launch.py' in source
    assert 'models/phase1_runtime/python' in source
    assert 'models/phase1/ViT-B-32.pt' in source
    defaults = _argument_defaults(LIVE_LAUNCH)
    assert defaults['runtime_path'] == (
        '/home/track-robot/track_robot_ws/models/phase1_runtime/python')
    assert defaults['checkpoint_path'] == (
        '/home/track-robot/track_robot_ws/models/phase1/ViT-B-32.pt')
    assert _calls(LIVE_LAUNCH, 'OpaqueFunction')
    for stage in ('sensors', 'phase1', 'phase2'):
        assert stage in source
    assert _included_launch_files_in_source_order(LIVE_LAUNCH) == [
        'semantic_search_sensors.launch.py',
        'semantic_search_phase0.launch.py',
        'semantic_search_phase1.launch.py',
        'semantic_memory_lidar_tracklets.launch.py',
        'semantic_memory_phase2.launch.py',
    ]
    assert source.index("if stage == 'phase2':") < source.index(
        "if stage in ('phase1', 'phase2'):")


def test_visualization_launch_is_foreground_passive_and_closes_with_rviz():
    source = _source(VISUALIZATION_LAUNCH)

    assert 'semantic_search_live_overlay' in source
    assert "package='rviz2'" in source
    assert "executable='rviz2'" in source
    assert 'semantic_search_phase1.rviz' in source
    assert 'semantic_search_phase2.rviz' in source
    assert 'OnProcessExit' in source
    assert 'EmitEvent' in source
    assert 'Shutdown' in source
    for forbidden in ('cmd_vel', 'controller', 'planner'):
        assert forbidden not in source


def test_saved_rviz_views_use_live_topics_and_custom_panel():
    phase1 = _source(PHASE1_RVIZ)
    phase2 = _source(PHASE2_RVIZ)
    panel = (
        'track_robot_semantic_search_rviz_plugins/'
        'SemanticSearchPanel'
    )

    assert panel in phase1
    assert panel in phase2
    assert '/semantic_search/overlay_image' in phase1
    assert 'zed_left_camera_optical_frame' in phase1
    assert '/semantic_search/overlay_image' in phase2
    assert '/rslidar_points' in phase2
    assert '/semantic_memory/markers' in phase2
    assert 'Fixed Frame: odom' in phase2


def test_camera_launch_is_opt_in_and_disables_zed_owned_tf_edges():
    source = _source(CAMERA_LAUNCH)
    includes = _calls(CAMERA_LAUNCH, 'IncludeLaunchDescription')

    assert len(includes) == 1
    assert 'zed_camera.launch.py' in source
    assert "'camera_model': 'zed2i'" in source
    assert "'publish_tf': 'false'" in source
    assert "'publish_map_tf': 'false'" in source
    assert "'publish_urdf': 'true'" in source
    _assert_if_condition(includes[0], 'start_camera')


def test_camera_extrinsic_policy_is_fail_closed_and_tf_is_conditional(
        tmp_path, monkeypatch):
    source = _source(CAMERA_LAUNCH)

    assert _calls(CAMERA_LAUNCH, 'OpaqueFunction')
    assert source.index(
        'OpaqueFunction(function=_launch_extrinsic)'
    ) < source.index('        zed_camera,')
    assert 'prototype camera extrinsic requires allow_degraded:=true' in source
    assert 'measured camera extrinsic file does not exist:' in source
    assert 'extrinsic_mode' in _declared_arguments(CAMERA_LAUNCH)
    assert 'extrinsic_file' in _declared_arguments(CAMERA_LAUNCH)
    assert 'allow_degraded' in _declared_arguments(CAMERA_LAUNCH)
    assert "mode == 'prototype'" in source
    assert "mode == 'measured'" in source
    assert '.is_file()' in source
    assert "'x': 0.27" in source
    assert "'z': 0.62" in source

    static_publishers = [
        call for call in _calls(CAMERA_LAUNCH, 'Node')
        if _contains_string(call, 'static_transform_publisher')
    ]
    assert len(static_publishers) == 1
    assert _keyword(static_publishers[0], 'condition') is not None

    camera_launch = _load_camera_launch_module()
    valid_config = {
        'calibration_id': 'calibration-2026-07-23',
        'parent_frame': 'base_link',
        'child_frame': 'zed_camera_link',
        'translation': {'x': 0.27, 'y': 0.0, 'z': 0.62},
        'rotation_rpy': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    }
    calibration = tmp_path / 'camera_extrinsic.yaml'

    invalid_configs = [
        {
            **valid_config,
            'calibration_id': 'replace_with_measured_calibration_id',
        },
        {**valid_config, 'parent_frame': 'map'},
        {**valid_config, 'child_frame': 'zed_camera_frame'},
    ]
    for field_group, field_name in (
            ('translation', 'x'), ('translation', 'y'), ('translation', 'z'),
            ('rotation_rpy', 'roll'), ('rotation_rpy', 'pitch'),
            ('rotation_rpy', 'yaw')):
        for non_finite_value in (float('nan'), float('inf')):
            config = yaml.safe_load(yaml.safe_dump(valid_config))
            config[field_group][field_name] = non_finite_value
            invalid_configs.append(config)

    for invalid_config in invalid_configs:
        calibration.write_text(yaml.safe_dump(invalid_config), encoding='utf-8')
        with pytest.raises(RuntimeError, match='measured camera extrinsic file is invalid'):
            camera_launch._measured_extrinsic(calibration)

    calibration.write_text(yaml.safe_dump(valid_config), encoding='utf-8')
    assert camera_launch._measured_extrinsic(calibration) == {
        'parent_frame': 'base_link',
        'child_frame': 'zed_camera_link',
        'x': 0.27,
        'y': 0.0,
        'z': 0.62,
        'roll': 0.0,
        'pitch': 0.0,
        'yaw': 0.0,
    }

    def _raise_os_error(self, *args, **kwargs):
        raise OSError('read failure')

    monkeypatch.setattr(camera_launch.Path, 'read_text', _raise_os_error)
    with pytest.raises(RuntimeError, match='measured camera extrinsic file is invalid'):
        camera_launch._measured_extrinsic(calibration)


def test_camera_extrinsic_example_has_the_calibration_schema():
    assert EXTRINSIC_CONFIG.is_file()
    config = yaml.safe_load(EXTRINSIC_CONFIG.read_text(encoding='utf-8'))

    assert config == {
        'calibration_id': 'replace_with_measured_calibration_id',
        'parent_frame': 'base_link',
        'child_frame': 'zed_camera_link',
        'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'rotation_rpy': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    }


def test_platform_directly_and_independently_gates_base_and_imu():
    source = _source(PLATFORM_LAUNCH)
    includes = _calls(PLATFORM_LAUNCH, 'IncludeLaunchDescription')

    assert 'bunker_base.launch.py' in source
    assert 'phidget_imu.launch.py' in source
    assert 'jetson_base.launch.py' not in source
    assert {'start_base', 'start_imu'} <= _declared_arguments(PLATFORM_LAUNCH)
    assert len(includes) == 2
    assert any(
        _contains_string(call, 'start_base')
        and _keyword(call, 'condition') is not None
        for call in includes
    )
    assert any(
        _contains_string(call, 'start_imu')
        and _keyword(call, 'condition') is not None
        for call in includes
    )
    assert '<exec_depend>track_robot_perception</exec_depend>' in _source(
        PACKAGE_ROOT / 'package.xml')


def test_sensors_launch_gates_each_hardware_module_and_forwards_arguments():
    source = _source(SENSORS_LAUNCH)
    arguments = _declared_arguments(SENSORS_LAUNCH)

    assert {
        'start_camera',
        'start_lidar',
        'start_base',
        'start_imu',
        'configure_network',
        'network_interface',
        'host_ip',
        'host_cidr',
        'extrinsic_mode',
        'extrinsic_file',
        'allow_degraded',
    } <= arguments
    assert 'semantic_search_camera.launch.py' in source
    assert 'semantic_search_platform.launch.py' in source
    assert 'rslidar_with_tf.launch.py' in source
    for argument in ('start_camera', 'start_lidar', 'start_base', 'start_imu'):
        assert argument in source


def test_lidar_tf_ownership_is_forwarded_and_defaults_to_local_publish():
    assert _argument_defaults(SENSORS_LAUNCH)[
        'publish_base_lidar_tf'] == 'true'
    assert _argument_defaults(LIVE_LAUNCH)[
        'publish_base_lidar_tf'] == 'true'
    assert _forwards_launch_configuration(
        SENSORS_LAUNCH, 'publish_base_lidar_tf')
    assert _forwards_launch_configuration(
        LIVE_LAUNCH, 'publish_base_lidar_tf')


def test_rslidar_static_tf_can_be_disabled_to_avoid_duplicate_edge():
    source = _source(RSLIDAR_IMPLEMENTATION)

    assert 'publish_base_lidar_tf' in _declared_arguments(
        RSLIDAR_IMPLEMENTATION)
    static_publishers = [
        call for call in _calls(RSLIDAR_IMPLEMENTATION, 'Node')
        if _contains_string(call, 'static_transform_publisher')
    ]
    assert len(static_publishers) == 1
    _assert_if_condition(static_publishers[0], 'publish_base_lidar_tf')
