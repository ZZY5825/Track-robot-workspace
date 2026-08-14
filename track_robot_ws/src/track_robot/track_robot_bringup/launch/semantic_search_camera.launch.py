import math
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def _as_bool(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _measured_extrinsic(path):
    try:
        config = yaml.safe_load(path.read_text(encoding='utf-8'))
        calibration_id = config['calibration_id']
        parent_frame = config['parent_frame']
        child_frame = config['child_frame']
        translation = config['translation']
        rotation = config['rotation_rpy']
        if (
                not isinstance(calibration_id, str)
                or not calibration_id.strip()
                or calibration_id == 'replace_with_measured_calibration_id'):
            raise ValueError('calibration_id is not measured')
        if parent_frame != 'base_link' or child_frame != 'zed_camera_link':
            raise ValueError('camera extrinsic frames are invalid')
        transform = {
            'parent_frame': parent_frame,
            'child_frame': child_frame,
            'x': float(translation['x']),
            'y': float(translation['y']),
            'z': float(translation['z']),
            'roll': float(rotation['roll']),
            'pitch': float(rotation['pitch']),
            'yaw': float(rotation['yaw']),
        }
        if not all(math.isfinite(transform[value]) for value in (
                'x', 'y', 'z', 'roll', 'pitch', 'yaw')):
            raise ValueError('camera extrinsic values must be finite')
        return transform
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
        raise RuntimeError(
            'measured camera extrinsic file is invalid: {}'.format(path)
        ) from error


def _launch_extrinsic(context):
    mode = LaunchConfiguration('extrinsic_mode').perform(context)
    allow_degraded = _as_bool(
        LaunchConfiguration('allow_degraded').perform(context))
    extrinsic_file = Path(
        LaunchConfiguration('extrinsic_file').perform(context))

    if mode not in ('none', 'prototype', 'measured', 'robot_description'):
        raise RuntimeError(
            'unknown camera extrinsic mode {!r}; expected none, prototype, '
            'measured, or robot_description'.format(mode))
    if mode in ('none', 'robot_description'):
        return []
    if mode == 'prototype' and not allow_degraded:
        raise RuntimeError(
            'prototype camera extrinsic requires allow_degraded:=true')
    if mode == 'measured' and not extrinsic_file.is_file():
        raise RuntimeError(
            'measured camera extrinsic file does not exist: {}'.format(
                extrinsic_file))
    if mode == 'measured':
        transform = _measured_extrinsic(extrinsic_file)
    else:
        # Degraded prototype estimate only: base->rslidar is z=0.70 m and
        # zed->rslidar is x=-0.27 m, z=0.08 m with zero rotation, giving
        # base->zed x=0.27 m, z=0.62 m. It is not a measured calibration.
        transform = {
            'parent_frame': 'base_link',
            'child_frame': 'zed_camera_link',
            'x': 0.27,
            'y': 0.0,
            'z': 0.62,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
        }

    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_zed_camera_tf',
            arguments=[
                str(transform['x']),
                str(transform['y']),
                str(transform['z']),
                str(transform['yaw']),
                str(transform['pitch']),
                str(transform['roll']),
                transform['parent_frame'],
                transform['child_frame'],
            ],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('extrinsic_mode'),
                "' in ['measured', 'prototype']",
            ])),
        ),
    ]


def generate_launch_description():
    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('zed_wrapper'),
            'launch',
            'zed_camera.launch.py',
        ])),
        launch_arguments={
            'camera_model': 'zed2i',
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'publish_urdf': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )
    camera_only_zed = GroupAction(actions=[
        SetParameter(
            name='depth.depth_mode',
            value=LaunchConfiguration('depth_mode')),
        SetParameter(name='depth.depth_stabilization', value=0),
        SetParameter(
            name='pos_tracking.pos_tracking_enabled',
            value=False,
        ),
        zed_camera,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('extrinsic_mode', default_value='none'),
        DeclareLaunchArgument('extrinsic_file', default_value=''),
        DeclareLaunchArgument('allow_degraded', default_value='false'),
        DeclareLaunchArgument('depth_mode', default_value='NONE'),
        OpaqueFunction(function=_launch_extrinsic),
        camera_only_zed,
    ])
