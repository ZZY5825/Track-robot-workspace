from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_parameters(config_path):
    config = yaml.safe_load(Path(config_path).read_text(encoding='utf-8'))
    if not isinstance(config, dict):
        raise RuntimeError('YOLO-World config must contain a ROS parameter map')
    for node_key in (
            'semantic_search_yolo_world_perception',
            '/semantic_search_yolo_world_perception',
            '/**'):
        node_config = config.get(node_key)
        if not isinstance(node_config, dict):
            continue
        parameters = node_config.get('ros__parameters')
        if isinstance(parameters, dict):
            return dict(parameters)
    raise RuntimeError('YOLO-World config has no node parameters')


def _launch_setup(context):
    parameters = _load_parameters(
        LaunchConfiguration('config_file').perform(context))
    for name, value_type in (
            ('use_sim_time', bool),
            ('image_topic', str),
            ('runtime_path', str),
            ('clip_runtime_path', str),
            ('world_checkpoint', str),
            ('clip_checkpoint', str),
            ('dino_local_repo', str),
            ('dino_checkpoint', str),
            ('device', int),
            ('dino_enabled', bool)):
        parameters[name] = ParameterValue(
            LaunchConfiguration(name), value_type=value_type)
    return [Node(
        package='track_robot_semantic_search',
        executable='semantic_search_yolo_world_perception',
        name='semantic_search_yolo_world_perception',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_perception')),
        additional_env={
            'LD_PRELOAD': '/lib/aarch64-linux-gnu/libgomp.so.1',
        },
        parameters=[parameters],
    )]


def _workspace_path(*parts):
    return PathJoinSubstitution([
        EnvironmentVariable('HOME'),
        'track_robot_ws',
        *parts,
    ])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_perception', default_value='false'),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_semantic_search'),
                'config',
                'semantic_search_yolo_world.yaml',
            ])),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument(
            'runtime_path',
            default_value=_workspace_path(
                'models', 'r0c_runtime', 'python')),
        DeclareLaunchArgument(
            'clip_runtime_path',
            default_value=_workspace_path(
                'models', 'phase1_runtime', 'python')),
        DeclareLaunchArgument(
            'world_checkpoint',
            default_value=_workspace_path(
                'models', 'r0c', 'yolov8s-worldv2.pt')),
        DeclareLaunchArgument(
            'clip_checkpoint',
            default_value=_workspace_path(
                'models', 'phase1', 'ViT-B-32.pt')),
        DeclareLaunchArgument(
            'dino_local_repo',
            default_value=_workspace_path(
                'src', 'track_robot_core', 'third_party', 'dinov3_py38')),
        DeclareLaunchArgument(
            'dino_checkpoint',
            default_value=_workspace_path(
                'models', 'dinov3_vits16plus_pretrain_lvd1689m.pth')),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument('dino_enabled', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
