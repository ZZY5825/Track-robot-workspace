from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_parameters(config_path):
    config = yaml.safe_load(Path(config_path).read_text(encoding='utf-8'))
    if not isinstance(config, dict):
        raise RuntimeError('Phase 1 config must contain a ROS parameter map')
    for node_key in ('semantic_search_perception',
                     '/semantic_search_perception', '/**'):
        node_config = config.get(node_key)
        if not isinstance(node_config, dict):
            continue
        parameters = node_config.get('ros__parameters')
        if isinstance(parameters, dict):
            return dict(parameters)
    raise RuntimeError(
        'Phase 1 config has no semantic_search_perception parameters')


def _launch_setup(context):
    parameters = _load_parameters(
        LaunchConfiguration('config_file').perform(context))
    parameters.update({
        'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'), value_type=bool),
        'image_topic': ParameterValue(
            LaunchConfiguration('image_topic'), value_type=str),
        'adapter_implementation': ParameterValue(
            LaunchConfiguration('adapter_implementation'), value_type=str),
        'model_name': ParameterValue(
            LaunchConfiguration('model_name'), value_type=str),
        'runtime_path': ParameterValue(
            LaunchConfiguration('runtime_path'), value_type=str),
        'checkpoint_path': ParameterValue(
            LaunchConfiguration('checkpoint_path'), value_type=str),
        'device': ParameterValue(
            LaunchConfiguration('device'), value_type=str),
    })

    perception = Node(
        package='track_robot_semantic_search',
        executable='semantic_search_perception',
        name='semantic_search_perception',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_perception')),
        # Foxy gives a node-specific YAML block precedence over wildcard
        # launch overrides. Merge them before creating the node so every key
        # has exactly one source.
        parameters=[parameters],
    )
    return [perception]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_perception', default_value='false'),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_semantic_search'),
                'config',
                'semantic_search_phase1.yaml',
            ])),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument(
            'adapter_implementation', default_value='openai_clip'),
        DeclareLaunchArgument('model_name', default_value='ViT-B/32'),
        DeclareLaunchArgument('runtime_path', default_value=''),
        DeclareLaunchArgument('checkpoint_path', default_value=''),
        DeclareLaunchArgument('device', default_value='auto'),
        OpaqueFunction(function=_launch_setup),
    ])
