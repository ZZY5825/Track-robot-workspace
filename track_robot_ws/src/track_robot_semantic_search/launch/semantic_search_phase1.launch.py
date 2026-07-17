from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_perception = LaunchConfiguration('start_perception')
    config_file = LaunchConfiguration('config_file')
    image_topic = LaunchConfiguration('image_topic')
    adapter_implementation = LaunchConfiguration('adapter_implementation')
    model_name = LaunchConfiguration('model_name')
    runtime_path = LaunchConfiguration('runtime_path')
    checkpoint_path = LaunchConfiguration('checkpoint_path')
    device = LaunchConfiguration('device')

    perception = Node(
        package='track_robot_semantic_search',
        executable='semantic_search_perception',
        name='semantic_search_perception',
        output='screen',
        condition=IfCondition(start_perception),
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'image_topic': image_topic,
                'adapter_implementation': adapter_implementation,
                'model_name': model_name,
                'runtime_path': runtime_path,
                'checkpoint_path': checkpoint_path,
                'device': device,
            },
        ],
    )

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
        perception,
    ])
