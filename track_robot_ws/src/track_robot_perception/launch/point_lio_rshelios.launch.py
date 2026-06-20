from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    rslidar_config_file = LaunchConfiguration('rslidar_config_file')
    imu_lio_adapter_config_file = LaunchConfiguration('imu_lio_adapter_config_file')
    configure_network = LaunchConfiguration('configure_network')
    network_interface = LaunchConfiguration('network_interface')
    host_ip = LaunchConfiguration('host_ip')
    host_cidr = LaunchConfiguration('host_cidr')

    network_setup = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            [
                'sudo -n ip link set ',
                network_interface,
                ' up && sudo -n ip addr replace ',
                host_ip,
                '/',
                host_cidr,
                ' dev ',
                network_interface,
            ],
        ],
        output='screen',
        condition=IfCondition(configure_network),
    )

    rslidar_node = Node(
        namespace='rslidar_sdk',
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        output='screen',
        parameters=[{'config_path': rslidar_config_file}],
        condition=IfCondition(LaunchConfiguration('start_lidar')),
    )

    delayed_lidar_start = TimerAction(
        period=1.0,
        actions=[rslidar_node],
    )

    imu_node = Node(
        package='track_robot_perception',
        executable='phidget_spatial_imu_node',
        name='phidget_spatial_imu',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('track_robot_perception'),
            'config',
            'phidget_imu.yaml',
        ])],
        condition=IfCondition(LaunchConfiguration('start_imu')),
    )

    adapter_node = Node(
        package='track_robot_perception',
        executable='rslidar_point_lio_adapter_node',
        name='rslidar_point_lio_adapter',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('lidar_topic'),
            'output_topic': LaunchConfiguration('adapted_lidar_topic'),
            'output_frame_id': LaunchConfiguration('lidar_frame'),
            'timestamp_field': LaunchConfiguration('timestamp_field'),
            'output_time_field': 'time',
            'keep_original_timestamp': True,
        }],
        condition=IfCondition(LaunchConfiguration('start_adapter')),
    )

    imu_lio_adapter_node = Node(
        package='track_robot_perception',
        executable='imu_lio_adapter_node',
        name='imu_lio_adapter',
        output='screen',
        parameters=[imu_lio_adapter_config_file, {
            'input_topic': LaunchConfiguration('raw_imu_topic'),
            'output_topic': LaunchConfiguration('lio_imu_topic'),
            'output_frame_id': LaunchConfiguration('lio_imu_frame'),
            'time_offset_sec': LaunchConfiguration('imu_time_offset_sec'),
        }],
        condition=IfCondition(LaunchConfiguration('start_imu_lio_adapter')),
    )

    point_lio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='point_lio',
        output='screen',
        parameters=[config_file, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('start_point_lio')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_perception'),
                'config',
                'point_lio_rshelios.yaml',
            ]),
        ),
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument(
            'adapted_lidar_topic',
            default_value='/rslidar_points_point_lio'),
        DeclareLaunchArgument('lidar_frame', default_value='rslidar'),
        DeclareLaunchArgument('timestamp_field', default_value='timestamp'),
        DeclareLaunchArgument('raw_imu_topic', default_value='/imu/data_raw'),
        DeclareLaunchArgument('lio_imu_topic', default_value='/imu/data_lio'),
        DeclareLaunchArgument('lio_imu_frame', default_value='rslidar'),
        DeclareLaunchArgument('imu_time_offset_sec', default_value='0.0'),
        DeclareLaunchArgument('start_adapter', default_value='false'),
        DeclareLaunchArgument('start_imu_lio_adapter', default_value='true'),
        DeclareLaunchArgument('start_point_lio', default_value='true'),
        DeclareLaunchArgument('start_imu', default_value='false'),
        DeclareLaunchArgument('start_lidar', default_value='false'),
        DeclareLaunchArgument('configure_network', default_value='false'),
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('host_cidr', default_value='24'),
        DeclareLaunchArgument(
            'rslidar_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_bringup'),
                'config',
                'rslidar_track_robot.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'imu_lio_adapter_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_perception'),
                'config',
                'imu_lio_adapter.yaml',
            ]),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        network_setup,
        delayed_lidar_start,
        imu_node,
        adapter_node,
        imu_lio_adapter_node,
        point_lio_node,
    ])
