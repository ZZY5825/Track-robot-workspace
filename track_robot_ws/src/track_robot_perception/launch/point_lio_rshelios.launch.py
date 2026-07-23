from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    rslidar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('track_robot_sensor_bringup'),
            'launch',
            'rslidar_with_tf.launch.py',
        ])),
        launch_arguments={
            'configure_network': configure_network,
            'network_interface': network_interface,
            'host_ip': host_ip,
            'host_cidr': host_cidr,
            'config_path': rslidar_config_file,
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_lidar')),
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

    body_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='point_lio_body_to_base_link_tf',
        arguments=[
            LaunchConfiguration('body_to_base_x'),
            LaunchConfiguration('body_to_base_y'),
            LaunchConfiguration('body_to_base_z'),
            '0.0',
            '0.0',
            '0.0',
            LaunchConfiguration('point_lio_body_frame'),
            LaunchConfiguration('base_frame'),
        ],
        condition=IfCondition(LaunchConfiguration('publish_body_to_base_link_tf')),
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
        DeclareLaunchArgument('publish_body_to_base_link_tf', default_value='true'),
        DeclareLaunchArgument('point_lio_body_frame', default_value='body'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        # Point-LIO's body frame is the LiDAR/IMU body. The bringup static TF
        # places rslidar 0.7 m above base_link, so this inverse bridge connects
        # camera_init -> body -> base_link -> rslidar for RViz.
        DeclareLaunchArgument('body_to_base_x', default_value='0.0'),
        DeclareLaunchArgument('body_to_base_y', default_value='0.0'),
        DeclareLaunchArgument('body_to_base_z', default_value='-0.7'),
        DeclareLaunchArgument('configure_network', default_value='true'),
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('host_cidr', default_value='24'),
        DeclareLaunchArgument(
            'rslidar_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_sensor_bringup'),
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
        rslidar_bringup,
        imu_node,
        adapter_node,
        imu_lio_adapter_node,
        point_lio_node,
        body_to_base_link_tf,
    ])
