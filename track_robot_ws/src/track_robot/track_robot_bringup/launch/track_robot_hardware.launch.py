from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('bunker_pro2'),
            'launch',
            'description.launch.py',
        ])),
        condition=IfCondition(LaunchConfiguration('start_description')),
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('track_robot_bringup'),
            'launch',
            'semantic_search_camera.launch.py',
        ])),
        launch_arguments={
            'start_camera': LaunchConfiguration('start_camera'),
            'extrinsic_mode': PythonExpression([
                "'", 'robot_description', "' if '",
                LaunchConfiguration('start_description'),
                "'.lower() in ('1', 'true', 'yes', 'on') else '",
                LaunchConfiguration('extrinsic_mode'),
                "'",
            ]),
            'extrinsic_file': LaunchConfiguration('extrinsic_file'),
            'allow_degraded': LaunchConfiguration('allow_degraded'),
            'depth_mode': LaunchConfiguration('camera_depth_mode'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('track_robot_bringup'),
            'launch',
            'rslidar_with_tf.launch.py',
        ])),
        launch_arguments={
            'configure_network': LaunchConfiguration('configure_network'),
            'network_interface': LaunchConfiguration('network_interface'),
            'host_ip': LaunchConfiguration('host_ip'),
            'host_cidr': LaunchConfiguration('host_cidr'),
            'driver_start_delay': LaunchConfiguration('driver_start_delay'),
            'config_path': LaunchConfiguration('lidar_config_path'),
            'publish_base_lidar_tf': PythonExpression([
                "'", 'false', "' if '",
                LaunchConfiguration('start_description'),
                "'.lower() in ('1', 'true', 'yes', 'on') else '",
                LaunchConfiguration('publish_base_lidar_tf'),
                "'",
            ]),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_lidar')),
    )
    bunker_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('bunker_base'),
            'launch',
            'bunker_base.launch.py',
        ])),
        launch_arguments={
            'base_frame': LaunchConfiguration('base_frame'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_base')),
    )
    phidget_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('track_robot_perception'),
            'launch',
            'phidget_imu.launch.py',
        ])),
        launch_arguments={
            'config_path': PathJoinSubstitution([
                FindPackageShare('track_robot_perception'),
                'config',
                'phidget_imu.yaml',
            ]),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_imu')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_description', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('camera_depth_mode', default_value='NONE'),
        DeclareLaunchArgument('configure_network', default_value='true'),
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('host_cidr', default_value='24'),
        DeclareLaunchArgument('driver_start_delay', default_value='1.0'),
        DeclareLaunchArgument(
            'publish_base_lidar_tf',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'lidar_config_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_sensor_bringup'),
                'config',
                'rslidar_track_robot.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'extrinsic_mode', default_value='robot_description'),
        DeclareLaunchArgument('extrinsic_file', default_value=''),
        DeclareLaunchArgument('allow_degraded', default_value='false'),
        description,
        camera,
        lidar,
        bunker_base,
        phidget_imu,
    ])
