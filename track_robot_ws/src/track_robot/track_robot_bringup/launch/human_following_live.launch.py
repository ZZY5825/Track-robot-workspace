from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package),
            'launch',
            launch_file,
        ])),
        launch_arguments=arguments.items(),
    )


def _boolean_launch_value(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value in ('1', 'true', 'yes', 'on'):
        return True
    if value in ('0', 'false', 'no', 'off'):
        return False
    raise RuntimeError('{} must be true or false'.format(name))


def _launch_setup(context):
    runtime_mode = LaunchConfiguration('runtime_mode').perform(context)
    motion_confirmed = _boolean_launch_value(context, 'motion_confirmed')
    start_rviz = _boolean_launch_value(context, 'start_rviz')

    if runtime_mode not in ('shadow', 'active'):
        raise RuntimeError("runtime_mode must be 'shadow' or 'active'")
    if runtime_mode == 'active' and not motion_confirmed:
        raise RuntimeError(
            'active human following requires motion_confirmed:=true')

    hardware = _include(
        'track_robot_bringup', 'track_robot_hardware.launch.py', {
            'start_description': LaunchConfiguration('start_description'),
            'start_camera': LaunchConfiguration('start_camera'),
            'start_lidar': LaunchConfiguration('start_lidar'),
            'start_base': LaunchConfiguration('start_base'),
            'start_imu': LaunchConfiguration('start_imu'),
            'base_frame': LaunchConfiguration('base_frame'),
            'camera_depth_mode': LaunchConfiguration('camera_depth_mode'),
            'configure_network': LaunchConfiguration('configure_network'),
            'network_interface': LaunchConfiguration('network_interface'),
            'host_ip': LaunchConfiguration('host_ip'),
            'host_cidr': LaunchConfiguration('host_cidr'),
            'driver_start_delay': LaunchConfiguration('driver_start_delay'),
            'publish_base_lidar_tf': LaunchConfiguration(
                'publish_base_lidar_tf'),
            'lidar_config_path': LaunchConfiguration('lidar_config_path'),
            'extrinsic_mode': LaunchConfiguration('extrinsic_mode'),
            'extrinsic_file': LaunchConfiguration('extrinsic_file'),
            'allow_degraded': LaunchConfiguration('allow_degraded'),
        })

    perception = _include(
        'track_robot_perception', 'human_tracking_simplified.launch.py', {
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'lidar_qos_reliability': LaunchConfiguration(
                'lidar_qos_reliability'),
            'base_frame': LaunchConfiguration('base_frame'),
            'tracking_frame': LaunchConfiguration('tracking_frame'),
            'map_frame': LaunchConfiguration('map_frame'),
            'tracker_backend': LaunchConfiguration('tracker_backend'),
            'camera_config_file': LaunchConfiguration('camera_config_file'),
            'tracklet_config_file': LaunchConfiguration(
                'tracklet_config_file'),
            'association_config_file': LaunchConfiguration(
                'association_config_file'),
            'model_path': LaunchConfiguration('model_path'),
            'resize_width': LaunchConfiguration('resize_width'),
        })

    decision = _include(
        'track_robot_decision', 'outdoor_follow_decision.launch.py', {
            'image_topic': LaunchConfiguration('image_topic'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'command_topic': '/follow/cmd_vel_safe',
            'require_health': LaunchConfiguration('require_decision_health'),
            'require_avoidance_feedback': 'true',
            'require_safety_feedback': 'true',
            'profile_config': LaunchConfiguration('profile_config'),
        })

    controller = _include(
        'track_robot_control', 'target_follow_controller.launch.py', {
            'decision_topic': '/follow/decision',
            'planned_cmd_vel_topic': '/follow/cmd_vel_planned',
            'marker_frame': LaunchConfiguration('base_frame'),
            'enable_cmd_vel': 'false',
            'profile_config': LaunchConfiguration('profile_config'),
        })

    safety = _include(
        'track_robot_safety', 'motion_safety.launch.py', {
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'base_frame': LaunchConfiguration('base_frame'),
            'lidar_qos_reliability': LaunchConfiguration(
                'lidar_qos_reliability'),
            'allow_latest_tf_fallback': LaunchConfiguration(
                'allow_latest_tf_fallback'),
            'enable_avoidance': LaunchConfiguration('enable_avoidance'),
            'require_bunker_status': LaunchConfiguration(
                'require_bunker_status'),
            'require_rc_state': LaunchConfiguration('require_rc_state'),
            'profile_config': LaunchConfiguration('profile_config'),
        })

    supervisor = Node(
        package='track_robot_decision',
        executable='human_following_supervisor_node',
        name='human_following_supervisor_node',
        output='screen',
        parameters=[
            LaunchConfiguration('profile_config'),
            {
                'runtime_mode': runtime_mode,
                'motion_confirmed': motion_confirmed,
            },
        ],
    )

    actions = [
        hardware,
        perception,
        decision,
        controller,
        safety,
        supervisor,
    ]

    if runtime_mode == 'active' and motion_confirmed:
        actions.append(Node(
            package='track_robot_core',
            executable='cmd_vel_gate',
            name='cmd_vel_gate',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('track_robot_bringup'),
                    'config',
                    'cmd_vel_gate_follow.yaml',
                ]),
                LaunchConfiguration('profile_config'),
                {
                    'input_topic': '/follow/cmd_vel_safe',
                    'output_topic': '/cmd_vel',
                },
            ],
        ))

    if start_rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='human_following_rviz',
            output='screen',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('track_robot_bringup'),
                    'rviz',
                    'human_following_live.rviz',
                ]),
            ],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('runtime_mode', default_value='shadow'),
        DeclareLaunchArgument('motion_confirmed', default_value='false'),
        DeclareLaunchArgument(
            'profile_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_bringup'),
                'config',
                'human_following_shadow.yaml',
            ]),
        ),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('start_description', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/zed/zed_node/left/camera_info'),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='zed_left_camera_optical_frame'),
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument(
            'lidar_qos_reliability', default_value='reliable'),
        DeclareLaunchArgument(
            'allow_latest_tf_fallback', default_value='false'),
        DeclareLaunchArgument(
            'tracking_frame', default_value='track_robot_center'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/zed/zed_node/imu/data'),
        DeclareLaunchArgument('odometry_topic', default_value='/odom'),
        DeclareLaunchArgument('tracker_backend', default_value='bytetrack'),
        DeclareLaunchArgument(
            'camera_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_perception'),
                'config',
                'human_tracking.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'tracklet_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_lidar_tracking'),
                'config',
                'lidar_tracklets.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'association_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_lidar_tracking'),
                'config',
                'selected_target_tracker.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=(
                '/home/track-robot/track_robot_ws/models/human_tracking/'
                'yolov8n-pose.pt')),
        DeclareLaunchArgument('resize_width', default_value='960'),
        DeclareLaunchArgument('camera_depth_mode', default_value='NONE'),
        DeclareLaunchArgument('configure_network', default_value='true'),
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('host_cidr', default_value='24'),
        DeclareLaunchArgument('driver_start_delay', default_value='1.0'),
        DeclareLaunchArgument(
            'publish_base_lidar_tf', default_value='false'),
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
        DeclareLaunchArgument('enable_avoidance', default_value='true'),
        DeclareLaunchArgument(
            'require_bunker_status', default_value='true'),
        DeclareLaunchArgument('require_rc_state', default_value='true'),
        DeclareLaunchArgument(
            'require_decision_health', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
