from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pose_node = Node(
        package='track_robot_perception',
        executable='zed_mask_rcnn_node',
        name='zed_pose_rcnn_node',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'output_image_topic': LaunchConfiguration('output_image_topic'),
            'output_text_topic': LaunchConfiguration('output_text_topic'),
            'model_config': LaunchConfiguration('model_config'),
            'score_threshold': LaunchConfiguration('score_threshold'),
            'device': LaunchConfiguration('device'),
            'publish_annotated_image': LaunchConfiguration('publish_annotated_image'),
            'publish_text': LaunchConfiguration('publish_text'),
            'run_every_n_frames': LaunchConfiguration('run_every_n_frames'),
            'resize_width': LaunchConfiguration('resize_width'),
            'max_detections': LaunchConfiguration('max_detections'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('output_image_topic', default_value='/pose_rcnn/annotated_image'),
        DeclareLaunchArgument('output_text_topic', default_value='/pose_rcnn/detections_text'),
        DeclareLaunchArgument(
            'model_config',
            default_value='COCO-Keypoints/keypoint_rcnn_R_50_FPN_1x.yaml'),
        DeclareLaunchArgument('score_threshold', default_value='0.7'),
        DeclareLaunchArgument('device', default_value='auto'),
        DeclareLaunchArgument('publish_annotated_image', default_value='true'),
        DeclareLaunchArgument('publish_text', default_value='true'),
        DeclareLaunchArgument('run_every_n_frames', default_value='10'),
        DeclareLaunchArgument('resize_width', default_value='640'),
        DeclareLaunchArgument('max_detections', default_value='10'),
        pose_node,
    ])
