from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    output_image_topic = LaunchConfiguration('output_image_topic')
    output_text_topic = LaunchConfiguration('output_text_topic')
    model_config = LaunchConfiguration('model_config')
    score_threshold = LaunchConfiguration('score_threshold')
    device = LaunchConfiguration('device')
    publish_annotated_image = LaunchConfiguration('publish_annotated_image')
    publish_text = LaunchConfiguration('publish_text')
    run_every_n_frames = LaunchConfiguration('run_every_n_frames')
    resize_width = LaunchConfiguration('resize_width')

    mask_rcnn_node = Node(
        package='track_robot_perception',
        executable='zed_mask_rcnn_node',
        name='zed_mask_rcnn_node',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'output_image_topic': output_image_topic,
            'output_text_topic': output_text_topic,
            'model_config': model_config,
            'score_threshold': score_threshold,
            'device': device,
            'publish_annotated_image': publish_annotated_image,
            'publish_text': publish_text,
            'run_every_n_frames': run_every_n_frames,
            'resize_width': resize_width,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('output_image_topic', default_value='/mask_rcnn/annotated_image'),
        DeclareLaunchArgument('output_text_topic', default_value='/mask_rcnn/detections_text'),
        DeclareLaunchArgument(
            'model_config',
            default_value='COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_1x.yaml'),
        DeclareLaunchArgument('score_threshold', default_value='0.5'),
        DeclareLaunchArgument('device', default_value='auto'),
        DeclareLaunchArgument('publish_annotated_image', default_value='true'),
        DeclareLaunchArgument('publish_text', default_value='true'),
        DeclareLaunchArgument('run_every_n_frames', default_value='10'),
        DeclareLaunchArgument('resize_width', default_value='640'),
        mask_rcnn_node,
    ])
