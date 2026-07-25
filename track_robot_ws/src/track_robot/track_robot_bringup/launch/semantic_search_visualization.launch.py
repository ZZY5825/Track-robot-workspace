from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_visualization(context):
    stage = LaunchConfiguration('stage').perform(context)
    configs = {
        'phase1': 'semantic_search_phase1.rviz',
        'phase2': 'semantic_search_phase2.rviz',
    }
    if stage not in configs:
        raise RuntimeError(
            'visualization stage must be phase1 or phase2, got {!r}'.format(
                stage))

    overlay = Node(
        package='track_robot_semantic_search',
        executable='semantic_search_live_overlay',
        name='semantic_search_live_overlay',
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='semantic_search_rviz',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('track_robot_bringup'),
                'rviz',
                configs[stage],
            ]),
        ],
    )
    close_with_rviz = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz,
            on_exit=[
                EmitEvent(event=Shutdown(
                    reason='semantic-search RViz window closed')),
            ],
        ))
    return [overlay, rviz, close_with_rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stage', default_value='phase1'),
        OpaqueFunction(function=_launch_visualization),
    ])

