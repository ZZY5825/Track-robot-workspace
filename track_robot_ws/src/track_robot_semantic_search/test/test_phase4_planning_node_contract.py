import ast
from pathlib import Path

import yaml

from track_robot_semantic_search.approach_planner_node import _output_header


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (
    PACKAGE / 'track_robot_semantic_search' / 'approach_planner_node.py')
SETUP = PACKAGE / 'setup.py'
LAUNCH = PACKAGE / 'launch' / 'semantic_search_phase4.launch.py'
CONFIG = PACKAGE / 'config' / 'semantic_search_phase4.yaml'
RVIZ = (
    PACKAGE.parent / 'track_robot' / 'track_robot_bringup'
    / 'rviz' / 'semantic_search_phase4.rviz')


def source(path):
    return path.read_text(encoding='utf-8')


def test_phase4_node_has_only_planning_and_visualization_interfaces():
    tree = ast.parse(source(SOURCE))
    publishers = []
    subscriptions = []
    for call in (
            node for node in ast.walk(tree) if isinstance(node, ast.Call)):
        function = call.func
        if isinstance(function, ast.Attribute) and call.args:
            if function.attr == 'create_publisher':
                publishers.append(ast.dump(call.args[1]))
            if function.attr == 'create_subscription':
                subscriptions.append(ast.dump(call.args[1]))

    joined_publishers = '\n'.join(publishers)
    joined_subscriptions = '\n'.join(subscriptions)
    assert 'approach_candidates_topic' in joined_publishers
    assert 'selected_goal_topic' in joined_publishers
    assert 'path_topic' in joined_publishers
    assert 'markers_topic' in joined_publishers
    assert 'diagnostics_topic' in joined_publishers
    assert 'selected_target_topic' in joined_subscriptions
    assert 'costmap_topic' in joined_subscriptions
    assert 'localization_topic' in joined_subscriptions

    text = source(SOURCE)
    for forbidden in (
            'geometry_msgs.msg.Twist',
            'cmd_vel',
            'ActionClient',
            'create_client',
            '/safety/arm'):
        assert forbidden not in text


def test_phase4_entry_point_launch_config_and_rviz_are_installed():
    setup = source(SETUP)
    assert (
        'semantic_search_phase4_planner = '
        in setup)
    assert (
        'track_robot_semantic_search.approach_planner_node:main'
        in setup)
    assert LAUNCH.is_file()
    assert CONFIG.is_file()
    assert RVIZ.is_file()


def test_phase4_defaults_are_planning_only_and_fail_closed():
    config = yaml.safe_load(source(CONFIG))['phase4_approach_planner'][
        'ros__parameters']
    assert config['selected_target_topic'] == '/semantic_memory/best_candidate'
    assert config['costmap_topic'] == '/safety/local_obstacle_grid'
    assert config['localization_topic'] == '/semantic_memory/localization_state'
    assert config['minimum_target_relevance'] >= 0.5
    assert config['maximum_target_uncertainty'] <= 0.5
    assert config['maximum_map_age_sec'] <= 0.5
    assert config['maximum_target_age_sec'] <= 1.0
    assert config['unknown_is_obstacle'] is True
    assert config['planning_only'] is True


def test_rviz_contains_target_candidates_goal_costmap_and_path():
    rviz = source(RVIZ)
    for topic in (
            '/semantic_search/phase4/markers',
            '/semantic_search/phase4/approach_candidates',
            '/semantic_search/phase4/selected_goal',
            '/safety/local_obstacle_grid',
            '/semantic_search/phase4/planned_path'):
        assert topic in rviz


def test_output_header_does_not_reuse_or_mutate_costmap_stamp():
    header = _output_header('base_link', 12_345_678_901)

    assert header.frame_id == 'base_link'
    assert header.stamp.sec == 12
    assert header.stamp.nanosec == 345_678_901
