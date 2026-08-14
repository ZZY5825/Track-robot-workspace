import importlib.util
from collections import Counter
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_ROOT = PACKAGE_ROOT / 'launch'
CONFIG_ROOT = PACKAGE_ROOT / 'config'
RVIZ_ROOT = PACKAGE_ROOT / 'rviz'
HUMAN_FOLLOWING_LIVE = LAUNCH_ROOT / 'human_following_live.launch.py'
SAFE_HUMAN_FOLLOWING = LAUNCH_ROOT / 'safe_human_following.launch.py'
HUMAN_FOLLOWING_RVIZ = RVIZ_ROOT / 'human_following_live.rviz'

FEATURE_INCLUDES = Counter({
    ('track_robot_bringup', 'track_robot_hardware.launch.py'): 1,
    ('track_robot_perception', 'human_tracking_simplified.launch.py'): 1,
    ('track_robot_decision', 'outdoor_follow_decision.launch.py'): 1,
    ('track_robot_control', 'target_follow_controller.launch.py'): 1,
    ('track_robot_safety', 'motion_safety.launch.py'): 1,
})
SOURCE_CONFIG_OVERRIDES = {
    'profile_config': str(CONFIG_ROOT / 'human_following_shadow.yaml'),
    'camera_config_file': str(
        PACKAGE_ROOT.parent.parent / 'track_robot_perception' / 'config' /
        'human_tracking.yaml'),
    'tracklet_config_file': str(
        PACKAGE_ROOT.parent / 'track_robot_lidar_tracking' / 'config' /
        'lidar_tracklets.yaml'),
    'association_config_file': str(
        PACKAGE_ROOT.parent / 'track_robot_lidar_tracking' / 'config' /
        'selected_target_tracker.yaml'),
    'lidar_config_path': str(
        PACKAGE_ROOT.parent / 'track_robot_sensor_bringup' / 'config' /
        'rslidar_track_robot.yaml'),
}


def _source(path):
    assert path.is_file(), 'required launch file is missing: {}'.format(path)
    return path.read_text(encoding='utf-8')


def _load_launch_module(path):
    module_name = 'human_following_contract_{}'.format(
        path.stem.replace('.', '_').replace('-', '_'))
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _launch_description(path):
    return _load_launch_module(path).generate_launch_description()


def _argument_defaults(path):
    context = LaunchContext()
    context.launch_configurations.update(SOURCE_CONFIG_OVERRIDES)
    defaults = {}
    for argument in _launch_description(path).get_launch_arguments():
        argument.execute(context)
        defaults[argument.name] = context.launch_configurations[argument.name]
    return defaults


def _expanded_actions(path, overrides=None):
    context = LaunchContext()
    context.launch_configurations.update(SOURCE_CONFIG_OVERRIDES)
    context.launch_configurations.update(overrides or {})
    description = _launch_description(path)

    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)

    actions = []
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            continue
        if isinstance(entity, OpaqueFunction):
            actions.extend(entity.execute(context) or [])
        else:
            actions.append(entity)
    return context, actions


def _perform(context, value):
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(value))


def _include_identity(include, context):
    source = include.launch_description_source
    location = source.__dict__['_LaunchDescriptionSource__location']
    assert len(location) == 1
    path_join = location[0]
    parts = path_join.__dict__['_PathJoinSubstitution__substitutions']
    package_substitution = parts[0]
    package = _perform(
        context, package_substitution.__dict__['_FindPackage__package'])
    return package, _perform(context, parts[-1])


def _include_arguments(include, context):
    return {
        name: _perform(context, value)
        for name, value in include.launch_arguments
    }


def _node_identity(node, context):
    return (
        _perform(context, node._Node__package),
        _perform(context, node._Node__node_executable),
    )


def _nodes(actions):
    return [action for action in actions if isinstance(action, Node)]


def _includes(actions):
    return [
        action for action in actions
        if isinstance(action, IncludeLaunchDescription)
    ]


def _effective_node_parameters(context, node, node_name):
    effective = {}
    for source in evaluate_parameters(context, node._Node__parameters):
        if isinstance(source, Path):
            document = yaml.safe_load(source.read_text(encoding='utf-8'))
            for scope in ('/**', node_name, '/{}'.format(node_name)):
                if scope in document:
                    effective.update(document[scope]['ros__parameters'])
        else:
            effective.update(source)
    return effective


def _yaml_parameters(path, node_name):
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    return document[node_name]['ros__parameters']


def test_shadow_is_the_fail_closed_default():
    defaults = _argument_defaults(HUMAN_FOLLOWING_LIVE)

    assert defaults['runtime_mode'] == 'shadow'
    assert defaults['motion_confirmed'] == 'false'


def test_invalid_runtime_mode_is_rejected_during_expansion():
    with pytest.raises(RuntimeError, match="runtime_mode must be 'shadow' or 'active'"):
        _expanded_actions(
            HUMAN_FOLLOWING_LIVE, {'runtime_mode': 'unexpected'})


def test_active_without_confirmation_is_rejected_during_expansion():
    with pytest.raises(
            RuntimeError,
            match='active human following requires motion_confirmed:=true'):
        _expanded_actions(HUMAN_FOLLOWING_LIVE, {
            'runtime_mode': 'active',
            'motion_confirmed': 'false',
        })


def test_shadow_expansion_has_full_feature_topology_and_zero_gate_actions():
    context, actions = _expanded_actions(HUMAN_FOLLOWING_LIVE)
    include_counts = Counter(
        _include_identity(include, context) for include in _includes(actions))
    node_counts = Counter(
        _node_identity(node, context) for node in _nodes(actions))

    assert len(actions) == 6
    assert len(_includes(actions)) == 5
    assert len(_nodes(actions)) == 1
    assert include_counts == FEATURE_INCLUDES
    assert node_counts == Counter({
        ('track_robot_decision', 'human_following_supervisor_node'): 1,
    })
    assert ('track_robot_core', 'cmd_vel_gate') not in node_counts


def test_confirmed_active_expansion_constructs_exactly_one_gate_action():
    context, actions = _expanded_actions(HUMAN_FOLLOWING_LIVE, {
        'runtime_mode': 'active',
        'motion_confirmed': 'true',
    })
    node_counts = Counter(
        _node_identity(node, context) for node in _nodes(actions))

    assert len(actions) == 7
    assert len(_includes(actions)) == 5
    assert len(_nodes(actions)) == 2
    assert node_counts == Counter({
        ('track_robot_decision', 'human_following_supervisor_node'): 1,
        ('track_robot_core', 'cmd_vel_gate'): 1,
    })


def test_rviz_is_the_only_optional_visualization_action():
    context, actions = _expanded_actions(
        HUMAN_FOLLOWING_LIVE, {'start_rviz': 'true'})
    node_counts = Counter(
        _node_identity(node, context) for node in _nodes(actions))

    assert len(actions) == 7
    assert node_counts == Counter({
        ('track_robot_decision', 'human_following_supervisor_node'): 1,
        ('rviz2', 'rviz2'): 1,
    })


def test_one_profile_path_is_forwarded_to_all_command_layers():
    profile = CONFIG_ROOT / 'human_following_supervised_test.yaml'
    context, actions = _expanded_actions(HUMAN_FOLLOWING_LIVE, {
        'runtime_mode': 'active',
        'motion_confirmed': 'true',
        'profile_config': str(profile),
    })
    includes = {
        _include_identity(include, context): _include_arguments(include, context)
        for include in _includes(actions)
    }

    for identity in (
            ('track_robot_decision', 'outdoor_follow_decision.launch.py'),
            ('track_robot_control', 'target_follow_controller.launch.py'),
            ('track_robot_safety', 'motion_safety.launch.py')):
        assert includes[identity]['profile_config'] == str(profile)

    nodes = {
        _node_identity(node, context): node for node in _nodes(actions)
    }
    supervisor_parameters = _effective_node_parameters(
        context,
        nodes[('track_robot_decision', 'human_following_supervisor_node')],
        'human_following_supervisor_node',
    )
    gate_parameters = _effective_node_parameters(
        context,
        nodes[('track_robot_core', 'cmd_vel_gate')],
        'cmd_vel_gate',
    )

    assert supervisor_parameters['blocked_disarm_timeout_sec'] == 10.0
    assert supervisor_parameters['runtime_mode'] == 'active'
    assert supervisor_parameters['motion_confirmed'] is True
    assert gate_parameters['max_linear_x'] == 0.05
    assert gate_parameters['max_angular_z'] == 0.15


def test_command_chain_and_controller_motion_ownership_are_exact():
    context, actions = _expanded_actions(HUMAN_FOLLOWING_LIVE, {
        'runtime_mode': 'active',
        'motion_confirmed': 'true',
    })
    includes = {
        _include_identity(include, context): _include_arguments(include, context)
        for include in _includes(actions)
    }
    controller = includes[
        ('track_robot_control', 'target_follow_controller.launch.py')]
    decision = includes[
        ('track_robot_decision', 'outdoor_follow_decision.launch.py')]
    planner = _yaml_parameters(
        PACKAGE_ROOT.parent / 'track_robot_safety' / 'config' /
        'local_trajectory_planner.yaml',
        'local_trajectory_planner_node',
    )
    safety = _yaml_parameters(
        PACKAGE_ROOT.parent / 'track_robot_safety' / 'config' /
        'motion_safety_supervisor.yaml',
        'motion_safety_supervisor_node',
    )
    gate = next(
        node for node in _nodes(actions)
        if _node_identity(node, context) == ('track_robot_core', 'cmd_vel_gate'))
    gate_parameters = _effective_node_parameters(
        context, gate, 'cmd_vel_gate')

    assert decision['command_topic'] == '/follow/cmd_vel_safe'
    assert controller['enable_cmd_vel'] == 'false'
    assert controller['planned_cmd_vel_topic'] == '/follow/cmd_vel_planned'
    assert planner['desired_cmd_topic'] == '/follow/cmd_vel_planned'
    assert planner['output_cmd_topic'] == '/follow/cmd_vel_avoiding'
    assert safety['planned_cmd_topic'] == '/follow/cmd_vel_avoiding'
    assert safety['safe_cmd_topic'] == '/follow/cmd_vel_safe'
    assert gate_parameters['input_topic'] == '/follow/cmd_vel_safe'
    assert gate_parameters['output_topic'] == '/cmd_vel'


def test_launch_composes_no_semantic_search_or_navigation_feature_actions():
    context, actions = _expanded_actions(HUMAN_FOLLOWING_LIVE)
    identities = list(
        _include_identity(include, context) for include in _includes(actions))
    identities.extend(
        _node_identity(node, context) for node in _nodes(actions))

    assert all('semantic' not in package and 'navigation' not in package
               for package, _ in identities)
    assert 'semantic_search_phase' not in _source(HUMAN_FOLLOWING_LIVE)


def test_compatibility_wrapper_delegates_feature_only_and_defaults_closed():
    defaults = _argument_defaults(SAFE_HUMAN_FOLLOWING)
    assert defaults['runtime_mode'] == 'shadow'
    assert defaults['motion_confirmed'] == 'false'
    assert defaults['start_cmd_vel_gate'] == 'false'

    context, actions = _expanded_actions(SAFE_HUMAN_FOLLOWING)
    assert len(actions) == 1
    assert len(_includes(actions)) == 1
    assert len(_nodes(actions)) == 0
    include = _includes(actions)[0]
    assert _include_identity(include, context) == (
        'track_robot_bringup', 'human_following_live.launch.py')
    arguments = _include_arguments(include, context)
    assert arguments['runtime_mode'] == 'shadow'
    assert arguments['motion_confirmed'] == 'false'
    assert arguments['start_description'] == 'false'
    assert arguments['start_camera'] == 'false'
    assert arguments['start_lidar'] == 'false'
    assert arguments['start_base'] == 'false'
    assert arguments['start_imu'] == 'false'
    assert arguments['start_rviz'] == 'false'


def test_compatibility_wrapper_rejects_legacy_gate_enable_request():
    with pytest.raises(
            RuntimeError,
            match='start_cmd_vel_gate cannot enable motion'):
        _expanded_actions(
            SAFE_HUMAN_FOLLOWING, {'start_cmd_vel_gate': 'true'})


def test_rviz_config_names_all_human_following_evidence_layers():
    config = yaml.safe_load(HUMAN_FOLLOWING_RVIZ.read_text(encoding='utf-8'))
    manager = config['Visualization Manager']
    displays = {display['Name']: display for display in manager['Displays']}

    assert manager['Global Options']['Fixed Frame'] == 'base_link'
    expected_topics = {
        'Camera / Fusion Target Markers': '/human_tracking/fused_target_marker',
        'Generic LiDAR Tracklets': '/human_tracking/lidar_tracklet_markers',
        'Observed Selected Physical Tracklet':
            '/human_tracking/selected_tracklet_marker',
        'Filtered Logical Target': '/human_tracking/selected_target_marker',
        'Follow Decision': '/follow/decision_markers',
        'Planned Command - Controller Direction / Speed':
            '/follow/controller_markers',
        'Avoidance Trajectories': '/follow/avoidance_trajectory_markers',
        'Safe Command - Safety Envelope':
            '/safety/collision_envelope_markers',
        'Authorization Session': '/human_following/supervisor_markers',
    }
    assert expected_topics.items() <= {
        (name, display['Topic']['Value'])
        for name, display in displays.items()
        if 'Topic' in display
    }
