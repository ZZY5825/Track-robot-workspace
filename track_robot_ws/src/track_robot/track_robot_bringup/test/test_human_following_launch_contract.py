import importlib.util
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[3]
LAUNCH_ROOT = PACKAGE_ROOT / 'launch'
CONFIG_ROOT = PACKAGE_ROOT / 'config'
RVIZ_ROOT = PACKAGE_ROOT / 'rviz'
HUMAN_FOLLOWING_LIVE = LAUNCH_ROOT / 'human_following_live.launch.py'
SAFE_HUMAN_FOLLOWING = LAUNCH_ROOT / 'safe_human_following.launch.py'
HUMAN_FOLLOWING_RVIZ = RVIZ_ROOT / 'human_following_live.rviz'

FEATURE_NODES = Counter({
    ('track_robot_perception', 'human_image_tracker_node'): 1,
    ('track_robot_perception', 'gesture_trigger_node'): 1,
    ('track_robot_perception', 'camera_target_lock_node'): 1,
    ('track_robot_lidar_tracking', 'lidar_tracklet_manager_node'): 1,
    (
        'track_robot_lidar_tracking',
        'selected_human_target_tracker_node',
    ): 1,
    ('track_robot_decision', 'perception_health_monitor_node'): 1,
    ('track_robot_decision', 'follow_behavior_tree_node'): 1,
    ('track_robot_control', 'target_follow_controller_node'): 1,
    ('track_robot_safety', 'local_obstacle_map_node'): 1,
    ('track_robot_safety', 'local_trajectory_planner_node'): 1,
    ('track_robot_safety', 'motion_safety_supervisor_node'): 1,
    ('track_robot_decision', 'human_following_supervisor_node'): 1,
})
FEATURE_ONLY_OVERRIDES = {
    'start_description': 'false',
    'start_camera': 'false',
    'start_lidar': 'false',
    'start_base': 'false',
    'start_imu': 'false',
}
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


def _source_package_roots():
    roots = {}
    package_manifests = []
    for search_root in (
            REPOSITORY_ROOT / 'src', REPOSITORY_ROOT / 'track_robot_ws' / 'src'):
        package_manifests.extend(search_root.rglob('package.xml'))
    for package_xml in sorted(
            package_manifests, key=lambda path: (len(path.parts), str(path))):
        package_name = ElementTree.parse(package_xml).getroot().findtext('name')
        assert package_name
        if package_name in roots:
            assert roots[package_name] in package_xml.parents
            continue
        roots[package_name] = package_xml.parent
    return roots


SOURCE_PACKAGE_ROOTS = _source_package_roots()


@dataclass
class _ExpandedNode:
    action: Node
    context: LaunchContext
    launch_path: Path

    @property
    def identity(self):
        return _node_identity(self.action, self.context)


@dataclass
class _ExpandedLaunchGraph:
    includes: list
    nodes: list


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


def _copy_context(context):
    copied = LaunchContext()
    copied.launch_configurations.update(context.launch_configurations)
    return copied


def _recursive_launch_graph(path, overrides=None):
    graph = _ExpandedLaunchGraph(includes=[], nodes=[])

    def expand_entities(entities, context, launch_path):
        for entity in entities:
            condition = getattr(entity, 'condition', None)
            if condition is not None and not condition.evaluate(context):
                continue

            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
            elif isinstance(entity, OpaqueFunction):
                expand_entities(
                    entity.execute(context) or [], context, launch_path)
            elif isinstance(entity, GroupAction):
                group_context = (
                    _copy_context(context)
                    if entity.__dict__['_GroupAction__scoped']
                    else context
                )
                group_context.launch_configurations.update({
                    name: _perform(context, value)
                    for name, value in entity.__dict__[
                        '_GroupAction__launch_configurations'].items()
                })
                expand_entities(
                    entity.__dict__['_GroupAction__actions'],
                    group_context,
                    launch_path,
                )
            elif isinstance(entity, IncludeLaunchDescription):
                identity = _include_identity(entity, context)
                graph.includes.append(identity)
                package, launch_file = identity
                package_root = SOURCE_PACKAGE_ROOTS.get(package)
                if package_root is None:
                    continue
                child_path = package_root / 'launch' / launch_file
                if child_path.is_file():
                    walk_launch(
                        child_path, _include_arguments(entity, context))
            elif isinstance(entity, Node):
                graph.nodes.append(_ExpandedNode(
                    action=entity,
                    context=_copy_context(context),
                    launch_path=launch_path,
                ))

    def walk_launch(launch_path, launch_overrides):
        context = LaunchContext()
        context.launch_configurations.update(SOURCE_CONFIG_OVERRIDES)
        context.launch_configurations.update(launch_overrides)
        description = _launch_description(launch_path)
        expand_entities(description.entities, context, launch_path)

    root_overrides = dict(SOURCE_CONFIG_OVERRIDES)
    root_overrides.update(overrides or {})
    walk_launch(path, root_overrides)
    return graph


def _feature_graph(path, overrides=None):
    effective_overrides = {}
    if path == HUMAN_FOLLOWING_LIVE:
        effective_overrides.update(FEATURE_ONLY_OVERRIDES)
    effective_overrides.update(overrides or {})
    return _recursive_launch_graph(path, effective_overrides)


def _graph_node(graph, identity):
    matching = [node for node in graph.nodes if node.identity == identity]
    assert len(matching) == 1, (
        'expected one {} node, found {}'.format(identity, len(matching)))
    return matching[0]


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
            document = yaml.safe_load(
                _source_parameter_path(source).read_text(encoding='utf-8'))
            for scope in ('/**', node_name, '/{}'.format(node_name)):
                if scope in document:
                    effective.update(document[scope]['ros__parameters'])
        else:
            effective.update(source)
    return effective


def _source_parameter_path(path):
    parts = path.parts
    for package, package_root in SOURCE_PACKAGE_ROOTS.items():
        marker = ('share', package)
        for index in range(len(parts) - 1):
            if parts[index:index + 2] == marker:
                candidate = package_root.joinpath(*parts[index + 2:])
                if candidate.is_file():
                    return candidate
    return path


def _graph_node_parameters(graph, identity, node_name):
    expanded = _graph_node(graph, identity)
    return _effective_node_parameters(
        expanded.context, expanded.action, node_name)


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


@pytest.mark.parametrize(
    'entrypoint', [HUMAN_FOLLOWING_LIVE, SAFE_HUMAN_FOLLOWING])
def test_shadow_expansion_has_full_feature_topology_and_zero_gate_actions(
        entrypoint):
    graph = _feature_graph(entrypoint)
    node_counts = Counter(node.identity for node in graph.nodes)

    assert node_counts == FEATURE_NODES


@pytest.mark.parametrize(
    'entrypoint', [HUMAN_FOLLOWING_LIVE, SAFE_HUMAN_FOLLOWING])
def test_confirmed_active_expansion_constructs_exactly_one_gate_action(
        entrypoint):
    graph = _feature_graph(entrypoint, {
        'runtime_mode': 'active',
        'motion_confirmed': 'true',
    })
    node_counts = Counter(node.identity for node in graph.nodes)

    assert node_counts == FEATURE_NODES + Counter({
        ('track_robot_core', 'cmd_vel_gate'): 1})


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


@pytest.mark.parametrize(
    'entrypoint', [HUMAN_FOLLOWING_LIVE, SAFE_HUMAN_FOLLOWING])
def test_one_profile_path_is_forwarded_to_all_command_layers(entrypoint):
    profile = CONFIG_ROOT / 'human_following_supervised_test.yaml'
    graph = _feature_graph(entrypoint, {
        'runtime_mode': 'active',
        'motion_confirmed': 'true',
        'profile_config': str(profile),
    })
    decision = _graph_node_parameters(
        graph,
        ('track_robot_decision', 'follow_behavior_tree_node'),
        'follow_behavior_tree_node',
    )
    controller = _graph_node_parameters(
        graph,
        ('track_robot_control', 'target_follow_controller_node'),
        'target_follow_controller_node',
    )
    planner = _graph_node_parameters(
        graph,
        ('track_robot_safety', 'local_trajectory_planner_node'),
        'local_trajectory_planner_node',
    )
    safety = _graph_node_parameters(
        graph,
        ('track_robot_safety', 'motion_safety_supervisor_node'),
        'motion_safety_supervisor_node',
    )
    supervisor = _graph_node_parameters(
        graph,
        ('track_robot_decision', 'human_following_supervisor_node'),
        'human_following_supervisor_node',
    )
    gate = _graph_node_parameters(
        graph,
        ('track_robot_core', 'cmd_vel_gate'),
        'cmd_vel_gate',
    )

    assert (
        decision['confirmed_max_linear'],
        decision['confirmed_max_angular'],
    ) == (0.05, 0.15)
    for parameters in (controller, planner, safety, gate):
        assert (
            parameters['max_linear_x'],
            parameters['max_angular_z'],
        ) == (0.05, 0.15)
    assert controller['allow_lidar_only_forward_motion'] is False
    assert safety['require_odom'] is True
    assert safety['odom_timeout_sec'] == 0.25
    assert supervisor['blocked_disarm_timeout_sec'] == 10.0
    assert supervisor['runtime_mode'] == 'active'
    assert supervisor['motion_confirmed'] is True


@pytest.mark.parametrize(
    'entrypoint', [HUMAN_FOLLOWING_LIVE, SAFE_HUMAN_FOLLOWING])
def test_command_chain_and_controller_motion_ownership_are_exact(entrypoint):
    graph = _feature_graph(entrypoint, {
        'runtime_mode': 'active',
        'motion_confirmed': 'true',
    })
    health = _graph_node_parameters(
        graph,
        ('track_robot_decision', 'perception_health_monitor_node'),
        'perception_health_monitor_node',
    )
    controller = _graph_node_parameters(
        graph,
        ('track_robot_control', 'target_follow_controller_node'),
        'target_follow_controller_node',
    )
    planner = _graph_node_parameters(
        graph,
        ('track_robot_safety', 'local_trajectory_planner_node'),
        'local_trajectory_planner_node',
    )
    safety = _graph_node_parameters(
        graph,
        ('track_robot_safety', 'motion_safety_supervisor_node'),
        'motion_safety_supervisor_node',
    )
    gate = _graph_node_parameters(
        graph, ('track_robot_core', 'cmd_vel_gate'), 'cmd_vel_gate')

    assert health['command_topic'] == '/follow/cmd_vel_safe'
    assert controller['decision_topic'] == '/follow/decision'
    assert controller['enable_cmd_vel'] is False
    assert controller['planned_cmd_vel_topic'] == '/follow/cmd_vel_planned'
    assert planner['desired_cmd_topic'] == '/follow/cmd_vel_planned'
    assert planner['output_cmd_topic'] == '/follow/cmd_vel_avoiding'
    assert safety['planned_cmd_topic'] == '/follow/cmd_vel_avoiding'
    assert safety['safe_cmd_topic'] == '/follow/cmd_vel_safe'
    assert gate['input_topic'] == '/follow/cmd_vel_safe'
    assert gate['output_topic'] == '/cmd_vel'


@pytest.mark.parametrize(
    'entrypoint', [HUMAN_FOLLOWING_LIVE, SAFE_HUMAN_FOLLOWING])
@pytest.mark.parametrize(
    'mode', [
        {'runtime_mode': 'shadow', 'motion_confirmed': 'false'},
        {'runtime_mode': 'active', 'motion_confirmed': 'true'},
    ])
def test_launch_composes_no_semantic_search_or_navigation_feature_actions(
        entrypoint, mode):
    graph = _feature_graph(entrypoint, mode)
    identities = graph.includes + [node.identity for node in graph.nodes]

    for identity in identities:
        flattened = ' '.join(identity).lower()
        assert 'semantic_search' not in flattened
        assert 'navigation' not in flattened


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


def test_compatibility_wrapper_rejects_active_without_confirmation():
    with pytest.raises(
            RuntimeError,
            match='active human following requires motion_confirmed:=true'):
        _feature_graph(SAFE_HUMAN_FOLLOWING, {
            'runtime_mode': 'active',
            'motion_confirmed': 'false',
        })


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
