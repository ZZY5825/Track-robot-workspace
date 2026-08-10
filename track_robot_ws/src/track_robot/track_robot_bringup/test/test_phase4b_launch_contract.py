import ast
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch' / 'semantic_search_phase4b.launch.py'
RVIZ = PACKAGE / 'rviz' / 'semantic_search_phase4b.rviz'
GUIDE = (
    PACKAGE.parents[2] / 'docs' / 'guides' / 'semantic-search' /
    'phase4b-nav2-supervised-test.md'
)


def _source(path):
    assert path.is_file(), 'required Phase 4B file is missing: {}'.format(path)
    return path.read_text(encoding='utf-8')


def test_phase4b_entrypoint_is_shadow_and_stationary_by_default():
    source = _source(LAUNCH)

    assert "default_value='SEMANTIC_SHADOW'" in source
    assert "'enable_semantic_execution', default_value='false'" in source
    assert "'physical_recovery_enabled', default_value='false'" in source
    assert "'start_base', default_value='false'" in source
    assert "'start_imu': 'false'" in source
    assert "'start_obstacle_map': 'false'" in source
    assert "'extrinsic_mode', default_value='robot_description'" in source


def test_phase4b_composes_phase4a_nav2_platform_and_rviz():
    source = _source(LAUNCH)

    assert 'semantic_search_phase4a.launch.py' in source
    assert 'semantic_search_platform.launch.py' in source
    assert 'phase4b_navigation.launch.py' in source
    assert "executable='semantic_search_live_overlay'" in source
    assert 'semantic_search_phase4b.rviz' in source
    assert "'runtime_mode': LaunchConfiguration('runtime_mode')" in source
    assert "'enable_semantic_execution':" in source
    assert "LaunchConfiguration('enable_semantic_execution')" in source
    assert "'physical_recovery_enabled':" in source
    assert "LaunchConfiguration('physical_recovery_enabled')" in source
    assert "'start_rviz': 'false'" in source
    assert "'start_phase4b_rviz', default_value='true'" in source
    assert (
        "condition=IfCondition(LaunchConfiguration('start_phase4b_rviz'))"
        in source
    )
    assert ast.parse(source)


def test_phase4b_enables_dino_identity_evidence_and_starts_platform_first():
    source = _source(LAUNCH)

    assert "'dino_enabled': LaunchConfiguration('dino_enabled')" in source
    assert "'dino_enabled', default_value='true'" in source
    assert 'platform,\n        phase4a,' in source
    assert "'base_frame': 'robot_bottom'" in source


def test_phase4b_rviz_exposes_semantic_and_nav2_evidence():
    source = _source(RVIZ)

    for topic in (
            '/semantic_search/phase4/markers',
            '/semantic_search/phase4/planned_path',
            '/semantic_navigation/shadow_path',
            '/global_costmap/costmap',
            '/local_costmap/costmap',
            '/plan',
            '/rslidar_points'):
        assert topic in source
    assert 'Fixed Frame: robot_bottom' in source
    assert 'Class: rviz_default_plugins/TF\n      Enabled: false' in source
    assert 'nav2_rviz_plugins/GoalTool' in source
    assert 'Class: rviz_default_plugins/RobotModel' in source
    assert 'Value: /robot_description' in source


def test_phase4b_operator_guide_matches_static_mission_and_dino_defaults():
    guide = _source(GUIDE)

    assert '.worktrees/main-integration/track_robot_ws' in guide
    assert 'run phase4b --no-dino' in guide
    assert '冻结当前 `odom` 中的接近位姿' in guide
    assert '`inflation_radius=0.60 m`' in guide
    assert '`cost_scaling_factor=12.0`' in guide
    assert '`0.88 x 0.80 m`' in guide
