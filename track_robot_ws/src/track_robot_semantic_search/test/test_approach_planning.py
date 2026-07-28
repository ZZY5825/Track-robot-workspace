import math

from track_robot_semantic_search.approach_planning import (
    GridMap,
    Phase4Planner,
    PlannerConfig,
    PlanningContext,
    TargetCandidate,
)


NOW_NS = 10_000_000_000


def make_grid(fill=0):
    width = 80
    height = 80
    return GridMap(
        frame_id='base_link',
        stamp_ns=NOW_NS - 50_000_000,
        resolution=0.1,
        width=width,
        height=height,
        origin_x=-4.0,
        origin_y=-4.0,
        data=tuple([fill] * (width * height)),
    )


def replace_cells(grid, cells, value=100):
    data = list(grid.data)
    for cell_x, cell_y in cells:
        data[cell_y * grid.width + cell_x] = value
    return GridMap(
        frame_id=grid.frame_id,
        stamp_ns=grid.stamp_ns,
        resolution=grid.resolution,
        width=grid.width,
        height=grid.height,
        origin_x=grid.origin_x,
        origin_y=grid.origin_y,
        data=tuple(data),
    )


def target(**overrides):
    values = {
        'memory_epoch_id': 11,
        'global_object_id': 42,
        'localization_epoch_id': 7,
        'query_id': 1234,
        'query_version': 2,
        'position_frame_id': 'base_link',
        'position_valid': True,
        'x': 2.0,
        'y': 0.0,
        'z': 0.4,
        'lifecycle_state': 'confirmed',
        'task_relevance': 0.82,
        'uncertainty': 0.18,
        'last_seen_ns': NOW_NS - 100_000_000,
    }
    values.update(overrides)
    return TargetCandidate(**values)


def context(candidates=None, grid=None, **overrides):
    values = {
        'now_ns': NOW_NS,
        'localization_epoch_id': 7,
        'localization_healthy': True,
        'robot_x': 0.0,
        'robot_y': 0.0,
        'target_candidates': tuple(
            [target()] if candidates is None else candidates),
        'grid': make_grid() if grid is None else grid,
    }
    values.update(overrides)
    return PlanningContext(**values)


def path_cells(grid, path):
    for pose in path:
        cell = grid.world_to_cell(pose.x, pose.y)
        assert cell is not None
        yield cell


def test_open_map_produces_standoff_goal_and_collision_free_path():
    grid = make_grid()
    target_cell = grid.world_to_cell(2.0, 0.0)
    grid = replace_cells(grid, [target_cell])

    result = Phase4Planner(PlannerConfig()).plan(context(grid=grid))

    assert result.status == 'PASS'
    assert result.reason == 'planned'
    assert result.target.global_object_id == 42
    assert result.target.memory_epoch_id == 11
    assert result.target.localization_epoch_id == 7
    assert result.target.query_id == 1234
    assert result.target.query_version == 2
    assert len(result.approach_candidates) >= 8
    assert result.selected_goal is not None
    assert result.path
    distance = math.hypot(
        result.selected_goal.x - result.target.x,
        result.selected_goal.y - result.target.y)
    assert abs(distance - 0.8) <= 0.11
    facing = math.atan2(
        result.target.y - result.selected_goal.y,
        result.target.x - result.selected_goal.x)
    assert abs(result.selected_goal.yaw - facing) < 1e-6
    assert all(grid.is_traversable(*cell) for cell in path_cells(grid, result.path))


def test_no_target_is_explicit_abstention():
    result = Phase4Planner(PlannerConfig()).plan(context(candidates=()))

    assert result.status == 'FAIL'
    assert result.reason == 'no_target'
    assert result.selected_goal is None
    assert result.path == ()


def test_close_scores_reject_ambiguous_target():
    candidates = (
        target(global_object_id=42, task_relevance=0.82),
        target(global_object_id=43, task_relevance=0.78, x=1.5, y=0.5),
    )

    result = Phase4Planner(PlannerConfig(
        minimum_target_margin=0.08)).plan(context(candidates=candidates))

    assert result.status == 'FAIL'
    assert result.reason == 'ambiguous_target'


def test_lost_or_stale_target_is_rejected():
    planner = Phase4Planner(PlannerConfig(maximum_target_age_sec=0.5))

    lost = planner.plan(context(candidates=(target(lifecycle_state='lost'),)))
    stale = planner.plan(context(candidates=(
        target(last_seen_ns=NOW_NS - 600_000_000),)))

    assert lost.reason == 'target_lost'
    assert stale.reason == 'target_lost'


def test_invalid_position_and_frame_are_rejected():
    planner = Phase4Planner(PlannerConfig())

    invalid = planner.plan(context(candidates=(target(position_valid=False),)))
    wrong_frame = planner.plan(context(candidates=(
        target(position_frame_id='odom'),)))

    assert invalid.reason == 'invalid_position'
    assert wrong_frame.reason == 'frame_mismatch'


def test_blocked_map_returns_no_path():
    grid = make_grid()
    wall = [(40, y) for y in range(grid.height)]
    blocked = replace_cells(grid, wall)

    result = Phase4Planner(PlannerConfig()).plan(context(
        candidates=(target(x=2.0, y=0.0),), grid=blocked))

    assert result.status == 'FAIL'
    assert result.reason == 'blocked_path'
    assert result.selected_goal is None
    assert result.path == ()


def test_stale_map_is_rejected_before_planning():
    grid = make_grid()
    stale_grid = GridMap(
        frame_id=grid.frame_id,
        stamp_ns=NOW_NS - 2_000_000_000,
        resolution=grid.resolution,
        width=grid.width,
        height=grid.height,
        origin_x=grid.origin_x,
        origin_y=grid.origin_y,
        data=grid.data,
    )

    result = Phase4Planner(PlannerConfig(
        maximum_map_age_sec=0.5)).plan(context(grid=stale_grid))

    assert result.status == 'FAIL'
    assert result.reason == 'stale_map'


def test_localization_reset_rejects_old_target_reference():
    result = Phase4Planner(PlannerConfig()).plan(context(
        localization_epoch_id=8,
        candidates=(target(localization_epoch_id=7),)))

    assert result.status == 'FAIL'
    assert result.reason == 'localization_reset'
    assert result.selected_goal is None
    assert result.path == ()


def test_unhealthy_localization_and_excess_uncertainty_fail_closed():
    planner = Phase4Planner(PlannerConfig(maximum_target_uncertainty=0.5))

    unhealthy = planner.plan(context(localization_healthy=False))
    uncertain = planner.plan(context(candidates=(target(uncertainty=0.8),)))

    assert unhealthy.reason == 'localization_unhealthy'
    assert uncertain.reason == 'target_uncertainty_high'
