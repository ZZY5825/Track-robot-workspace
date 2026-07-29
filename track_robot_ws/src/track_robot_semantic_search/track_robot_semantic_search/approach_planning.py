"""Fail-closed, planning-only standoff approach generation."""

from dataclasses import dataclass
import heapq
import math
from typing import List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class GridMap:
    frame_id: str
    stamp_ns: int
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    data: Tuple[int, ...]

    def valid(self) -> bool:
        return (
            bool(self.frame_id)
            and self.stamp_ns >= 0
            and math.isfinite(self.resolution)
            and self.resolution > 0.0
            and self.width > 0
            and self.height > 0
            and len(self.data) == self.width * self.height
            and math.isfinite(self.origin_x)
            and math.isfinite(self.origin_y)
        )

    def world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if not self.valid() or not math.isfinite(x) or not math.isfinite(y):
            return None
        cell_x = int(math.floor((x - self.origin_x) / self.resolution))
        cell_y = int(math.floor((y - self.origin_y) / self.resolution))
        if (
                cell_x < 0 or cell_y < 0
                or cell_x >= self.width or cell_y >= self.height):
            return None
        return cell_x, cell_y

    def cell_to_world(self, cell_x: int, cell_y: int) -> Tuple[float, float]:
        return (
            self.origin_x + (cell_x + 0.5) * self.resolution,
            self.origin_y + (cell_y + 0.5) * self.resolution,
        )

    def is_traversable(
            self, cell_x: int, cell_y: int,
            occupied_threshold: int = 50,
            unknown_is_obstacle: bool = True) -> bool:
        if (
                cell_x < 0 or cell_y < 0
                or cell_x >= self.width or cell_y >= self.height):
            return False
        value = self.data[cell_y * self.width + cell_x]
        if value < 0:
            return not unknown_is_obstacle
        return value < occupied_threshold


@dataclass(frozen=True)
class TargetCandidate:
    memory_epoch_id: int
    global_object_id: int
    localization_epoch_id: int
    query_id: int
    query_version: int
    position_frame_id: str
    position_valid: bool
    x: float
    y: float
    z: float
    lifecycle_state: str
    task_relevance: float
    uncertainty: float
    last_seen_ns: int


@dataclass(frozen=True)
class PlanningContext:
    now_ns: int
    localization_epoch_id: int
    localization_healthy: bool
    robot_x: float
    robot_y: float
    target_candidates: Tuple[TargetCandidate, ...]
    grid: GridMap


@dataclass(frozen=True)
class PlannerConfig:
    standoff_distance: float = 0.8
    candidate_count: int = 16
    minimum_target_relevance: float = 0.5
    minimum_target_margin: float = 0.08
    maximum_target_uncertainty: float = 0.5
    maximum_target_age_sec: float = 0.75
    maximum_map_age_sec: float = 0.5
    maximum_search_expansions: int = 30_000
    occupied_threshold: int = 50
    unknown_is_obstacle: bool = True
    enable_path_shortcutting: bool = False


@dataclass(frozen=True)
class PlanResult:
    status: str
    reason: str
    target: Optional[TargetCandidate] = None
    approach_candidates: Tuple[Pose2D, ...] = ()
    selected_goal: Optional[Pose2D] = None
    path: Tuple[Pose2D, ...] = ()
    search_expansions: int = 0
    search_budget_exhausted: bool = False
    raw_path_pose_count: int = 0
    path_length_m: float = 0.0
    path_shortcut_applied: bool = False


@dataclass(frozen=True)
class _SearchResult:
    goal_index: Optional[int]
    cells: Tuple[Tuple[int, int], ...]
    expansions: int
    budget_exhausted: bool


class Phase4Planner:
    """Generate a local collision-free path without any execution interface."""

    _NEIGHBORS = (
        (-1, -1, math.sqrt(2.0)),
        (0, -1, 1.0),
        (1, -1, math.sqrt(2.0)),
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (-1, 1, math.sqrt(2.0)),
        (0, 1, 1.0),
        (1, 1, math.sqrt(2.0)),
    )

    def __init__(self, config: PlannerConfig):
        self._config = config
        if (
                not math.isfinite(config.standoff_distance)
                or config.standoff_distance <= 0.0
                or config.candidate_count < 4
                or config.candidate_count > 128
                or config.maximum_target_age_sec <= 0.0
                or config.maximum_map_age_sec <= 0.0
                or config.maximum_search_expansions <= 0):
            raise ValueError('invalid Phase 4 planner configuration')

    @staticmethod
    def _fail(reason: str) -> PlanResult:
        return PlanResult(status='FAIL', reason=reason)

    def plan(self, context: PlanningContext) -> PlanResult:
        if not context.grid.valid():
            return self._fail('invalid_map')
        map_age_ns = context.now_ns - context.grid.stamp_ns
        if (
                map_age_ns < 0
                or map_age_ns >
                int(self._config.maximum_map_age_sec * 1_000_000_000)):
            return self._fail('stale_map')
        if not context.localization_healthy:
            return self._fail('localization_unhealthy')
        if not context.target_candidates:
            return self._fail('no_target')

        ordered = sorted(
            context.target_candidates,
            key=lambda item: (-item.task_relevance, item.global_object_id))
        target = ordered[0]
        if (
                len(ordered) > 1
                and target.task_relevance - ordered[1].task_relevance
                < self._config.minimum_target_margin):
            return self._fail('ambiguous_target')
        if target.lifecycle_state != 'confirmed':
            return self._fail('target_lost')
        target_age_ns = context.now_ns - target.last_seen_ns
        if (
                target_age_ns < 0
                or target_age_ns >
                int(self._config.maximum_target_age_sec * 1_000_000_000)):
            return self._fail('target_lost')
        if (
                target.memory_epoch_id <= 0
                or target.global_object_id <= 0
                or target.localization_epoch_id <= 0
                or target.query_id <= 0
                or target.query_version <= 0):
            return self._fail('invalid_target_reference')
        if target.localization_epoch_id != context.localization_epoch_id:
            return self._fail('localization_reset')
        if target.position_frame_id != context.grid.frame_id:
            return self._fail('frame_mismatch')
        if (
                not target.position_valid
                or not all(math.isfinite(value) for value in (
                    target.x, target.y, target.z))):
            return self._fail('invalid_position')
        if (
                not math.isfinite(target.task_relevance)
                or target.task_relevance <
                self._config.minimum_target_relevance):
            return self._fail('target_confidence_low')
        if (
                not math.isfinite(target.uncertainty)
                or target.uncertainty < 0.0
                or target.uncertainty >
                self._config.maximum_target_uncertainty):
            return self._fail('target_uncertainty_high')

        start = context.grid.world_to_cell(context.robot_x, context.robot_y)
        if start is None or not self._traversable(context.grid, start):
            return self._fail('blocked_path')

        candidates = self._approach_candidates(target)
        valid_candidates: List[Pose2D] = []
        valid_goals: List[Tuple[int, int]] = []
        for candidate in candidates:
            goal = context.grid.world_to_cell(candidate.x, candidate.y)
            if goal is None or not self._traversable(context.grid, goal):
                continue
            valid_candidates.append(candidate)
            valid_goals.append(goal)

        search = self._shortest_approach_path(
            context.grid, start, tuple(valid_goals))
        if search.budget_exhausted:
            return PlanResult(
                status='FAIL',
                reason='search_budget_exhausted',
                target=target,
                approach_candidates=tuple(valid_candidates),
                search_expansions=search.expansions,
                search_budget_exhausted=True,
            )
        if search.goal_index is None or not search.cells:
            return PlanResult(
                status='FAIL',
                reason='blocked_path',
                target=target,
                approach_candidates=tuple(valid_candidates),
                search_expansions=search.expansions,
            )
        total_expansions = search.expansions
        published_cells = search.cells
        if self._config.enable_path_shortcutting:
            published_cells = self._shortcut_path(
                context.grid, search.cells)
            if not published_cells:
                remaining_expansions = (
                    self._config.maximum_search_expansions
                    - total_expansions)
                if remaining_expansions <= 0:
                    return PlanResult(
                        status='FAIL',
                        reason='search_budget_exhausted',
                        target=target,
                        approach_candidates=tuple(valid_candidates),
                        search_expansions=total_expansions,
                        search_budget_exhausted=True,
                        raw_path_pose_count=len(search.cells),
                    )
                safe_search = self._shortest_approach_path(
                    context.grid,
                    start,
                    tuple(valid_goals),
                    prevent_corner_cutting=True,
                    maximum_expansions=remaining_expansions,
                )
                total_expansions += safe_search.expansions
                if safe_search.budget_exhausted:
                    return PlanResult(
                        status='FAIL',
                        reason='search_budget_exhausted',
                        target=target,
                        approach_candidates=tuple(valid_candidates),
                        search_expansions=total_expansions,
                        search_budget_exhausted=True,
                        raw_path_pose_count=len(search.cells),
                    )
                if (
                        safe_search.goal_index is None
                        or not safe_search.cells):
                    return PlanResult(
                        status='FAIL',
                        reason='blocked_path',
                        target=target,
                        approach_candidates=tuple(valid_candidates),
                        search_expansions=total_expansions,
                        raw_path_pose_count=len(search.cells),
                    )
                search = safe_search
                published_cells = self._shortcut_path(
                    context.grid, search.cells)
                if not published_cells:
                    return PlanResult(
                        status='FAIL',
                        reason='blocked_path',
                        target=target,
                        approach_candidates=tuple(valid_candidates),
                        search_expansions=total_expansions,
                        raw_path_pose_count=len(search.cells),
                    )
        best = valid_candidates[search.goal_index]
        path = self._path_poses(
            context.grid, published_cells, best.yaw)
        return PlanResult(
            status='PASS',
            reason='planned',
            target=target,
            approach_candidates=tuple(valid_candidates),
            selected_goal=best,
            path=path,
            search_expansions=total_expansions,
            raw_path_pose_count=len(search.cells),
            path_length_m=(
                self._path_cost(published_cells)
                * context.grid.resolution),
            path_shortcut_applied=(
                tuple(published_cells) != tuple(search.cells)),
        )

    def _approach_candidates(
            self, target: TargetCandidate) -> Tuple[Pose2D, ...]:
        output = []
        for index in range(self._config.candidate_count):
            angle = (
                2.0 * math.pi * float(index)
                / float(self._config.candidate_count))
            x = target.x + self._config.standoff_distance * math.cos(angle)
            y = target.y + self._config.standoff_distance * math.sin(angle)
            yaw = math.atan2(target.y - y, target.x - x)
            output.append(Pose2D(x=x, y=y, yaw=yaw))
        return tuple(output)

    def _traversable(
            self, grid: GridMap, cell: Tuple[int, int]) -> bool:
        return grid.is_traversable(
            cell[0], cell[1],
            occupied_threshold=self._config.occupied_threshold,
            unknown_is_obstacle=self._config.unknown_is_obstacle)

    def _shortest_approach_path(
            self, grid: GridMap, start: Tuple[int, int],
            goals: Sequence[Tuple[int, int]],
            prevent_corner_cutting: bool = False,
            maximum_expansions: Optional[int] = None) -> _SearchResult:
        if not goals:
            return _SearchResult(None, (), 0, False)
        expansion_limit = (
            self._config.maximum_search_expansions
            if maximum_expansions is None
            else maximum_expansions)

        goal_indices = {}
        for index, goal in enumerate(goals):
            goal_indices.setdefault(goal, []).append(index)
        frontier = [(0.0, start)]
        came_from = {start: None}
        cost_so_far = {start: 0.0}
        expansions = 0
        selected_goal = None
        selected_goal_index = None
        selected_cost = None
        while frontier:
            if (
                    selected_cost is not None
                    and frontier[0][0] > selected_cost):
                break
            if expansions >= expansion_limit:
                return _SearchResult(None, (), expansions, True)
            current_cost, current = heapq.heappop(frontier)
            if current_cost > cost_so_far.get(current, math.inf):
                continue
            expansions += 1
            if current in goal_indices:
                goal_index = min(goal_indices[current])
                if (
                        selected_goal_index is None
                        or goal_index < selected_goal_index):
                    selected_goal = current
                    selected_goal_index = goal_index
                    selected_cost = current_cost
                continue
            for dx, dy, move_cost in self._NEIGHBORS:
                next_cell = current[0] + dx, current[1] + dy
                if not self._traversable(grid, next_cell):
                    continue
                if (
                        prevent_corner_cutting
                        and dx != 0 and dy != 0
                        and (
                            not self._traversable(
                                grid, (current[0] + dx, current[1]))
                            or not self._traversable(
                                grid, (current[0], current[1] + dy)))):
                    continue
                next_cost = current_cost + move_cost
                if (
                        next_cell in cost_so_far
                        and next_cost >= cost_so_far[next_cell]):
                    continue
                cost_so_far[next_cell] = next_cost
                heapq.heappush(frontier, (next_cost, next_cell))
                came_from[next_cell] = current
        if selected_goal is None or selected_goal_index is None:
            return _SearchResult(None, (), expansions, False)
        cells = []
        current = selected_goal
        while current is not None:
            cells.append(current)
            current = came_from[current]
        cells.reverse()
        return _SearchResult(
            selected_goal_index, tuple(cells), expansions, False)

    @staticmethod
    def _supercover_cells(
            start: Tuple[int, int],
            end: Tuple[int, int]) -> Tuple[Tuple[int, int], ...]:
        x, y = start
        end_x, end_y = end
        delta_x = end_x - x
        delta_y = end_y - y
        count_x = abs(delta_x)
        count_y = abs(delta_y)
        step_x = 0 if delta_x == 0 else (1 if delta_x > 0 else -1)
        step_y = 0 if delta_y == 0 else (1 if delta_y > 0 else -1)
        index_x = 0
        index_y = 0
        output = [(x, y)]

        def append(cell):
            if output[-1] != cell:
                output.append(cell)

        while index_x < count_x or index_y < count_y:
            decision = (
                (1 + 2 * index_x) * count_y
                - (1 + 2 * index_y) * count_x)
            if decision == 0:
                append((x + step_x, y))
                append((x, y + step_y))
                x += step_x
                y += step_y
                index_x += 1
                index_y += 1
                append((x, y))
            elif decision < 0:
                x += step_x
                index_x += 1
                append((x, y))
            else:
                y += step_y
                index_y += 1
                append((x, y))
        return tuple(output)

    def _line_of_sight(
            self, grid: GridMap,
            start: Tuple[int, int],
            end: Tuple[int, int]) -> bool:
        return all(
            self._traversable(grid, cell)
            for cell in self._supercover_cells(start, end))

    def _shortcut_path(
            self, grid: GridMap,
            cells: Sequence[Tuple[int, int]]
            ) -> Tuple[Tuple[int, int], ...]:
        if len(cells) <= 2:
            return tuple(cells)
        output = [cells[0]]
        anchor = 0
        while anchor < len(cells) - 1:
            selected = None
            for candidate in range(len(cells) - 1, anchor, -1):
                if self._line_of_sight(
                        grid, cells[anchor], cells[candidate]):
                    selected = candidate
                    break
            if selected is None:
                return ()
            output.append(cells[selected])
            anchor = selected
        return tuple(output)

    @staticmethod
    def _path_cost(cells: Sequence[Tuple[int, int]]) -> float:
        cost = 0.0
        for left, right in zip(cells, cells[1:]):
            cost += math.hypot(
                right[0] - left[0], right[1] - left[1])
        return cost

    @staticmethod
    def _path_poses(
            grid: GridMap, cells: Sequence[Tuple[int, int]],
            final_yaw: float) -> Tuple[Pose2D, ...]:
        output = []
        for index, cell in enumerate(cells):
            x, y = grid.cell_to_world(*cell)
            if index + 1 < len(cells):
                next_x, next_y = grid.cell_to_world(*cells[index + 1])
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                yaw = final_yaw
            output.append(Pose2D(x=x, y=y, yaw=yaw))
        return tuple(output)
