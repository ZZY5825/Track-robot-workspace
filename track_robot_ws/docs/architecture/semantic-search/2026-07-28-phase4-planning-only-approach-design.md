# Phase 4 Planning-Only Standoff Approach

Date: 2026-07-28

## Scope

Phase 4 converts one fail-closed Phase 3 selected semantic object into a local
standoff goal and a collision-free path for operator inspection. It is a
planning and visualization feature only. It owns no navigation action client,
controller, `Twist`, or `cmd_vel` publisher.

The runtime consumes:

- `/semantic_memory/best_candidate`
- `/semantic_memory/localization_state`
- `/safety/local_obstacle_grid`

It publishes:

- `/semantic_search/phase4/approach_candidates`
- `/semantic_search/phase4/selected_goal`
- `/semantic_search/phase4/planned_path`
- `/semantic_search/phase4/markers`
- `/semantic_search/phase4/diagnostics`

## Fail-closed contract

Planning is rejected before graph search when any of these conditions is true:

- no selected target or more than one selected target;
- target lifecycle is lost;
- target position is invalid or its frame differs from the costmap frame;
- query, memory epoch, global object, or localization epoch references are
  absent or inconsistent;
- target relevance is below the configured threshold, ambiguity margin is too
  small, or uncertainty is too high;
- target, map, or localization state is stale or unhealthy;
- the localization epoch changes after the selected target was produced.

For a valid target, the planner samples 16 poses on a 0.8 m standoff circle,
rejects occupied and unknown cells, and runs bounded 8-connected A* on the
local occupancy grid. The chosen pose faces the target. A failed search emits
empty path and goal products plus a bounded diagnostic reason.

## Safety boundary

`planning_only` is required to be `true`; node construction fails otherwise.
The launch file starts only the planner and optional RViz. It does not start
the safety supervisor, local trajectory executor, base driver, or any motion
interface. An empty plan is the only valid result for unsafe or incomplete
inputs.

## Validation boundary

`semantic_search_phase4_validate` is the deterministic functional contract.
It covers success, no target, ambiguous target, target lost, invalid position,
blocked path, stale map, and localization reset. The live collector
`semantic_search_phase04_live_validate` separately checks the real ROS graph,
cross-phase IDs, frames, timestamps, path output, planner latency, and absence
of `cmd_vel` publishers.

The deterministic contract proves planner logic. It does not prove that live
Phase 0–3 inputs are calibrated, spatially valid, or able to produce a selected
target. A live Phase 4 PASS requires those upstream conditions.
