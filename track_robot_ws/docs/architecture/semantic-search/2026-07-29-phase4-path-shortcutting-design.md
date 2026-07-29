# Phase 4 Path Shortcutting

Date: 2026-07-29

## Observed problem

The Phase 4 planner searches an 8-connected occupancy grid and publishes every
cell in the resulting path. On an empty 240 by 240 grid at 0.05 m resolution,
a target at approximately `(3.17, -1.64)` produces 49 poses and six direction
runs even though the start and selected standoff goal have direct line of
sight. The grid path is approximately 6.7 percent longer than that straight
segment. A target directly in front already produces a straight path.

This is a path representation and grid-discretization problem. It is not a
failure of target selection, obstacle mapping, or the existing bounded search.

## Scope

Keep the existing target gates, 16 standoff candidates, bounded multi-goal
search, selected goal, topics, messages, IDs, frames, timestamps, and
planning-only safety boundary unchanged.

Add an optional collision-checked path-shortcutting stage after search and
before conversion to `Pose2D`. Enable it in the Phase 4 and Phase 4A
configuration files. A single parameter must disable it and restore the raw
grid path for rollback.

Do not add spline smoothing, motion execution, Nav2 integration, controller
constraints, or changes to costmap inflation in this change.

## Algorithm

1. Reconstruct the existing raw grid path.
2. Starting at the first path cell, test cells from the goal backwards and
   select the farthest cell with collision-free line of sight.
3. Append that cell as the next waypoint and repeat until reaching the goal.
4. Convert only the retained cells to poses. Orient intermediate poses along
   the following segment and preserve the selected standoff goal yaw at the
   final pose.

Line-of-sight testing uses a supercover grid traversal. Every cell touched by
the segment must pass the same occupied threshold and unknown-cell policy as
the graph search. A segment that passes exactly between blocked diagonal cells
is rejected. This prevents shortcutting across obstacle corners.

The shortcut stage must return the raw path unchanged if it cannot prove a
safe shortcut. It must never convert a failed search into a successful plan.

## Configuration and diagnostics

Add one boolean planner parameter:

- `enable_path_shortcutting`: rollback switch, enabled by the Phase 4 and
  Phase 4A YAML configurations.

Add diagnostic values without changing message schemas:

- raw and published path pose counts;
- published path segment count;
- path length in metres;
- whether shortcutting changed the path.

## Safety and regression gates

- Every supercover cell of every published segment is traversable.
- Unknown and occupied cells remain obstacles under the existing policy.
- Open space with direct line of sight publishes exactly one segment.
- A blocking wall still returns `blocked_path`.
- An obstacle requiring a detour retains the necessary waypoints.
- Disabling the feature reproduces the current raw grid path.
- All existing no-target, ambiguity, lost-target, stale-map, localization
  reset, invalid-position, search-budget, frame, and uncertainty behavior is
  unchanged.
- Existing planner and launch-contract tests pass.
- Deterministic Phase 0-4A regression passes with no ID, frame, timestamp,
  safety, or output-interface regression.
- Planner P95 latency must not increase by more than five percent on the same
  replay or live test. Otherwise the feature is disabled and the change is
  rejected.

## Expected result

For the reproduced empty-grid case, the published path should fall from 49
poses and six direction runs to two poses and one straight segment. Around
obstacles, the planner should retain only collision-required turns. The robot
still receives no executable motion command.
