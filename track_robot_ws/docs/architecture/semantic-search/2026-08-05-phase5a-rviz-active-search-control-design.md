# Phase 5A RViz Active Search Control Design

**Date:** 2026-08-05

**Status:** Approved for implementation

## 1. Goal

Extend the existing semantic-search RViz panel with one operator control that
starts and stops Phase 5A bounded active search. The control is labelled
**Start Finding** while idle and **Stop Finding** while a search is active.

A single start click must submit the text currently shown in the panel, wait
for the active-search manager to create a correlated rotation intent, and then
authorize that rotation once. The operator must not need a second terminal or
a separate authorization click.

## 2. Existing boundaries

The feature reuses the interfaces already implemented by Phase 5A:

- action: `/semantic_search/search_for_object`
  (`track_robot_interfaces/action/SearchForObject`);
- rotation authorization: `/semantic_search/active_search/authorize_rotation`
  (`std_srvs/srv/Trigger`);
- explicit search stop: `/semantic_search/active_search/cancel`
  (`std_srvs/srv/Trigger`).

The panel remains an operator client. It does not publish `Twist`,
`SearchMotionIntent`, Nav2 goals, or `/cmd_vel`. Phase 5A remains rotation-only;
Phase 4B approach continues to require the separate **Start Approach** control.
RC override, E-stop, the motion-safety supervisor, and the velocity gate remain
authoritative.

## 3. Chosen architecture

Use a feedback-driven ROS 2 action client inside the existing RViz plugin.
Do not add a wrapper node or a new public service.

The start flow is:

1. Normalize and validate the English text in the query field.
2. Send one `SearchForObject` goal with a 60-second timeout,
   `allow_rotation=true`, and `maximum_rotation_angle=1.5708` radians.
3. Change the button to **Stop Finding** as soon as the goal is pending.
4. Adopt the authoritative `query_id` from action feedback as query version
   one, without publishing a duplicate query message.
5. Wait until feedback reports `WAITING_FOR_AUTHORIZATION`.
6. Call `authorize_rotation` exactly once for that action goal.
7. Keep displaying action feedback until the goal reaches a terminal result.
8. Return the button to **Start Finding**.

Authorization must not be called when the goal is merely accepted. At that
time the motion adapter may not yet have a pending intent, which would produce
the known `no_pending_rotation_intent` race.

The stop flow is:

1. Request cancellation of the active action goal.
2. Call `/semantic_search/active_search/cancel` as a bounded explicit stop.
3. Keep the button disabled while cancellation is pending.
4. Restore the idle state after the action result or a bounded local failure.

## 4. UI and state model

Add one button and one status row:

- button: **Start Finding** / **Stop Finding**;
- status label: **Active search**.

The local UI state is one of:

- `IDLE`;
- `GOAL_PENDING`;
- `SEARCHING`;
- `AUTHORIZATION_PENDING`;
- `AUTHORIZED`;
- `CANCEL_PENDING`.

Only `IDLE` may send a new goal. Any active state makes the same button a stop
control, preventing duplicate concurrent searches. A pure C++ state helper
owns these transitions and the one-shot authorization decision; ROS callbacks
only translate action/service events into state-machine events.

Representative status text includes:

- `sending search goal`;
- `observing`;
- `waiting for rotation authorization`;
- `authorizing rotation`;
- `rotating`;
- `target confirmed: object <global_id>`;
- `not found`;
- `timed out`;
- `cancelled`;
- the bounded rejection or service error reason.

All ROS callbacks queue widget changes onto the Qt thread. No callback blocks
the RViz UI waiting for an action server, result, or service.

## 5. Query ownership and correlation

The Phase 5A manager, not the panel, allocates the query ID and publishes the
canonical `/semantic_search/query` message. Therefore **Start Finding** must
not invoke the existing **New Query** publisher first.

When the first valid action feedback supplies its non-zero query ID, the panel
adopts `{query_text, query_id, query_version=1}` in `QuerySession`. This lets
the existing region and diagnostic displays accept only Phase 5A output for
the current search. Adopting an external query never republishes it and never
changes the ID assigned by the manager.

If a late feedback or result belongs to an older local search generation, it
is ignored. This prevents a cancelled search from overwriting the next
search's UI.

## 6. Failure handling

- Empty or oversized text is rejected locally and no action goal is sent.
- If the action server is unavailable, the robot remains stationary and the
  button returns to idle.
- A rejected action goal returns the panel to idle with the rejection shown.
- Rotation authorization is attempted only once and only after the exact
  feedback state is observed.
- Authorization rejection terminates the local request by cancelling both the
  action and the active-search motion adapter; it is never retried in a loop.
- Stop remains safe when only one of action cancellation or the explicit
  cancel service is available.
- Unknown feedback reasons are displayed but never interpreted as permission
  to rotate.
- Panel destruction does not dereference stale Qt widgets from late callbacks;
  callbacks are associated with the current search generation.

## 7. Configuration and compatibility

Add configurable RViz properties for the three interface names while keeping
these defaults:

- `search_action=/semantic_search/search_for_object`;
- `authorize_rotation_service=/semantic_search/active_search/authorize_rotation`;
- `cancel_search_service=/semantic_search/active_search/cancel`.

The goal timeout and maximum angle remain fixed Phase 5A operator-policy
values in this first version: 60 seconds and 90 degrees. They are not exposed
as casual UI controls.

The package adds the existing ROS 2 Foxy dependency `rclcpp_action`; it adds no
downloaded runtime dependency and changes no ROS message or action layout.

## 8. Verification

Pure tests cover:

- idle-to-start and active-to-stop button behavior;
- rejection of duplicate starts;
- one-shot authorization only for `WAITING_FOR_AUTHORIZATION`;
- query-ID adoption without publication;
- cancellation and terminal reset;
- stale-generation callback rejection.

Plugin contract tests cover action/service names, `rclcpp_action` dependency,
the absence of velocity APIs, and preservation of existing approach controls.

Runtime acceptance uses the Phase 5A rotation-supervised launch and verifies:

1. one click starts passive observation;
2. a blind target causes one authorized bounded rotation sequence;
3. the button shows **Stop Finding** during the task;
4. pressing it stops the task and rotation;
5. finding a target reports the returned global object ID;
6. no-target search ends within 60 seconds;
7. **Start Approach** remains a separate explicit operator action;
8. all executable velocity still passes through the existing safety chain.

## 9. Out of scope

- automatic Phase 4B approach after target confirmation;
- translation during active search;
- changes to semantic thresholds, DINOv3, YOLO-World, memory association, or
  Nav2 behavior;
- browser/Foxglove control;
- persistent search history;
- automatic retry after a safety or authorization rejection.
