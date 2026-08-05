# Track Robot Semantic Search RViz Plugins

This ROS 2 Foxy package provides the
`track_robot_semantic_search_rviz_plugins/SemanticSearchPanel` RViz panel.
It is intentionally separate from the Python model runtime so Qt and RViz
dependencies do not enter semantic inference.

The panel publishes canonical language-query JSON to `/semantic_search/query`
for its existing query flow. It displays correlated perception diagnostics,
semantic-region counts, active semantic-memory objects, and the zero-or-one
fail-closed best candidate.

For the Phase 5A **Start Finding** / **Stop Finding** path, the panel uses
only these ROS clients:

- `SearchForObject` at `/semantic_search/search_for_object`;
- `Trigger` authorization at
  `/semantic_search/active_search/authorize_rotation`; and
- `Trigger` cancellation at `/semantic_search/active_search/cancel`.

It publishes no velocity and does not start Phase 4B approach automatically.
**Start Finding** / **Stop Finding** controls only a bounded, rotation-only
search. The existing Phase 4B **Start Approach** and **Cancel & Disarm**
clients remain separate explicit operator controls.

The Finding path applies the same NFKC normalization and 512-Unicode-codepoint
limit as **New Query** and **Revise Query** before it sends a goal. While a
Finding task is active, the query field and both manual query buttons are
disabled: the active-search manager owns the authoritative query ID/version
until the action reaches a terminal result.

**Stop Finding** sends both action cancellation and the explicit cancellation
service request. If no terminal action result arrives within three seconds,
the button becomes **Retry Stop** and the panel reports that cancellation is
unconfirmed. Retry sends both cancellation requests again and restarts the
deadline. An accepted cancellation request is not proof that motion has
stopped; RC override and E-stop remain authoritative throughout.

Use the installed bringup entry point instead of loading the plugin manually:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 run track_robot_bringup semantic_search_ctl visualize phase1
ros2 run track_robot_bringup semantic_search_ctl visualize phase2
```

The visualization runs in the foreground. Closing RViz also stops the
visualization-owned image overlay node, while a separately running
semantic-search stack and externally owned hardware remain untouched.
