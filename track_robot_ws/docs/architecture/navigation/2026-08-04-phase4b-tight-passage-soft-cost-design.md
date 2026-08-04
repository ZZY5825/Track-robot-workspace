# Phase 4B Tight-Passage Soft-Cost Design

## Objective

Allow the Bunker to use a measured `1.0 m` passage more readily without making
the robot's physical collision boundary traversable.

## Decision

Keep the existing geometry and collision boundary unchanged:

- physical footprint: `0.88 m x 0.80 m`;
- local and global inflation radius: `0.60 m`;
- footprint padding: `0.0 m`;
- safety supervisor and final velocity gate: unchanged.

Reduce only the traversable soft-cost gradient by changing the synchronized
Nav2 inflation decay parameters from `7.0` to `12.0` in:

- the local costmap inflation layer;
- the global costmap inflation layer;
- the regulated pure-pursuit controller's inflation-cost model.

The higher scaling factor makes costs outside the inscribed footprint decay
faster. It does not lower lethal obstacle cells or the inscribed collision
boundary. A `1.0 m` passage therefore remains geometrically constrained by the
`0.80 m` body, while Navfn is less likely to reject or avoid it solely because
of high soft costs.

## Alternatives

- Reducing `inflation_radius` to `0.50 m` was rejected for this step because
  Navfn does not reason about the rectangular footprint orientation. It could
  produce corner-clipping plans that the controller later rejects.
- Smac Hybrid-A* remains the preferred future footprint-aware planner, but its
  Foxy plugin is not installed and introducing it is outside this bounded
  tuning change.

## Regression Gates

- Contract tests must keep the `0.60 m` radii and require all three scaling
  parameters to equal `12.0`.
- Navigation and safety tests must remain green.
- `track_robot_navigation` must build successfully.
- Runtime validation must first use planning-only mode, followed by supervised
  low-speed execution through a measured passage no narrower than `1.0 m`.
- Reject the tuning if it causes obstacle contact, corner clipping, increased
  controller collision stops, or bypasses the existing safety chain.
