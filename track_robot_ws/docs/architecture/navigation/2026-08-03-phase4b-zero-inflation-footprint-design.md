# Phase 4B Zero-Inflation Footprint Design

## Decision

Phase 4B uses the measured rectangular robot body as the collision footprint:

- length: `0.88 m` (`x = +/-0.44 m`)
- width: `0.80 m` (`y = +/-0.40 m`)
- additional Nav2 and safety inflation: `0.0 m`
- Nav2 footprint padding: `0.0 m`

The same geometry is applied to both Nav2 costmaps, the local obstacle map,
and the Nav2 motion safety supervisor. Both Nav2 inflation layers are disabled.
Keeping these values consistent avoids
planning with one robot size and supervising with another.

## Safety Boundary

This test mode disables only extra obstacle inflation. It does not disable:

- the physical footprint collision check;
- LiDAR obstacle marking and clearing;
- braking-distance and stale-input checks;
- the motion safety supervisor;
- the final velocity gate or RC override.

The local obstacle self-filter margin remains `0.03 m`. It removes chassis and
sensor-mount returns and is not a navigation clearance around external
obstacles.

## Alternatives Rejected

- Changing Nav2 only would leave the supervisor using the old oversized body.
- Changing the supervisor only would leave Nav2 producing unnecessarily wide
  detours.
- Removing obstacle layers would make the robot body geometry meaningless and
  is outside the requested test.

## Validation

Contract tests must prove all active Phase 4B configs use the same footprint
and zero inflation. The package tests and build must remain green. A runtime
test may then confirm the loaded parameters before the operator starts motion.
