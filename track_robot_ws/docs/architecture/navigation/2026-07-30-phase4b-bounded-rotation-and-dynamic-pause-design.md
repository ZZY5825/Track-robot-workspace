# Phase 4B Bounded Rotation Safety and Dynamic-Pause Design

Date: 2026-07-30

## 1. Context and measured failure

The Phase 4B manual Nav2 test commanded a 0.50 m forward goal. After the
operator moved the robot outside the initial obstacle inflation zone, the
robot:

- accepted the goal;
- started at 0.10 m/s;
- travelled 0.271 m;
- then requested a small in-place correction of -0.026 rad/s.

The motion safety supervisor stopped the correction even though the reported
physical-footprint clearance was about 0.39 m. The current pure-rotation branch
checks every obstacle inside the robot's full circumscribed circle. It
therefore models an unlimited rotation rather than the finite angle required
to stop the current angular command.

Evidence is recorded in:

```text
/tmp/phase4b_manual_motion_0p1_clear_20260730
```

The relevant navigation and safety baseline has 29 passing tests.

## 2. Goals

1. Replace the unlimited pure-rotation collision approximation with a bounded,
   braking-based swept-footprint check.
2. Increase the supervised Phase 4B speed limits to 0.15 m/s linear and
   0.50 rad/s angular.
3. Reduce additional obstacle inflation to 65% of its current value without
   changing the physical robot footprint.
4. Allow a transient dynamic obstacle to pause navigation and automatically
   resume after the obstacle clears.
5. Preserve immediate fail-closed handling for RC override, E-stop, stale
   inputs, base faults and invalid semantic references.

## 3. Non-goals

- No change to the physical Bunker footprint.
- No bypass of the motion safety supervisor or final velocity gate.
- No automatic re-arm after RC override, E-stop or a base fault.
- No semantic output may command the base directly.
- No indefinite recovery loop.
- No change to Phase 2 identity ownership or Phase 3 target ranking.

## 4. Selected design

### 4.1 Bounded pure-rotation collision prediction

For a pure-rotation command with angular speed `omega`, compute the maximum
rotation that can occur before a supervised stop:

```text
stop_angle =
    omega^2 / (2 * angular_braking_deceleration)
  + abs(omega) * response_latency
  + fixed_rotation_margin
```

Initial parameters:

```text
angular_braking_deceleration: 0.80 rad/s^2
fixed_rotation_margin:        0.05 rad
```

The supervisor samples the robot's inflated rectangular footprint from zero
to `stop_angle` using the existing trajectory time step. A collision in that
finite swept region produces the existing BLOCKED state and zero safe command.
Obstacles outside that region do not block a small heading correction.

The existing curved-motion prediction remains unchanged. Existing message
types, topics and state values also remain unchanged.

### 4.2 Speed envelope

The Phase 4B Nav2 path is limited consistently at every layer:

| Parameter | Previous | New |
| --- | ---: | ---: |
| desired linear velocity | 0.10 m/s | 0.15 m/s |
| final linear hard limit | 0.10 m/s | 0.15 m/s |
| rotate-to-heading velocity | 0.20 rad/s | 0.40 rad/s |
| final angular hard limit | 0.25 rad/s | 0.50 rad/s |
| linear acceleration | 1.00 m/s^2 | 1.50 m/s^2 |
| angular acceleration | 0.25 rad/s^2 | 0.50 rad/s^2 |

At the 10 Hz controller rate, 1.50 m/s^2 permits the first linear command to
reach 0.15 m/s. The existing linear braking-distance formula remains active.
At 0.15 m/s its configured stopping envelope grows automatically from about
0.495 m to about 0.533 m.

### 4.3 Inflation

Only the additional buffer outside the physical footprint changes:

| Layer | Previous | New (65%) |
| --- | ---: | ---: |
| safety supervisor inflation | 0.20 m | 0.13 m |
| obstacle-map visualization inflation | 0.20 m | 0.13 m |
| Nav2 local/global costmap inflation radius | 0.25 m | 0.1625 m |

The Nav2 inflation remains slightly larger than the final safety inflation so
the planner prefers paths that the independent safety supervisor can accept.

### 4.4 Transient dynamic-obstacle pause

When an obstacle enters the safety envelope:

1. the safety supervisor remains armed but publishes zero velocity;
2. the active Nav2 goal remains active;
3. planner replanning continues at 1 Hz;
4. the controller continues checking the current path;
5. when the obstacle clears, inputs are fresh and the path is valid, the safe
   command resumes automatically.

The Nav2 progress allowance increases from 8 seconds to 30 seconds. This gives
a walking person time to cross without creating an unbounded wait. If the
obstacle remains for 30 seconds, Nav2 aborts normally and the command becomes
stale/zero.

For semantic execution, an armed `STATE_BLOCKED` is a transient HOLD condition,
not a reason to cancel the active goal. All other non-permitting safety states
remain fail-closed:

- RC override: cancel and disarm;
- E-stop: cancel and latch;
- stale odometry/map/command: cancel;
- base fault: cancel;
- target loss or reference mismatch: cancel.

## 5. Interface and configuration impact

Public ROS interfaces do not change. The following new parameters are added to
the motion safety supervisor:

```text
angular_braking_deceleration
fixed_rotation_margin
```

All changed behavior remains configurable in the existing YAML files.

## 6. Validation

### 6.1 Automated regression

Add tests proving:

1. a -0.026 rad/s correction with an obstacle outside the bounded stop sweep
   is not rejected by the former full-circle approximation;
2. a point inside the bounded rotational stop sweep still blocks;
3. the stop angle grows with angular speed;
4. all three Phase 4B velocity layers use 0.15 m/s and 0.50 rad/s limits;
5. both inflation layers equal 65% of their previous values;
6. BLOCKED holds an active semantic goal while RC/E-stop/stale states cancel;
7. Nav2 retains the goal for a bounded 30-second no-progress interval.

Run affected unit/config tests, package tests and the complete existing
Phase 0-4B regression suite.

### 6.2 Offline evidence

Replay the recorded Phase 4B bag and compare:

- old full-circle result: BLOCKED at -0.026 rad/s;
- new bounded-sweep result;
- minimum reported clearance;
- safe/final command caps;
- state transitions and stale/drop rates.

### 6.3 Live safety gates

Before executable motion:

1. run a disarmed Nav2 probe and verify first linear raw command is 0.15 m/s;
2. verify safe and final commands remain zero while disarmed;
3. run a short supervised manual goal in a clear area;
4. inject a walking-person obstacle and verify zero motion followed by
   automatic resume;
5. inject RC override and verify immediate disarm with no automatic resume.

All ROS nodes and services are stopped after testing.

## 7. Rollback

The change is rollback-safe through configuration and one isolated collision
model commit:

- restore Phase 4B speed and inflation YAML values;
- restore the previous pure-rotation branch;
- restore the 8-second progress allowance.

No recorded data, public messages, topics, IDs or semantic-memory state require
migration.
