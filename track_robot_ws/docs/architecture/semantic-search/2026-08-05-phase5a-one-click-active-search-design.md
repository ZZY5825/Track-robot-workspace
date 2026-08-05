# Phase 5A One-Click Active Search Design

Date: 2026-08-05

## Decision

`Start Finding` is the operator's authorization to run one bounded,
rotation-only search.  Phase 5A must not require a second authorization
service handshake or expose `Retry Stop` to the operator.

This matches the established `Start Approaching` interaction: one explicit
operator action starts the supervised behavior, while RC override, E-stop and
the command watchdog remain authoritative.

## Runtime flow

```text
Start Finding
  -> SearchForObject action
  -> passive semantic observation
  -> no confirmed candidate
  -> bounded SearchMotionIntent
  -> search_motion_adapter automatically arms the existing safety chain
  -> Nav2 Spin
  -> motion safety supervisor
  -> cmd_vel gate
  -> Bunker base
```

No component may publish directly to final `/cmd_vel` except the existing
gate.  Forward motion remains forbidden during Phase 5A search.

## Operator controls

- `Start Finding`: submit one action goal and authorize its bounded rotations.
- `Stop Finding`: cancel the action and current Spin, disarm motion and return
  the panel to `Start Finding` after the terminal result.
- The panel has no authorization-pending state and no `Retry Stop` state.
- Query controls remain locked only while one search action is active.

## State ownership

- The active-search manager owns query/search progress and terminal results.
- The motion adapter owns Nav2 Spin execution and calls the existing safety arm
  and disarm services internally.
- The panel is only an action client; it does not call a second rotation
  authorization service.
- Repeated perception rankings while waiting, rotating or settling must not
  create duplicate Spin goals.

## Failure behavior

- RC override, E-stop, stale odometry, stale safety state or rejected safety
  arm stops the Spin and returns a specific terminal reason.
- Action cancellation always publishes a stop intent and clears the server's
  single-goal reservation.
- A disconnected or late callback must not leave the button in a retry-only
  state; after terminal or confirmed cancellation the UI returns to idle.

## Acceptance criteria

1. One click reaches `SPIN_REQUESTED` without a separate authorization RPC.
2. Non-zero angular velocity appears in raw, safe and final command topics.
3. No forward velocity is produced.
4. `Stop Finding`, RC override and E-stop stop rotation.
5. A completed/cancelled/failed search restores `Start Finding`.
6. Existing passive and shadow modes remain motionless.
7. Existing Phase 0-4A interfaces and the safety/gate chain remain unchanged.
