# Task 1 Implementation Report

Status: DONE

Commit: `3b36604 fix: make Bunker RC mode revoke follow control`

Implemented:

- Bunker `control_mode == 3` is an authoritative RC override even with centered sticks.
- Stick movement remains a redundant RC override source.
- RC takeover immediately disarms safety, clears bootstrap command state, and publishes zero safe command.
- Returning to CAN mode leaves safety disarmed.
- Follow decision requests logical target reset when entering RC override.
- Added isolated ROS-domain launch tests and component documentation.

Verification:

```text
colcon build --packages-select track_robot_safety track_robot_decision
Result: 2 packages finished successfully.

colcon test --packages-select track_robot_safety track_robot_decision
colcon test-result --verbose
Result: 5 tests, 0 errors, 0 failures, 0 skipped.
```

The first sandboxed launch-test attempt failed before node startup with
`getifaddrs: Operation not permitted`. Re-running with local DDS/network
permission passed without code changes.

Concerns: none.

## Review Fix

Status: DONE

Implemented:

- Queue RC target resets until a successful `/human_tracking/reset_target` response.
- Limit reset work to one in-flight request and retry unavailable or failed requests at 200 ms intervals.
- Assert a normal RC transition produces exactly one successful reset after settling.
- Add a failed-first-response launch test that proves a subsequent reset succeeds.
- Rename the RC safety test to match its disarmed-after-CAN-restore assertion.

TDD and verification:

```text
source /opt/ros/foxy/setup.bash && source track_robot_ws/install/setup.bash && \
colcon test --base-paths track_robot_ws/src --build-base track_robot_ws/build \
  --install-base track_robot_ws/install --test-result-base track_robot_ws/test_results \
  --packages-select track_robot_decision \
  --ctest-args -R test_follow_decision_launch --output-on-failure
Result before implementation: expected failure. `test_rc_reset_retries_after_failed_response`
timed out after the first failed Trigger response; no retry was sent.

source /opt/ros/foxy/setup.bash && colcon build --base-paths track_robot_ws/src \
  --build-base track_robot_ws/build --install-base track_robot_ws/install \
  --packages-select track_robot_decision track_robot_safety
Result: 2 packages finished successfully.

source /opt/ros/foxy/setup.bash && source track_robot_ws/install/setup.bash && \
colcon test --base-paths track_robot_ws/src --build-base track_robot_ws/build \
  --install-base track_robot_ws/install --test-result-base track_robot_ws/test_results \
  --packages-select track_robot_decision track_robot_safety \
  --ctest-args -R 'test_(follow_decision_launch|rc_control_mode_launch)' --output-on-failure
Result: 2 tests, 0 errors, 0 failures, 0 skipped.
```

The initial sandboxed launch-test attempt failed before node startup with
`getifaddrs: Operation not permitted`. The expected-red and passing ROS launch
tests were rerun with local DDS/network permission. A later direct sandboxed
`ctest` check failed for the same environment restriction and did not indicate
a code regression.

Concerns: none.

## Final Missing Safety Regression

Added a focused launch regression proving a non-neutral RC stick disarms safety
while Bunker remains in CAN control mode, zeros the safe command, and does not
automatically rearm after the stick returns to neutral.

Verification:

```text
source /opt/ros/foxy/setup.bash && source track_robot_ws/install/setup.bash && \
colcon test --base-paths track_robot_ws/src --build-base track_robot_ws/build \
  --install-base track_robot_ws/install --test-result-base track_robot_ws/test_results \
  --packages-select track_robot_safety \
  --ctest-args -R test_rc_control_mode_launch --output-on-failure
Result: 1 package finished successfully.

source /opt/ros/foxy/setup.bash && colcon test-result \
  --test-result-base track_robot_ws/test_results --verbose
Result: 2 tests, 0 errors, 0 failures, 0 skipped.
```

The focused launch test was run with DDS/network permission.

Concerns: none.

## Remaining Review Findings

Status: DONE

Implemented:

- Latch automatic target behavior on every transition into RC override.
- Keep stale pre-RC target evidence motionless after returning to DISARMED until
  the RC reset succeeds and a no-target `TargetState` is observed.
- Accept a newly locked logical target normally after the relock latch clears.
- Assign a generation to every queued RC or target-lost reset.
- Ignore stale async responses without clearing a newer pending generation.
- Keep one reset request in flight while retaining retry behavior for failed or
  unavailable services.
- Add isolated launch-test nodes for relock, delayed response, unavailable
  service, and overlapping reset-generation regressions.

TDD red results:

```text
colcon test --packages-select track_robot_decision \
  --ctest-args -R test_follow_decision_launch --output-on-failure
Result before relock implementation: expected failure.
test_rc_clear_requires_reset_and_no_target_before_relock observed behavior 2
(FOLLOW_CONFIRMED) after RC_OVERRIDE -> DISARMED with the stale target.

colcon test --packages-select track_robot_decision \
  --ctest-args -R test_follow_decision_launch --output-on-failure
Result before generation-safe callbacks: expected failure.
test_new_rc_generation_waits_for_own_reset_response timed out with one reset
success because the older response cleared the newer pending obligation.
```

Final verification:

```text
source /opt/ros/foxy/setup.bash && colcon build \
  --base-paths track_robot_ws/src \
  --build-base track_robot_ws/build \
  --install-base track_robot_ws/install \
  --packages-select track_robot_decision
Result: 1 package finished successfully.

source /opt/ros/foxy/setup.bash && source track_robot_ws/install/setup.bash && \
colcon test --base-paths track_robot_ws/src \
  --build-base track_robot_ws/build \
  --install-base track_robot_ws/install \
  --test-result-base track_robot_ws/test_results \
  --packages-select track_robot_decision \
  --ctest-args -R test_follow_decision_launch --output-on-failure
colcon test-result --test-result-base track_robot_ws/test_results --verbose
Result: 2 tests, 0 errors, 0 failures, 0 skipped.
```

The ROS launch tests were run with local DDS/network permission as required.

Concerns: none.

## Final Task 1 Review Fix

Status: DONE

Implemented:

- Detect RC takeover edges from `rc_override_active || STATE_RC_OVERRIDE`.
- Queue one generation-safe RC reset and relock on the false-to-true takeover edge,
  even when `STATE_EMERGENCY_STOP` remains the selected safety state.
- Preserve hard-stop selection for emergency stop while preventing stale target reuse
  until reset success and `NO_TARGET` are both observed.
- Add the masked-emergency-stop RC launch regression and make repeated test
  publications tolerant of local DDS endpoint discovery.

TDD and verification:

```text
source /opt/ros/foxy/setup.bash && source track_robot_ws/install/setup.bash && \
colcon test --base-paths track_robot_ws/src --build-base track_robot_ws/build \
  --install-base track_robot_ws/install --test-result-base track_robot_ws/test_results \
  --packages-select track_robot_decision \
  --ctest-args -R test_follow_decision_launch --output-on-failure
Result before implementation: expected failure.
test_emergency_stop_masked_rc_requires_relock timed out waiting for the reset
service after STATE_EMERGENCY_STOP with rc_override_active=true.

source /opt/ros/foxy/setup.bash && colcon build --base-paths track_robot_ws/src \
  --build-base track_robot_ws/build --install-base track_robot_ws/install \
  --packages-select track_robot_decision
Result: 1 package finished successfully.

source /opt/ros/foxy/setup.bash && source track_robot_ws/install/setup.bash && \
colcon test --base-paths track_robot_ws/src --build-base track_robot_ws/build \
  --install-base track_robot_ws/install --test-result-base track_robot_ws/test_results \
  --packages-select track_robot_decision \
  --ctest-args -R test_follow_decision_launch --output-on-failure
Result: 1 package finished successfully; 7 tests, 0 errors, 0 failures, 0 skipped.
```

The ROS launch tests were run with local DDS/network permission.

Concerns: none.
