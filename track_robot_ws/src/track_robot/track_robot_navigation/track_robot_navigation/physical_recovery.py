"""Bounded physical recovery policy for a preserved semantic mission.

This module is intentionally independent from ROS and target ownership.  It
only decides which already-supervised navigation action should be attempted
next after a navigation failure.
"""

from dataclasses import dataclass
from enum import Enum
import math


class RecoveryStage(Enum):
    IDLE = 'idle'
    SPIN = 'spin'
    NAVIGATE_AFTER_SPIN = 'navigate_after_spin'
    BACK_UP = 'back_up'
    NAVIGATE_AFTER_BACK_UP = 'navigate_after_back_up'
    HOLD = 'hold'
    REPLAN_ONLY = 'replan_only'


class RecoveryCommand(Enum):
    NAVIGATE = 'navigate'
    SPIN = 'spin'
    BACK_UP = 'back_up'
    HOLD = 'hold'


@dataclass(frozen=True)
class RecoveryDecision:
    command: RecoveryCommand
    stage: RecoveryStage
    cycle: int
    reason: str
    not_before_s: float


class PhysicalRecoveryPolicy:
    """Choose a bounded recovery sequence without owning mission state."""

    def __init__(
            self, enabled, cooldown_sec, maximum_cycles, spin_sign=1):
        cooldown_sec = float(cooldown_sec)
        if not math.isfinite(cooldown_sec) or not 0.1 <= cooldown_sec <= 10.0:
            raise ValueError('cooldown_sec must be within [0.1, 10.0]')
        if isinstance(maximum_cycles, bool) or not isinstance(
                maximum_cycles, int) or not 1 <= maximum_cycles <= 5:
            raise ValueError('maximum_cycles must be an integer within [1, 5]')
        if spin_sign not in (-1, 1):
            raise ValueError('spin_sign must be -1 or 1')

        self._enabled = bool(enabled)
        self._cooldown_sec = cooldown_sec
        self._maximum_cycles = maximum_cycles
        self._spin_sign = spin_sign
        self._stage = RecoveryStage.IDLE
        self._cycle = 0
        self._not_before_s = 0.0

    @property
    def enabled(self):
        return self._enabled

    @property
    def stage(self):
        return self._stage

    @property
    def cycle(self):
        return self._cycle

    @property
    def spin_sign(self):
        return self._spin_sign

    @property
    def maximum_cycles(self):
        return self._maximum_cycles

    def _decision(self, command, reason):
        return RecoveryDecision(
            command=command,
            stage=self._stage,
            cycle=self._cycle,
            reason=reason,
            not_before_s=self._not_before_s,
        )

    def _legacy_decision(self):
        return RecoveryDecision(
            command=RecoveryCommand.NAVIGATE,
            stage=RecoveryStage.IDLE,
            cycle=0,
            reason='physical_recovery_disabled',
            not_before_s=0.0,
        )

    def reset(self):
        self._stage = RecoveryStage.IDLE
        self._cycle = 0
        self._not_before_s = 0.0
        if not self._enabled:
            return self._legacy_decision()
        return self._decision(RecoveryCommand.NAVIGATE, 'recovery_reset')

    def next_command(self, now_s):
        if not self._enabled:
            return self._legacy_decision()

        now_s = float(now_s)
        if now_s < self._not_before_s:
            return self._decision(RecoveryCommand.HOLD, 'recovery_cooldown')

        if self._stage is RecoveryStage.HOLD:
            if self._cycle >= self._maximum_cycles:
                self._stage = RecoveryStage.REPLAN_ONLY
                reason = 'physical_recovery_limit_reached'
            else:
                self._stage = RecoveryStage.IDLE
                reason = 'recovery_cycle_ready'
            self._not_before_s = 0.0
            return self._decision(RecoveryCommand.NAVIGATE, reason)

        if self._stage is RecoveryStage.REPLAN_ONLY:
            self._not_before_s = 0.0
            return self._decision(
                RecoveryCommand.NAVIGATE, 'replan_only_retry')

        if self._stage is RecoveryStage.SPIN:
            return self._decision(RecoveryCommand.SPIN, 'spin_pending')
        if self._stage is RecoveryStage.BACK_UP:
            return self._decision(RecoveryCommand.BACK_UP, 'back_up_pending')
        return self._decision(RecoveryCommand.NAVIGATE, 'navigation_pending')

    def navigation_aborted(self, now_s):
        if not self._enabled:
            return self._legacy_decision()

        now_s = float(now_s)
        if self._stage is RecoveryStage.IDLE:
            self._stage = RecoveryStage.SPIN
            return self._decision(
                RecoveryCommand.SPIN, 'navigation_failed_try_spin')
        if self._stage is RecoveryStage.NAVIGATE_AFTER_SPIN:
            self._stage = RecoveryStage.BACK_UP
            return self._decision(
                RecoveryCommand.BACK_UP, 'navigation_failed_try_back_up')
        if self._stage is RecoveryStage.NAVIGATE_AFTER_BACK_UP:
            return self._enter_hold(now_s, 'navigation_failed_after_back_up')
        if self._stage is RecoveryStage.REPLAN_ONLY:
            self._not_before_s = now_s + self._cooldown_sec
            return self._decision(
                RecoveryCommand.HOLD, 'replan_only_navigation_failed')
        raise ValueError(
            'navigation_aborted is invalid while stage is '
            f'{self._stage.value}')

    def recovery_finished(self, command, succeeded, now_s):
        if not self._enabled:
            return self._legacy_decision()

        if command is RecoveryCommand.SPIN:
            if self._stage is not RecoveryStage.SPIN:
                raise ValueError('spin result received while spin is not pending')
            if succeeded:
                self._stage = RecoveryStage.NAVIGATE_AFTER_SPIN
                return self._decision(
                    RecoveryCommand.NAVIGATE, 'spin_succeeded')
            self._stage = RecoveryStage.BACK_UP
            return self._decision(RecoveryCommand.BACK_UP, 'spin_failed')

        if command is RecoveryCommand.BACK_UP:
            if self._stage is not RecoveryStage.BACK_UP:
                raise ValueError(
                    'back_up result received while back_up is not pending')
            if succeeded:
                self._stage = RecoveryStage.NAVIGATE_AFTER_BACK_UP
                return self._decision(
                    RecoveryCommand.NAVIGATE, 'back_up_succeeded')
            return self._enter_hold(float(now_s), 'back_up_failed')

        raise ValueError('recovery result command must be SPIN or BACK_UP')

    def _enter_hold(self, now_s, reason):
        self._cycle += 1
        self._stage = RecoveryStage.HOLD
        self._not_before_s = now_s + self._cooldown_sec
        return self._decision(RecoveryCommand.HOLD, reason)
