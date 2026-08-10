import pytest

from track_robot_navigation.physical_recovery import (
    PhysicalRecoveryPolicy,
    RecoveryCommand,
    RecoveryStage,
)


def policy(enabled=True, maximum_cycles=2, spin_sign=1):
    return PhysicalRecoveryPolicy(
        enabled=enabled,
        cooldown_sec=2.0,
        maximum_cycles=maximum_cycles,
        spin_sign=spin_sign,
    )


def test_successful_recoveries_follow_the_bounded_sequence():
    value = policy()

    assert value.next_command(10.0).command is RecoveryCommand.NAVIGATE

    spin = value.navigation_aborted(11.0)
    assert spin.command is RecoveryCommand.SPIN
    assert spin.stage is RecoveryStage.SPIN

    after_spin = value.recovery_finished(RecoveryCommand.SPIN, True, 12.0)
    assert after_spin.command is RecoveryCommand.NAVIGATE
    assert after_spin.stage is RecoveryStage.NAVIGATE_AFTER_SPIN

    backup = value.navigation_aborted(13.0)
    assert backup.command is RecoveryCommand.BACK_UP
    assert backup.stage is RecoveryStage.BACK_UP

    after_backup = value.recovery_finished(
        RecoveryCommand.BACK_UP, True, 14.0)
    assert after_backup.command is RecoveryCommand.NAVIGATE
    assert after_backup.stage is RecoveryStage.NAVIGATE_AFTER_BACK_UP

    hold = value.navigation_aborted(15.0)
    assert hold.command is RecoveryCommand.HOLD
    assert hold.stage is RecoveryStage.HOLD
    assert hold.cycle == 1
    assert hold.not_before_s == pytest.approx(17.0)
    assert value.next_command(16.9).command is RecoveryCommand.HOLD
    assert value.next_command(17.0).command is RecoveryCommand.NAVIGATE


def test_disabled_policy_always_uses_legacy_navigation():
    value = policy(enabled=False)

    assert value.next_command(1.0).command is RecoveryCommand.NAVIGATE
    assert value.navigation_aborted(2.0).command is RecoveryCommand.NAVIGATE
    assert value.recovery_finished(
        RecoveryCommand.SPIN, False, 3.0).command is RecoveryCommand.NAVIGATE


def test_failed_spin_advances_to_backup_without_changing_direction():
    value = policy(spin_sign=-1)

    value.navigation_aborted(1.0)
    backup = value.recovery_finished(RecoveryCommand.SPIN, False, 2.0)

    assert backup.command is RecoveryCommand.BACK_UP
    assert backup.stage is RecoveryStage.BACK_UP
    assert value.spin_sign == -1


def test_failed_backup_enters_cooldown_hold():
    value = policy()

    value.navigation_aborted(1.0)
    value.recovery_finished(RecoveryCommand.SPIN, False, 2.0)
    hold = value.recovery_finished(RecoveryCommand.BACK_UP, False, 3.0)

    assert hold.command is RecoveryCommand.HOLD
    assert hold.stage is RecoveryStage.HOLD
    assert hold.cycle == 1
    assert hold.not_before_s == pytest.approx(5.0)


def test_maximum_cycles_falls_back_to_replan_only_without_more_motion():
    value = policy(maximum_cycles=1)

    value.navigation_aborted(1.0)
    value.recovery_finished(RecoveryCommand.SPIN, False, 2.0)
    value.recovery_finished(RecoveryCommand.BACK_UP, False, 3.0)

    retry = value.next_command(5.0)
    assert retry.command is RecoveryCommand.NAVIGATE
    assert retry.stage is RecoveryStage.REPLAN_ONLY

    hold = value.navigation_aborted(6.0)
    assert hold.command is RecoveryCommand.HOLD
    assert hold.stage is RecoveryStage.REPLAN_ONLY
    assert value.next_command(7.9).command is RecoveryCommand.HOLD
    assert value.next_command(8.0).command is RecoveryCommand.NAVIGATE
    assert value.navigation_aborted(9.0).command is RecoveryCommand.HOLD


def test_reset_clears_progress_but_retains_spin_direction():
    value = policy(spin_sign=-1)
    value.navigation_aborted(1.0)

    decision = value.reset()

    assert decision.command is RecoveryCommand.NAVIGATE
    assert decision.stage is RecoveryStage.IDLE
    assert decision.cycle == 0
    assert value.spin_sign == -1


@pytest.mark.parametrize('cooldown_sec', [0.0, 0.099, 10.001])
def test_rejects_invalid_cooldown(cooldown_sec):
    with pytest.raises(ValueError):
        PhysicalRecoveryPolicy(
            enabled=True,
            cooldown_sec=cooldown_sec,
            maximum_cycles=1,
        )


@pytest.mark.parametrize('maximum_cycles', [0, 6])
def test_rejects_invalid_cycle_limit(maximum_cycles):
    with pytest.raises(ValueError):
        PhysicalRecoveryPolicy(
            enabled=True,
            cooldown_sec=1.0,
            maximum_cycles=maximum_cycles,
        )


@pytest.mark.parametrize('spin_sign', [-2, 0, 2])
def test_rejects_invalid_spin_direction(spin_sign):
    with pytest.raises(ValueError):
        PhysicalRecoveryPolicy(
            enabled=True,
            cooldown_sec=1.0,
            maximum_cycles=1,
            spin_sign=spin_sign,
        )
