import math
from dataclasses import dataclass
from enum import IntEnum


class MemoryMode(IntEnum):
    OBSERVATION_ONLY = 0
    LOCAL_SESSION = 1
    WORLD = 2


@dataclass(frozen=True)
class LocalizationSample:
    stamp_ns: int
    local_pose_fresh: bool
    imu_fresh: bool
    local_tf_available: bool
    world_pose_fresh: bool
    world_tf_available: bool
    world_pose_stamp_ns: int
    world_covariance_xy_m2: float
    world_yaw_variance_rad2: float
    world_x: float
    world_y: float
    world_yaw: float
    source_timestamp_rollback: bool = False
    world_source_timestamp_rollback: bool = False


@dataclass(frozen=True)
class LocalizationDecision:
    mode: MemoryMode
    epoch_id: int
    epoch_changed: bool
    reason: str


def angle_difference(first: float, second: float) -> float:
    return math.atan2(
        math.sin(first - second), math.cos(first - second))


class LocalizationHealthEvaluator:
    def __init__(
            self,
            world_enabled: bool,
            world_stable_samples: int,
            maximum_world_xy_variance_m2: float,
            maximum_world_yaw_variance_rad2: float,
            maximum_world_jump_m: float,
            maximum_world_yaw_jump_rad: float):
        self.world_enabled = bool(world_enabled)
        self.world_stable_samples = max(1, int(world_stable_samples))
        self.maximum_world_xy_variance_m2 = float(
            maximum_world_xy_variance_m2)
        self.maximum_world_yaw_variance_rad2 = float(
            maximum_world_yaw_variance_rad2)
        self.maximum_world_jump_m = float(maximum_world_jump_m)
        self.maximum_world_yaw_jump_rad = float(
            maximum_world_yaw_jump_rad)
        self.epoch_id = 1
        self.previous_stamp_ns = None
        self.previous_world_pose = None
        self.previous_world_stamp_ns = None
        self.previous_mode = MemoryMode.OBSERVATION_ONLY
        self.world_stable_count = 0

    def _decision(
            self, mode: MemoryMode, reason: str,
            epoch_changed: bool = False) -> LocalizationDecision:
        self.previous_mode = mode
        return LocalizationDecision(
            mode=mode,
            epoch_id=self.epoch_id,
            epoch_changed=epoch_changed,
            reason=reason,
        )

    def _new_epoch(self) -> None:
        self.epoch_id += 1
        self.world_stable_count = 0
        self.previous_world_pose = None
        self.previous_world_stamp_ns = None

    def update(self, sample: LocalizationSample) -> LocalizationDecision:
        if sample.stamp_ns < 0:
            raise ValueError('stamp_ns must be non-negative')
        if sample.source_timestamp_rollback:
            self._new_epoch()
            self.previous_stamp_ns = sample.stamp_ns
            return self._decision(
                MemoryMode.OBSERVATION_ONLY,
                'timestamp_rollback',
                epoch_changed=True,
            )
        if (self.previous_stamp_ns is not None and
                sample.stamp_ns < self.previous_stamp_ns):
            self._new_epoch()
            self.previous_stamp_ns = sample.stamp_ns
            return self._decision(
                MemoryMode.OBSERVATION_ONLY,
                'timestamp_rollback',
                epoch_changed=True,
            )
        self.previous_stamp_ns = sample.stamp_ns

        local_checks = (
            (sample.local_pose_fresh, 'local_pose_stale'),
            (sample.imu_fresh, 'imu_stale'),
            (sample.local_tf_available, 'local_tf_unavailable'),
        )
        for healthy, reason in local_checks:
            if not healthy:
                changed = self.previous_mode != MemoryMode.OBSERVATION_ONLY
                if changed:
                    self._new_epoch()
                return self._decision(
                    MemoryMode.OBSERVATION_ONLY, reason, changed)

        world_values_finite = all(math.isfinite(value) for value in (
            sample.world_covariance_xy_m2,
            sample.world_yaw_variance_rad2,
            sample.world_x,
            sample.world_y,
            sample.world_yaw,
        ))
        world_healthy = (
            self.world_enabled and
            sample.world_pose_fresh and
            sample.world_tf_available and
            world_values_finite and
            sample.world_covariance_xy_m2 >= 0.0 and
            sample.world_yaw_variance_rad2 >= 0.0 and
            sample.world_covariance_xy_m2 <=
            self.maximum_world_xy_variance_m2 and
            sample.world_yaw_variance_rad2 <=
            self.maximum_world_yaw_variance_rad2
        )
        if not world_healthy:
            changed = self.previous_mode == MemoryMode.WORLD
            if changed:
                self._new_epoch()
            self.world_stable_count = 0
            self.previous_world_pose = None
            reason = (
                'world_disabled' if not self.world_enabled
                else 'world_pose_unhealthy')
            return self._decision(
                MemoryMode.LOCAL_SESSION, reason, changed)

        if sample.world_pose_stamp_ns < 0:
            world_state_active = (
                self.previous_mode == MemoryMode.WORLD or
                self.world_stable_count > 0 or
                self.previous_world_pose is not None or
                self.previous_world_stamp_ns is not None
            )
            if world_state_active:
                self._new_epoch()
            return self._decision(
                MemoryMode.LOCAL_SESSION,
                'world_stamp_invalid',
                epoch_changed=world_state_active,
            )
        if sample.world_source_timestamp_rollback:
            self._new_epoch()
            return self._decision(
                MemoryMode.LOCAL_SESSION,
                'world_timestamp_rollback',
                epoch_changed=True,
            )
        if (self.previous_world_stamp_ns is not None and
                sample.world_pose_stamp_ns < self.previous_world_stamp_ns):
            self._new_epoch()
            return self._decision(
                MemoryMode.LOCAL_SESSION,
                'world_timestamp_rollback',
                epoch_changed=True,
            )
        new_world_sample = (
            sample.world_pose_stamp_ns != self.previous_world_stamp_ns)
        if not new_world_sample:
            if self.world_stable_count < self.world_stable_samples:
                return self._decision(
                    MemoryMode.LOCAL_SESSION, 'world_stabilizing')
            return self._decision(MemoryMode.WORLD, 'world_healthy')
        self.previous_world_stamp_ns = sample.world_pose_stamp_ns

        current_pose = (sample.world_x, sample.world_y, sample.world_yaw)
        if self.previous_world_pose is not None:
            distance = math.hypot(
                current_pose[0] - self.previous_world_pose[0],
                current_pose[1] - self.previous_world_pose[1],
            )
            yaw_step = abs(angle_difference(
                current_pose[2], self.previous_world_pose[2]))
            if (distance > self.maximum_world_jump_m or
                    yaw_step > self.maximum_world_yaw_jump_rad):
                self._new_epoch()
                self.previous_world_pose = current_pose
                self.previous_world_stamp_ns = sample.world_pose_stamp_ns
                return self._decision(
                    MemoryMode.LOCAL_SESSION,
                    'world_pose_jump',
                    epoch_changed=True,
                )

        self.previous_world_pose = current_pose
        self.world_stable_count += 1
        if self.world_stable_count < self.world_stable_samples:
            return self._decision(
                MemoryMode.LOCAL_SESSION, 'world_stabilizing')
        return self._decision(MemoryMode.WORLD, 'world_healthy')
