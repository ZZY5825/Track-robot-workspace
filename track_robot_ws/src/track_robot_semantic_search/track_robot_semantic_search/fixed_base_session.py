"""Pure contract for an operator-asserted, fixed-base test session."""

from dataclasses import dataclass


MEMORY_LOCAL_SESSION = 1
_REASON = 'operator_asserted_fixed_base_test'


@dataclass(frozen=True)
class FixedBaseState:
    stamp_ns: int
    memory_mode: int
    localization_epoch_id: int
    canonical_frame_id: str
    local_frame_id: str
    base_frame_id: str
    local_healthy: bool
    world_healthy: bool
    reason: str


class FixedBaseSession:
    """Keep one bounded local-session identity for a stationary test."""

    def __init__(self, epoch_id: int, frame_id: str = 'base_link'):
        if not isinstance(epoch_id, int) or isinstance(epoch_id, bool):
            raise ValueError('epoch_id must be a positive integer')
        if epoch_id <= 0:
            raise ValueError('epoch_id must be a positive integer')
        if not isinstance(frame_id, str) or not 1 <= len(frame_id) <= 128:
            raise ValueError('frame_id must contain 1 to 128 characters')
        self._epoch_id = epoch_id
        self._frame_id = frame_id

    def build_state(self, stamp_ns: int) -> FixedBaseState:
        if not isinstance(stamp_ns, int) or isinstance(stamp_ns, bool):
            raise ValueError('stamp_ns must be a non-negative integer')
        if stamp_ns < 0:
            raise ValueError('stamp_ns must be a non-negative integer')
        return FixedBaseState(
            stamp_ns=stamp_ns,
            memory_mode=MEMORY_LOCAL_SESSION,
            localization_epoch_id=self._epoch_id,
            canonical_frame_id=self._frame_id,
            local_frame_id=self._frame_id,
            base_frame_id=self._frame_id,
            local_healthy=True,
            world_healthy=False,
            reason=_REASON,
        )
