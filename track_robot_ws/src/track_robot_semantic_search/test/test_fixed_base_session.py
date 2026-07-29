import pytest

from track_robot_semantic_search.fixed_base_session import (
    MEMORY_LOCAL_SESSION,
    FixedBaseSession,
)


def test_fixed_base_session_keeps_one_local_epoch():
    session = FixedBaseSession(epoch_id=71, frame_id='base_link')

    first = session.build_state(1_000_000_000)
    second = session.build_state(1_100_000_000)

    assert first.localization_epoch_id == second.localization_epoch_id == 71
    assert first.memory_mode == MEMORY_LOCAL_SESSION
    assert first.canonical_frame_id == 'base_link'
    assert first.local_frame_id == 'base_link'
    assert first.base_frame_id == 'base_link'
    assert first.local_healthy is True
    assert first.world_healthy is False
    assert first.reason == 'operator_asserted_fixed_base_test'


def test_fixed_base_session_restart_uses_new_epoch():
    first = FixedBaseSession(epoch_id=71)
    second = FixedBaseSession(epoch_id=72)

    assert (
        first.build_state(1).localization_epoch_id
        != second.build_state(1).localization_epoch_id
    )


@pytest.mark.parametrize(
    'epoch_id,frame_id',
    [
        (0, 'base_link'),
        (-1, 'base_link'),
        (1, ''),
        (1, 'x' * 129),
    ],
)
def test_fixed_base_session_rejects_invalid_identity(epoch_id, frame_id):
    with pytest.raises(ValueError):
        FixedBaseSession(epoch_id=epoch_id, frame_id=frame_id)


def test_fixed_base_session_rejects_negative_stamp():
    with pytest.raises(ValueError):
        FixedBaseSession(epoch_id=1).build_state(-1)
