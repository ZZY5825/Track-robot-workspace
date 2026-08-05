from types import SimpleNamespace

from track_robot_semantic_search.active_search_manager_node import (
    _feedback_reason,
    _rotation_transition_in_progress,
)
from track_robot_semantic_search.active_search_policy import SearchState


def test_waiting_feedback_is_not_masked_by_nonterminal_adapter_reason():
    context = SimpleNamespace(
        state=SearchState.WAITING_FOR_AUTHORIZATION,
        terminal_status=None,
        terminal_reason='authorization_required',
    )

    assert _feedback_reason(context) == 'WAITING_FOR_AUTHORIZATION'


def test_terminal_feedback_keeps_the_terminal_reason():
    context = SimpleNamespace(
        state=SearchState.SAFETY_REJECTED,
        terminal_status=4,
        terminal_reason='rc_override',
    )

    assert _feedback_reason(context) == 'rc_override'


def test_waiting_rotating_and_settling_block_duplicate_rotation_decisions():
    assert _rotation_transition_in_progress(
        SearchState.WAITING_FOR_AUTHORIZATION)
    assert _rotation_transition_in_progress(SearchState.ROTATING)
    assert _rotation_transition_in_progress(SearchState.SETTLING)
    assert not _rotation_transition_in_progress(SearchState.OBSERVING)
