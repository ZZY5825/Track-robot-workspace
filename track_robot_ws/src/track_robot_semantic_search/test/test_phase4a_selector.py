from dataclasses import replace

import pytest

from track_robot_semantic_search.phase4a_selector import (
    FixedBaseTargetSelector,
    ObjectCandidate,
    SelectionSnapshot,
    SelectorConfig,
)


NOW_NS = 10_000_000_000


def candidate(**overrides):
    value = ObjectCandidate(
        memory_epoch_id=11,
        global_object_id=42,
        localization_epoch_id=71,
        query_id=2026072801,
        query_version=1,
        lifecycle='confirmed',
        support='camera_lidar',
        position_frame_id='base_link',
        position_valid=True,
        x=1.60,
        y=-0.20,
        z=0.35,
        relevance=0.72,
        uncertainty=0.28,
        last_seen_ns=NOW_NS - 50_000_000,
    )
    return replace(value, **overrides)


def snapshot(candidates=(), **overrides):
    value = SelectionSnapshot(
        now_ns=NOW_NS,
        query_id=2026072801,
        query_version=1,
        candidates=tuple(candidates),
    )
    return replace(value, **overrides)


def test_selector_requires_three_consecutive_snapshots():
    selector = FixedBaseTargetSelector(SelectorConfig())

    first = selector.update(snapshot((candidate(),)))
    second = selector.update(snapshot((candidate(),)))
    third = selector.update(snapshot((candidate(),)))

    assert first.reason == 'confirming_target'
    assert second.reason == 'confirming_target'
    assert third.reason == 'ready'
    assert third.target == candidate()


def test_selector_accepts_confirmed_stereo_depth_support():
    selector = FixedBaseTargetSelector(SelectorConfig(
        confirmation_snapshots=1))

    result = selector.update(snapshot((
        candidate(support='camera_depth'),)))

    assert result.reason == 'ready'
    assert result.target.support == 'camera_depth'


def test_selector_key_change_restarts_confirmation():
    selector = FixedBaseTargetSelector(SelectorConfig())
    selector.update(snapshot((candidate(),)))
    selector.update(snapshot((candidate(),)))

    result = selector.update(snapshot((candidate(global_object_id=43),)))

    assert result.reason == 'confirming_target'
    assert result.target is None


def test_selector_rejects_ambiguous_top_pair():
    selector = FixedBaseTargetSelector(SelectorConfig())
    result = selector.update(snapshot((
        candidate(relevance=0.72),
        candidate(global_object_id=43, relevance=0.68),
    )))
    assert result.reason == 'ambiguous_target'


@pytest.mark.parametrize(
    'changed,reason',
    [
        ({'query_id': 99}, 'query_mismatch'),
        ({'lifecycle': 'tentative'}, 'target_not_confirmed'),
        ({'support': 'camera_only'}, 'no_spatial_support'),
        ({'position_valid': False}, 'invalid_position'),
        ({'position_frame_id': 'map'}, 'frame_mismatch'),
        ({'relevance': 0.49}, 'below_test_relevance'),
        ({'uncertainty': 0.51}, 'uncertainty_too_high'),
        ({'last_seen_ns': NOW_NS - 1_000_000_001}, 'stale_target'),
    ],
)
def test_selector_fails_closed_for_candidate_gate(changed, reason):
    selector = FixedBaseTargetSelector(SelectorConfig())
    result = selector.update(snapshot((candidate(**changed),)))
    assert result.reason == reason
    assert result.target is None


def test_selector_rejects_snapshot_query_mismatch():
    selector = FixedBaseTargetSelector(SelectorConfig())
    result = selector.update(snapshot(
        (candidate(),), query_version=2))
    assert result.reason == 'query_mismatch'


def test_selector_rejects_unstable_position_spread():
    selector = FixedBaseTargetSelector(SelectorConfig())
    selector.update(snapshot((candidate(x=1.0, y=0.0),)))
    selector.update(snapshot((candidate(x=1.0, y=0.0),)))

    result = selector.update(snapshot((candidate(x=1.8, y=0.0),)))

    assert result.reason == 'unstable_position'
    assert result.target is None


def test_selector_clears_confirmation_after_no_target():
    selector = FixedBaseTargetSelector(SelectorConfig())
    selector.update(snapshot((candidate(),)))
    selector.update(snapshot((candidate(),)))
    assert selector.update(snapshot()).reason == 'no_target'
    assert selector.update(snapshot((candidate(),))).reason == 'confirming_target'


def test_selector_rejects_invalid_config():
    with pytest.raises(ValueError):
        FixedBaseTargetSelector(SelectorConfig(confirmation_snapshots=0))
