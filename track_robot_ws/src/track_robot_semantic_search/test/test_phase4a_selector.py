from dataclasses import replace
from pathlib import Path

import pytest

from track_robot_semantic_search.phase4a_selector import (
    FixedBaseTargetSelector,
    ObjectCandidate,
    SelectionSnapshot,
    SelectorConfig,
    classify_spatial_support,
)


NOW_NS = 10_000_000_000
PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_position_valid_camera_object_is_camera_depth_without_lidar():
    assert classify_spatial_support(
        support='camera_only',
        position_valid=True,
        fallback_depth_available=False,
    ) == 'camera_depth'
    assert classify_spatial_support(
        support='camera_only',
        position_valid=False,
        fallback_depth_available=False,
    ) == 'other'


def test_selector_never_overwrites_canonical_phase2_position_with_fallback():
    source = (
        PACKAGE_ROOT
        / 'track_robot_semantic_search'
        / 'phase4a_selector_node.py'
    ).read_text()

    geometry_body = source.split(
        '    def _geometry(self, message, now_ns):', 1)[1].split(
        '    def _candidate(', 1)[0]
    assert 'if message.position_valid:' in geometry_body
    assert geometry_body.index('if message.position_valid:') < (
        geometry_body.index('self._depth_geometry.get'))


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


def test_selector_holds_last_ready_target_during_brief_depth_gap():
    selector = FixedBaseTargetSelector(SelectorConfig(
        confirmation_snapshots=1))
    ready = selector.update(snapshot((candidate(support='camera_depth'),)))

    held = selector.update(snapshot((
        candidate(support='other'),),
        now_ns=NOW_NS + 200_000_000))

    assert held.status == 'READY'
    assert held.reason == 'holding_last_target'
    assert held.target == ready.target


def test_selector_does_not_hold_last_target_past_age_limit():
    selector = FixedBaseTargetSelector(SelectorConfig(
        confirmation_snapshots=1))
    selector.update(snapshot((candidate(support='camera_depth'),)))

    expired = selector.update(snapshot(
        (candidate(support='other'),),
        now_ns=NOW_NS + 1_100_000_000))

    assert expired.status == 'NOT_READY'
    assert expired.target is None


def test_selector_holds_same_target_during_small_relevance_dip():
    selector = FixedBaseTargetSelector(SelectorConfig(
        minimum_relevance=0.42,
        confirmation_snapshots=1))
    ready = selector.update(snapshot((
        candidate(relevance=0.43),)))

    held = selector.update(snapshot((
        candidate(relevance=0.41),),
        now_ns=NOW_NS + 200_000_000))

    assert held.status == 'READY'
    assert held.reason == 'holding_last_target'
    assert held.target == ready.target


def test_selector_rejects_relevance_below_retention_floor():
    selector = FixedBaseTargetSelector(SelectorConfig(
        minimum_relevance=0.42,
        retained_minimum_relevance=0.40,
        confirmation_snapshots=1))
    selector.update(snapshot((candidate(relevance=0.43),)))

    rejected = selector.update(snapshot((
        candidate(relevance=0.39),),
        now_ns=NOW_NS + 200_000_000))

    assert rejected.reason == 'below_test_relevance'
    assert rejected.target is None


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
    with pytest.raises(ValueError):
        FixedBaseTargetSelector(SelectorConfig(
            minimum_relevance=0.50,
            retained_minimum_relevance=0.51))
