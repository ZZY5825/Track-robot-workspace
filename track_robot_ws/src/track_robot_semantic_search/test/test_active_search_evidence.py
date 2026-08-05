import pytest

from track_robot_semantic_search.active_search_evidence import (
    BoundedEvidenceBook,
    EvidenceConfig,
    EvidenceStatus,
    ObjectEvidenceKey,
    ViewEvidence,
)


def _key(global_object_id=22):
    return ObjectEvidenceKey(
        memory_epoch_id=4,
        global_object_id=global_object_id,
        localization_epoch_id=7,
        query_id=100,
        query_version=1,
    )


def _evidence(
        key=None,
        heading=45.0,
        stamp=10.1,
        selected=True,
        relevance=0.61,
        uncertainty=0.18):
    return ViewEvidence(
        key=key or _key(),
        heading_deg=heading,
        horizontal_fov_deg=70.0,
        source_stamp_sec=stamp,
        task_relevance=relevance,
        uncertainty=uncertainty,
        phase3_selected=selected,
    )


def _bound_book(config=None):
    book = BoundedEvidenceBook(config or EvidenceConfig.defaults())
    book.bind_domain(4, 7, 100, 1)
    return book


def test_three_fresh_phase3_snapshots_confirm_one_key():
    book = _bound_book()
    for stamp in (10.1, 10.4, 10.8):
        assert book.add(_evidence(stamp=stamp), settled_after=10.0)

    decision = book.evaluate(search_exhausted=False)

    assert decision.status is EvidenceStatus.CONFIRMED
    assert decision.selected_key == _key()
    assert decision.candidate_count == 1
    assert decision.covered_heading_count == 1


def test_pre_settle_evidence_is_rejected():
    book = _bound_book()

    accepted = book.add(_evidence(stamp=9.9), settled_after=10.0)

    assert not accepted
    assert book.record_count == 0


def test_near_duplicate_heading_does_not_expand_coverage():
    book = _bound_book()
    book.add(_evidence(heading=45.0, stamp=10.1), settled_after=10.0)
    book.add(_evidence(heading=51.0, stamp=10.2), settled_after=10.0)

    assert book.covered_heading_count == 1
    assert book.coverage_intervals_deg == ((10.0, 80.0),)


def test_competing_keys_end_uncertain_not_merged():
    book = _bound_book()
    book.add(
        _evidence(key=_key(22), heading=45.0, selected=False),
        settled_after=10.0,
    )
    book.add(
        _evidence(
            key=_key(31), heading=-45.0, stamp=10.2, selected=False),
        settled_after=10.0,
    )

    decision = book.evaluate(search_exhausted=True)

    assert decision.status is EvidenceStatus.UNCERTAIN
    assert decision.selected_key is None
    assert decision.candidate_count == 2


def test_empty_exhausted_search_is_not_found():
    decision = _bound_book().evaluate(search_exhausted=True)

    assert decision.status is EvidenceStatus.NOT_FOUND
    assert decision.reason == 'search_exhausted_without_candidates'


def test_unfinished_search_without_confirmation_keeps_observing():
    book = _bound_book()
    book.add(_evidence(selected=False), settled_after=10.0)

    assert book.evaluate(False).status is EvidenceStatus.OBSERVING


def test_epoch_or_query_change_invalidates_the_bound_domain():
    unbound = BoundedEvidenceBook(EvidenceConfig.defaults())
    assert not unbound.is_bound

    book = _bound_book()
    assert book.is_bound

    assert not book.domain_changed(4, 7, 100, 1)
    assert book.domain_changed(5, 7, 100, 1)
    assert book.domain_changed(4, 8, 100, 1)
    assert book.domain_changed(4, 7, 101, 1)
    assert book.domain_changed(4, 7, 100, 2)


def test_domain_mismatched_object_is_rejected_not_reassigned():
    book = _bound_book()
    wrong_epoch = ObjectEvidenceKey(5, 22, 7, 100, 1)

    assert not book.add(_evidence(key=wrong_epoch), settled_after=10.0)
    assert book.record_count == 0


def test_records_expire_and_storage_remains_bounded():
    config = EvidenceConfig(
        confirmation_snapshots=3,
        duplicate_heading_tolerance_deg=10.0,
        evidence_ttl_sec=2.0,
        maximum_records=4,
    )
    book = _bound_book(config)
    for index in range(7):
        book.add(
            _evidence(stamp=10.0 + index * 0.1, selected=False),
            settled_after=10.0,
        )

    assert book.record_count == 4
    book.expire(now_sec=13.0)
    assert book.record_count == 0


def test_invalid_evidence_config_and_values_are_rejected():
    with pytest.raises(ValueError, match='confirmation_snapshots'):
        EvidenceConfig(0, 10.0, 12.0, 40)

    book = _bound_book()
    with pytest.raises(ValueError, match='heading_deg'):
        book.add(_evidence(heading=float('nan')), settled_after=10.0)
