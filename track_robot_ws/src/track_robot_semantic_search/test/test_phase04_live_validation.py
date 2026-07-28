from types import SimpleNamespace

from track_robot_semantic_search.phase04_live_validation import (
    Phase04LiveEvidence,
    build_live_report,
)


def ns(**values):
    return SimpleNamespace(**values)


def stamp(nanoseconds):
    return ns(
        sec=nanoseconds // 1_000_000_000,
        nanosec=nanoseconds % 1_000_000_000)


def header(nanoseconds, frame='base_link'):
    return ns(stamp=stamp(nanoseconds), frame_id=frame)


def semantic_object():
    return ns(
        global_object_id=42,
        localization_epoch_id=7,
        active_query_id=1234,
        active_query_version=2,
        position_frame_id='base_link',
        position_valid=True,
        task_relevance=0.82,
        uncertainty=0.18,
        lifecycle_state=1,
        last_seen=stamp(9_900_000_000),
    )


def test_complete_chain_passes_with_consistent_references():
    evidence = Phase04LiveEvidence(
        expected_query_id=1234,
        expected_query_version=2,
        collection_start_ns=10_000_000_000,
    )
    evidence.localization(ns(
        header=header(9_950_000_000),
        localization_epoch_id=7,
        local_healthy=True,
        canonical_frame_id='base_link',
        reason='healthy'))
    evidence.regions(ns(
        header=header(9_960_000_000, 'zed_left_camera_optical_frame'),
        query_id=1234,
        query_version=2,
        regions=[ns(
            query_id=1234,
            query_version=2,
            fused_score=0.75,
            header=header(
                9_960_000_000,
                'zed_left_camera_optical_frame'))]))
    evidence.observations(ns(
        header=header(9_965_000_000, 'zed_left_camera_optical_frame'),
        observations=[ns(
            query_id=1234,
            query_version=2,
            position_valid=True,
            position_frame_id='base_link',
            localization_epoch_id=7)]))
    for sequence in range(3):
        evidence.active_objects(ns(
            header=header(9_970_000_000 + sequence),
            memory_epoch_id=11,
            snapshot_sequence=sequence + 1,
            objects=[semantic_object()]))
    evidence.best_candidate(ns(
        header=header(9_980_000_000),
        memory_epoch_id=11,
        snapshot_sequence=1,
        objects=[semantic_object()]))
    evidence.costmap(ns(
        header=header(9_985_000_000),
        info=ns(width=80, height=80, resolution=0.1)))
    evidence.phase4_diagnostics(ns(status=[ns(
        message='planned',
        values=[
            ns(key='status', value='PASS'),
            ns(key='reason', value='planned'),
            ns(key='latency_ms', value='3.5'),
            ns(key='memory_epoch_id', value='11'),
            ns(key='global_object_id', value='42'),
            ns(key='localization_epoch_id', value='7'),
            ns(key='query_id', value='1234'),
            ns(key='query_version', value='2'),
        ])]))
    evidence.path(ns(
        header=header(9_990_000_000),
        poses=[ns(), ns(), ns()]))

    report = build_live_report(
        evidence, query_text='green bottle', duration_sec=20.0)

    assert [report['phases'][name]['status'] for name in (
        'phase0', 'phase1', 'phase2', 'phase3', 'phase4')] == [
            'PASS', 'PASS', 'PASS', 'PASS', 'PASS']
    assert report['cross_phase_consistency']['status'] == 'PASS'
    assert report['cross_phase_consistency']['memory_epoch_ids'] == [11]
    assert report['cross_phase_consistency']['global_object_ids'] == [42]
    assert report['cross_phase_consistency']['localization_epoch_ids'] == [7]
    assert report['cross_phase_consistency']['query_ids'] == [1234]
    assert report['safety']['planning_only'] is True


def test_phase2_target_stability_ignores_nonquery_background_objects():
    evidence = Phase04LiveEvidence(
        expected_query_id=1234,
        expected_query_version=2,
        collection_start_ns=10_000_000_000,
    )
    background = semantic_object()
    background.global_object_id = 1
    background.active_query_id = 0
    background.active_query_version = 0
    target = semantic_object()
    evidence.active_objects(ns(
        header=header(9_970_000_000),
        memory_epoch_id=11,
        snapshot_sequence=1,
        objects=[background, target],
    ))

    report = build_live_report(
        evidence, query_text='green bottle', duration_sec=1.0)

    phase2 = report['phases']['phase2']
    assert phase2['evidence']['global_object_ids'] == [42]
    assert phase2['evidence']['query_ids'] == [1234]


def test_missing_streams_are_not_evaluated():
    report = build_live_report(
        Phase04LiveEvidence(1234, 2, 10_000_000_000),
        query_text='green bottle',
        duration_sec=20.0)

    assert all(
        phase['status'] == 'NOT EVALUATED'
        for phase in report['phases'].values())


def test_empty_phase3_selection_and_no_target_phase4_are_failures():
    evidence = Phase04LiveEvidence(1234, 2, 10_000_000_000)
    evidence.best_candidate(ns(
        header=header(9_980_000_000),
        memory_epoch_id=11,
        snapshot_sequence=1,
        objects=[]))
    evidence.phase4_diagnostics(ns(status=[ns(
        message='no_target',
        values=[
            ns(key='status', value='FAIL'),
            ns(key='reason', value='no_target'),
            ns(key='latency_ms', value='0.1'),
        ])]))

    report = build_live_report(
        evidence, query_text='green bottle', duration_sec=20.0)

    assert report['phases']['phase3']['status'] == 'FAIL'
    assert 'no selected target' in report['phases']['phase3']['failures']
    assert report['phases']['phase4']['status'] == 'FAIL'
    assert 'no_target' in report['phases']['phase4']['failures']
