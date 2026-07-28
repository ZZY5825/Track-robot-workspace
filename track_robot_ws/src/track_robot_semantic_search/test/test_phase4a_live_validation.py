from types import SimpleNamespace

from track_robot_semantic_search.phase04_live_validation import (
    Phase04LiveEvidence,
    build_live_report,
)
from track_robot_semantic_search.phase4a_live_validation import (
    PHASE4A_LOCALIZATION_TOPIC,
    PHASE4A_SELECTED_TARGET_TOPIC,
    PHASE4A_SPATIAL_OBJECTS_TOPIC,
    parser,
)


def ns(**values):
    return SimpleNamespace(**values)


def test_phase4a_report_requires_correlated_ready_advice():
    evidence = Phase04LiveEvidence(2026072801, 1, 10_000_000_000)
    evidence.advice(ns(data=(
        'READY target="green bottle" position=front 1.60m,right 0.20m '
        'ADVISORY_ONLY')))
    evidence.phase4a_diagnostics(ns(status=[ns(
        message='ready',
        values=[
            ns(key='status', value='READY'),
            ns(key='reason', value='ready'),
            ns(key='memory_epoch_id', value='11'),
            ns(key='global_object_id', value='42'),
            ns(key='localization_epoch_id', value='7'),
            ns(key='query_id', value='2026072801'),
            ns(key='query_version', value='1'),
        ])]))

    report = build_live_report(
        evidence,
        query_text='green bottle',
        duration_sec=20.0,
        require_advisory=True)

    assert report['phases']['phase4a_advisory']['status'] == 'PASS'
    assert report['phases']['phase4a_advisory']['evidence'][
        'latest_advice'].startswith('READY ')
    assert report['safety']['advisory_only'] is True


def test_phase4a_report_rejects_not_ready_only_output():
    evidence = Phase04LiveEvidence(2026072801, 1, 10_000_000_000)
    evidence.advice(ns(data='NOT_READY reason=no_target ADVISORY_ONLY'))
    evidence.phase4a_diagnostics(ns(status=[ns(
        message='no_target',
        values=[
            ns(key='status', value='NOT_READY'),
            ns(key='reason', value='no_target'),
        ])]))

    report = build_live_report(
        evidence,
        query_text='green bottle',
        duration_sec=20.0,
        require_advisory=True)

    phase = report['phases']['phase4a_advisory']
    assert phase['status'] == 'FAIL'
    assert 'no_target' in phase['failures']


def test_phase4a_cli_hard_codes_stationary_bridge_topics():
    arguments = parser().parse_args([
        '--query', 'green bottle',
        '--query-id', '2026072801',
        '--output', '/tmp/report.json',
    ])

    assert arguments.localization_topic == PHASE4A_LOCALIZATION_TOPIC
    assert arguments.active_objects_topic == PHASE4A_SPATIAL_OBJECTS_TOPIC
    assert arguments.selected_target_topic == PHASE4A_SELECTED_TARGET_TOPIC
    assert arguments.query_version == 1


def test_phase2_report_preserves_runtime_association_diagnostics():
    evidence = Phase04LiveEvidence(2026072801, 1, 10_000_000_000)
    evidence.memory_diagnostics(ns(status=[ns(
        message='running',
        values=[
            ns(key='lidar_messages', value='42'),
            ns(key='rejected_lidar_batches', value='0'),
            ns(key='association_debug_pairs', value='17'),
            ns(key='accepted_camera_attachments', value='3'),
            ns(key='lidar_reason', value='updated'),
            ns(key='association_reason',
               value='runtime_attachment_evaluated'),
        ])]))

    report = build_live_report(
        evidence,
        query_text='green bottle',
        duration_sec=2.0,
        require_advisory=True)

    diagnostics = report['phases']['phase2']['evidence'][
        'latest_runtime_diagnostics']
    assert diagnostics['lidar_messages'] == '42'
    assert diagnostics['accepted_camera_attachments'] == '3'
    assert diagnostics['association_reason'] == (
        'runtime_attachment_evaluated')


def test_observation_contract_probe_names_out_of_bounds_roi():
    evidence = Phase04LiveEvidence(2026072801, 1, 10_000_000_000)
    evidence.observations(ns(
        header=ns(stamp=ns(sec=10, nanosec=0),
                  frame_id='zed_left_camera_optical_frame'),
        producer_epoch_id=3,
        observations=[ns(
            producer_epoch_id=3,
            visual_candidate_id=9,
            camera_stamp_valid=True,
            image_width=1280,
            image_height=720,
            roi=ns(
                x_offset=1270,
                y_offset=10,
                width=40,
                height=80),
            query_id=2026072801,
            query_version=1,
            localization_epoch_id=0,
            position_valid=False,
        )]))

    report = build_live_report(
        evidence,
        query_text='green bottle',
        duration_sec=2.0,
        require_advisory=True)

    assert report['phases']['phase1']['evidence'][
        'observation_contract_failures'] == {'roi_out_of_bounds': 1}


def test_phase2_report_measures_nearest_camera_lidar_source_delta():
    evidence = Phase04LiveEvidence(2026072801, 1, 10_000_000_000)
    evidence.observation_stamps.extend([
        1_000_000_000,
        2_000_000_000,
    ])
    evidence.lidar_tracklets(ns(
        header=ns(stamp=ns(sec=1, nanosec=30_000_000))))
    evidence.lidar_tracklets(ns(
        header=ns(stamp=ns(sec=2, nanosec=70_000_000))))

    report = build_live_report(
        evidence,
        query_text='green bottle',
        duration_sec=2.0,
        require_advisory=True)
    timing = report['phases']['phase2']['evidence'][
        'camera_lidar_nearest_delta_ms']

    assert timing['sample_count'] == 2
    assert timing['minimum'] == 30.0
    assert timing['maximum'] == 70.0


def test_phase2_report_measures_online_prior_lidar_delta():
    evidence = Phase04LiveEvidence(2026072801, 1, 10_000_000_000)
    evidence.lidar_tracklets(ns(
        header=ns(stamp=ns(sec=1, nanosec=0))))
    evidence.observations(ns(
        header=ns(
            stamp=ns(sec=1, nanosec=150_000_000),
            frame_id='zed_left_camera_optical_frame'),
        producer_epoch_id=3,
        observations=[]))

    report = build_live_report(
        evidence,
        query_text='green bottle',
        duration_sec=2.0,
        require_advisory=True)
    timing = report['phases']['phase2']['evidence'][
        'camera_to_available_lidar_delta_ms']

    assert timing['sample_count'] == 1
    assert timing['p50'] == 150.0
