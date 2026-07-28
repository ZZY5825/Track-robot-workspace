import math

import pytest


MISSING = object()


def report(
        candidate_id, dataset_id='grounding-r0', dataset_checksum='a' * 64,
        recall=0.90,
        false_accept=0.02, median_iou=0.60, p95_ms=100.0,
        semantic_rate_hz=10.0, peak_mib=1024.0, **overrides):
    value = {
        'dataset_id': dataset_id,
        'dataset_checksum': dataset_checksum,
        'candidate_id': candidate_id,
        'validation_selection': {
            'status': 'selected',
            'threshold': 0.70,
        },
        'runtime_available': True,
        'platform_compatible': True,
        'licence_approved': True,
        'human_reviewed_test_labels': True,
        'test_metrics': {
            'top1_recall_iou_50': recall,
            'target_absent_false_accept_rate': false_accept,
            'median_accepted_positive_iou': median_iou,
        },
        'resources': {
            'p95_complete_path_ms': p95_ms,
            'semantic_rate_hz': semantic_rate_hz,
            'incremental_cuda_reserved_mib': peak_mib,
        },
    }
    value.update(overrides)
    return value


def set_path(value, path, replacement):
    parent = value
    for key in path[:-1]:
        parent = parent[key]
    if replacement is MISSING:
        del parent[path[-1]]
    else:
        parent[path[-1]] = replacement
    return value


def test_selects_highest_recall_candidate_that_passes_every_gate():
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    result = select_grounding_candidate([
        report(
            'fast', recall=0.86, false_accept=0.04,
            median_iou=0.60, p95_ms=70.0, peak_mib=800.0),
        report(
            'accurate', recall=0.93, false_accept=0.03,
            median_iou=0.64, p95_ms=130.0, peak_mib=1200.0),
        report(
            'too_slow', recall=0.99, false_accept=0.01,
            median_iou=0.80, p95_ms=151.0, peak_mib=900.0),
    ])

    assert result.status == 'selected'
    assert result.selected_candidate_id == 'accurate'
    assert result.ranking == ('accurate', 'fast')
    assert result.rejected == {
        'too_slow': ('latency_p95_at_most_150_ms',),
    }


def test_ranking_uses_every_tie_break_in_declared_order():
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    result = select_grounding_candidate([
        report(
            'lexical-z', recall=0.90, false_accept=0.02,
            median_iou=0.60, p95_ms=90.0),
        report(
            'lower-latency', recall=0.90, false_accept=0.02,
            median_iou=0.60, p95_ms=80.0),
        report(
            'higher-iou', recall=0.90, false_accept=0.02,
            median_iou=0.70, p95_ms=120.0),
        report(
            'lower-false-accept', recall=0.90, false_accept=0.01,
            median_iou=0.55, p95_ms=140.0),
        report(
            'higher-recall', recall=0.91, false_accept=0.04,
            median_iou=0.51, p95_ms=149.0),
        report(
            'lexical-a', recall=0.90, false_accept=0.02,
            median_iou=0.60, p95_ms=90.0),
    ])

    assert result.ranking == (
        'higher-recall',
        'lower-false-accept',
        'higher-iou',
        'lower-latency',
        'lexical-a',
        'lexical-z',
    )
    assert result.selected_candidate_id == 'higher-recall'


def test_ranking_is_independent_of_report_input_order():
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    reports = [
        report('c', recall=0.90),
        report('a', recall=0.90),
        report('b', recall=0.90),
    ]

    assert select_grounding_candidate(reports).ranking == ('a', 'b', 'c')
    assert select_grounding_candidate(reversed(reports)).ranking == (
        'a', 'b', 'c')


def test_no_passing_candidate_is_explicitly_unavailable():
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    result = select_grounding_candidate([
        report('candidate', runtime_available=False),
    ])

    assert result.status == 'unavailable'
    assert result.selected_candidate_id is None
    assert result.ranking == ()
    assert result.rejected == {'candidate': ('runtime_available',)}


def test_empty_report_set_is_explicitly_unavailable():
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    result = select_grounding_candidate([])

    assert result.status == 'unavailable'
    assert result.selected_candidate_id is None
    assert result.rejected == {}
    assert result.ranking == ()


def test_ranking_is_an_immutable_tuple():
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    result = select_grounding_candidate([report('candidate')])

    assert result.ranking == ('candidate',)
    with pytest.raises(AttributeError):
        result.ranking.append('mutated')


def test_rejected_mapping_disallows_item_assignment_and_clear():
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    result = select_grounding_candidate([
        report('candidate', runtime_available=False),
    ])

    with pytest.raises(TypeError):
        result.rejected['other'] = ('runtime_available',)
    with pytest.raises(AttributeError):
        result.rejected.clear()


def test_rejection_reasons_are_immutable_tuples():
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    result = select_grounding_candidate([
        report('candidate', runtime_available=False),
    ])

    assert result.rejected['candidate'] == ('runtime_available',)
    with pytest.raises(AttributeError):
        result.rejected['candidate'].append('mutated')


@pytest.mark.parametrize('path,value,reason', [
    (('runtime_available',), False, 'runtime_available'),
    (('platform_compatible',), False, 'platform_compatible'),
    (('licence_approved',), False, 'licence_approved'),
    (
        ('human_reviewed_test_labels',), False,
        'human_reviewed_test_labels',
    ),
    (
        ('test_metrics', 'top1_recall_iou_50'), 0.849,
        'top1_recall_iou_50_at_least_0_85',
    ),
    (
        ('test_metrics', 'target_absent_false_accept_rate'), 0.051,
        'false_accept_rate_at_most_0_05',
    ),
    (
        ('test_metrics', 'median_accepted_positive_iou'), 0.499,
        'median_iou_at_least_0_50',
    ),
    (
        ('resources', 'p95_complete_path_ms'), 150.001,
        'latency_p95_at_most_150_ms',
    ),
    (
        ('resources', 'semantic_rate_hz'), 4.999,
        'semantic_rate_at_least_5_hz',
    ),
    (
        ('resources', 'incremental_cuda_reserved_mib'), 1536.001,
        'incremental_cuda_at_most_1536_mib',
    ),
])
def test_each_hard_gate_rejects_with_its_exact_reason(path, value, reason):
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    candidate = set_path(report('failed'), path, value)

    result = select_grounding_candidate([candidate])

    assert result.rejected == {'failed': (reason,)}


def test_threshold_boundaries_pass():
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    result = select_grounding_candidate([
        report(
            'boundary', recall=0.85, false_accept=0.05,
            median_iou=0.50, p95_ms=150.0, semantic_rate_hz=5.0,
            peak_mib=1536.0),
    ])

    assert result.status == 'selected'
    assert result.selected_candidate_id == 'boundary'
    assert result.rejected == {}


def test_returns_every_rejection_reason_in_gate_order():
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    candidate = report(
        'all-failed',
        recall=0.1,
        false_accept=0.9,
        median_iou=0.1,
        p95_ms=999.0,
        semantic_rate_hz=1.0,
        peak_mib=9999.0,
        runtime_available=False,
        platform_compatible=False,
        licence_approved=False,
        human_reviewed_test_labels=False,
    )

    result = select_grounding_candidate([candidate])

    assert result.rejected['all-failed'] == (
        'runtime_available',
        'platform_compatible',
        'licence_approved',
        'human_reviewed_test_labels',
        'top1_recall_iou_50_at_least_0_85',
        'false_accept_rate_at_most_0_05',
        'median_iou_at_least_0_50',
        'latency_p95_at_most_150_ms',
        'semantic_rate_at_least_5_hz',
        'incremental_cuda_at_most_1536_mib',
    )


@pytest.mark.parametrize('path', [
    ('runtime_available',),
    ('platform_compatible',),
    ('licence_approved',),
    ('human_reviewed_test_labels',),
])
@pytest.mark.parametrize('value', [MISSING, None, 1, 0, 'yes'])
def test_boolean_gates_fail_closed_for_missing_or_wrongly_typed_values(
        path, value):
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    candidate = set_path(report('bad-boolean'), path, value)

    result = select_grounding_candidate([candidate])

    assert result.status == 'unavailable'
    assert result.rejected['bad-boolean']


NUMERIC_PATHS = [
    ('test_metrics', 'top1_recall_iou_50'),
    ('test_metrics', 'target_absent_false_accept_rate'),
    ('test_metrics', 'median_accepted_positive_iou'),
    ('resources', 'p95_complete_path_ms'),
    ('resources', 'semantic_rate_hz'),
    ('resources', 'incremental_cuda_reserved_mib'),
]


@pytest.mark.parametrize('path', NUMERIC_PATHS)
@pytest.mark.parametrize(
    'value', [MISSING, None, True, False, '1.0', math.nan, math.inf, -math.inf])
def test_numeric_gates_fail_closed_without_exceptions(path, value):
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    candidate = set_path(report('bad-number'), path, value)

    result = select_grounding_candidate([candidate])

    assert result.status == 'unavailable'
    assert result.rejected['bad-number']
    assert result.ranking == ()


@pytest.mark.parametrize('path,value', [
    (('test_metrics', 'top1_recall_iou_50'), -0.1),
    (('test_metrics', 'top1_recall_iou_50'), 1.1),
    (('test_metrics', 'target_absent_false_accept_rate'), -0.1),
    (('test_metrics', 'target_absent_false_accept_rate'), 1.1),
    (('test_metrics', 'median_accepted_positive_iou'), -0.1),
    (('test_metrics', 'median_accepted_positive_iou'), 1.1),
    (('resources', 'p95_complete_path_ms'), -0.1),
    (('resources', 'semantic_rate_hz'), -0.1),
    (('resources', 'incremental_cuda_reserved_mib'), -0.1),
])
def test_semantically_invalid_numeric_ranges_cannot_pass(path, value):
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    result = select_grounding_candidate([
        set_path(report('out-of-range'), path, value),
    ])

    assert result.status == 'unavailable'
    assert result.rejected['out-of-range']


@pytest.mark.parametrize('field', ['test_metrics', 'resources'])
@pytest.mark.parametrize('value', [MISSING, None, [], 'invalid'])
def test_missing_or_malformed_nested_sections_reject_without_exceptions(
        field, value):
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    candidate = set_path(report('bad-section'), (field,), value)

    result = select_grounding_candidate([candidate])

    assert result.status == 'unavailable'
    assert result.rejected['bad-section']


def test_duplicate_candidate_ids_are_rejected():
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    with pytest.raises(ValueError, match='duplicate candidate_id'):
        select_grounding_candidate([
            report('same'),
            report('same'),
        ])


def test_mixed_dataset_ids_are_rejected():
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    with pytest.raises(ValueError, match='dataset_id'):
        select_grounding_candidate([
            report('first', dataset_id='dataset-a'),
            report('second', dataset_id='dataset-b'),
        ])


def test_mixed_dataset_checksums_are_rejected_even_when_ids_match():
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    with pytest.raises(ValueError, match='dataset_checksum'):
        select_grounding_candidate([
            report('first', dataset_checksum='a' * 64),
            report('second', dataset_checksum='b' * 64),
        ])


@pytest.mark.parametrize('checksum', [
    MISSING,
    None,
    True,
    1,
    '',
    'a' * 63,
    'A' * 64,
    'g' * 64,
])
def test_dataset_checksum_must_be_lowercase_64_hex(checksum):
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    candidate = set_path(
        report('candidate'), ('dataset_checksum',), checksum)

    with pytest.raises(ValueError, match='dataset_checksum'):
        select_grounding_candidate([candidate])


@pytest.mark.parametrize('selection', [
    MISSING,
    None,
    [],
    'selected',
    {},
    {'status': 'selected'},
    {'threshold': 0.5},
    {'status': 'unavailable', 'threshold': None},
    {'status': True, 'threshold': 0.5},
    {'status': 'selected', 'threshold': None},
    {'status': 'selected', 'threshold': True},
    {'status': 'selected', 'threshold': False},
    {'status': 'selected', 'threshold': '0.5'},
    {'status': 'selected', 'threshold': math.nan},
    {'status': 'selected', 'threshold': math.inf},
    {'status': 'selected', 'threshold': -math.inf},
    {'status': 'selected', 'threshold': -0.1},
    {'status': 'selected', 'threshold': 1.1},
])
def test_validation_selection_fails_closed_without_exceptions(selection):
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    candidate = set_path(
        report('candidate'), ('validation_selection',), selection)

    result = select_grounding_candidate([candidate])

    assert result.status == 'unavailable'
    assert result.rejected == {
        'candidate': ('validation_threshold_selected',),
    }


@pytest.mark.parametrize('threshold', [0, 0.0, 1, 1.0])
def test_validation_threshold_boundaries_are_selected(threshold):
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    candidate = report(
        'candidate',
        validation_selection={
            'status': 'selected',
            'threshold': threshold,
        },
    )

    result = select_grounding_candidate([candidate])

    assert result.status == 'selected'
    assert result.rejected == {}


@pytest.mark.parametrize('field,value', [
    ('candidate_id', MISSING),
    ('candidate_id', None),
    ('candidate_id', ''),
    ('dataset_id', MISSING),
    ('dataset_id', None),
    ('dataset_id', ''),
])
def test_identity_fields_must_be_nonempty_strings(field, value):
    from track_robot_semantic_search.grounding_selection import (
        select_grounding_candidate,
    )

    candidate = set_path(report('candidate'), (field,), value)

    with pytest.raises(ValueError, match=field):
        select_grounding_candidate([candidate])
