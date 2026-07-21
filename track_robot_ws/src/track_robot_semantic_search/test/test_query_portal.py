import json

import pytest

from track_robot_semantic_search.query_portal import (
    UINT64_MAX,
    InteractiveCommand,
    QueryIdAllocator,
    QueryRequest,
    QuerySession,
    SubmissionResult,
    parse_diagnostic,
    parse_interactive_command,
)


def test_request_builds_canonical_normalized_payload():
    request = QueryRequest.create('  red\u3000ｂａｇ\n ', 7, 2)

    assert request.query_text == 'red bag'
    assert request.payload == (
        '{"query_id":7,"query_text":"red bag","query_version":2}')
    assert json.loads(request.payload) == {
        'query_id': 7,
        'query_text': 'red bag',
        'query_version': 2,
    }


@pytest.mark.parametrize('text', ['', '   ', 'x' * 513])
def test_request_rejects_invalid_text(text):
    with pytest.raises(ValueError):
        QueryRequest.create(text, 1, 1)


@pytest.mark.parametrize('field', ['query_id', 'query_version'])
@pytest.mark.parametrize('value', [0, -1, True, 1.5, UINT64_MAX + 1])
def test_request_rejects_invalid_positive_uint64(field, value):
    values = {'query_id': 1, 'query_version': 1}
    values[field] = value

    with pytest.raises(ValueError, match=field):
        QueryRequest.create('target', **values)


def test_request_accepts_maximum_uint64_values():
    request = QueryRequest.create('target', UINT64_MAX, UINT64_MAX)

    assert request.query_id == UINT64_MAX
    assert request.query_version == UINT64_MAX


def test_diagnostic_parses_and_matches_only_the_same_query_key():
    request = QueryRequest.create('target', 9, 3)
    accepted = parse_diagnostic(
        '{"state":"query_accepted","reason":"ok","model_ready":true,'
        '"query_id":9,"query_version":3}')
    stale = parse_diagnostic(
        '{"state":"query_accepted","reason":"old","model_ready":true,'
        '"query_id":8,"query_version":3}')

    assert accepted is not None
    assert accepted.matches(request)
    assert accepted.accepted
    assert not accepted.rejected
    assert stale is not None
    assert not stale.matches(request)


def test_rejected_diagnostic_is_correlated_and_typed():
    request = QueryRequest.create('target', 11, 4)
    diagnostic = parse_diagnostic(
        '{"state":"query_rejected","reason":"invalid",'
        '"model_ready":false,"query_id":11,"query_version":4}')

    assert diagnostic is not None
    assert diagnostic.matches(request)
    assert diagnostic.rejected
    assert not diagnostic.accepted
    assert diagnostic.model_ready is False


def test_ready_diagnostic_without_query_key_is_available_but_uncorrelated():
    diagnostic = parse_diagnostic(
        '{"state":"ready","reason":"model loaded","model_ready":true}')

    assert diagnostic is not None
    assert diagnostic.query_id is None
    assert diagnostic.query_version is None
    assert not diagnostic.matches(QueryRequest.create('target', 1, 1))


@pytest.mark.parametrize('payload', [
    'not json',
    '[]',
    '{}',
    '{"state":1,"reason":"bad"}',
    '{"state":"ready","reason":2}',
    '{"state":"ready","reason":"ok","model_ready":"yes"}',
    '{"state":"query_accepted","reason":"ok","query_id":1}',
    '{"state":"ready","reason":"' + ('x' * 257) + '"}',
])
def test_diagnostic_parser_ignores_malformed_or_unbounded_payload(payload):
    assert parse_diagnostic(payload) is None


def test_diagnostic_parser_rejects_oversized_payload_before_accepting_fields():
    payload = json.dumps({
        'state': 'ready',
        'reason': 'ok',
        'model_ready': True,
        'padding': 'x' * 4096,
    })

    assert parse_diagnostic(payload) is None


def test_allocator_is_monotonic_when_clock_repeats_or_rolls_back():
    ticks = iter([5_000_000, 5_000_000, 4_000_000])
    allocator = QueryIdAllocator(clock_ns=lambda: next(ticks))

    assert [allocator.next_id() for _ in range(3)] == [5000, 5001, 5002]


def test_allocator_uses_explicit_first_id_then_remains_monotonic():
    allocator = QueryIdAllocator(
        clock_ns=lambda: 1_000_000,
        initial_id=42)

    assert allocator.next_id() == 42
    assert allocator.next_id() == 1000


def test_allocator_rejects_invalid_explicit_id():
    with pytest.raises(ValueError, match='query_id'):
        QueryIdAllocator(initial_id=0)


def test_allocator_rejects_uint64_exhaustion():
    allocator = QueryIdAllocator(initial_id=UINT64_MAX)
    assert allocator.next_id() == UINT64_MAX

    with pytest.raises(ValueError, match='query_id'):
        allocator.next_id()


def test_session_new_and_revise_transitions():
    session = QuerySession(
        QueryIdAllocator(clock_ns=lambda: 20_000_000))

    first = session.new_query('red bag')
    revised = session.revise_query('black bag')
    second = session.new_query('box')

    assert (first.query_id, first.query_version) == (20000, 1)
    assert (revised.query_id, revised.query_version) == (20000, 2)
    assert (second.query_id, second.query_version) == (20001, 1)
    assert session.current == second


def test_session_applies_explicit_initial_version_only_to_first_query():
    session = QuerySession(
        QueryIdAllocator(clock_ns=lambda: 1_000, initial_id=10),
        initial_version=7)

    first = session.new_query('first')
    second = session.new_query('second')

    assert (first.query_id, first.query_version) == (10, 7)
    assert (second.query_id, second.query_version) == (11, 1)


def test_session_revise_requires_current_query():
    session = QuerySession(QueryIdAllocator(initial_id=1))

    with pytest.raises(ValueError, match='current query'):
        session.revise_query('target')


def test_session_revise_rejects_version_overflow():
    session = QuerySession(
        QueryIdAllocator(initial_id=1),
        initial_version=UINT64_MAX)
    session.new_query('target')

    with pytest.raises(ValueError, match='query_version'):
        session.revise_query('revised target')


@pytest.mark.parametrize('line,expected', [
    ('red bag', InteractiveCommand('new', 'red bag')),
    (':new red bag', InteractiveCommand('new', 'red bag')),
    (':revise black bag', InteractiveCommand('revise', 'black bag')),
    (':status', InteractiveCommand('status')),
    (':help', InteractiveCommand('help')),
    (':quit', InteractiveCommand('quit')),
])
def test_interactive_command_parser(line, expected):
    assert parse_interactive_command(line) == expected


@pytest.mark.parametrize('line,reason', [
    ('', 'empty'),
    ('   ', 'empty'),
    (':new', 'requires query text'),
    (':revise', 'requires query text'),
    (':status extra', 'takes no text'),
    (':unknown target', 'unknown command'),
])
def test_interactive_command_parser_rejects_invalid_commands(line, reason):
    with pytest.raises(ValueError, match=reason):
        parse_interactive_command(line)


@pytest.mark.parametrize('factory,outcome,exit_code', [
    (SubmissionResult.accepted, 'accepted', 0),
    (SubmissionResult.rejected, 'rejected', 2),
    (SubmissionResult.no_subscriber, 'no_subscriber', 3),
    (SubmissionResult.timed_out, 'timeout', 4),
])
def test_submission_result_factories_have_deterministic_exit_codes(
        factory, outcome, exit_code):
    result = factory('reason')

    assert result.outcome == outcome
    assert result.reason == 'reason'
    assert result.exit_code == exit_code
