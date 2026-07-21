import io

import pytest

from track_robot_semantic_search.query_cli import (
    HELP_TEXT,
    parser,
    run_interactive,
    run_one_shot,
)
from track_robot_semantic_search.query_portal import (
    PortalDiagnostic,
    QueryIdAllocator,
    QueryRequest,
    QuerySession,
    SubmissionResult,
)


class FakeClient:
    def __init__(self, *results):
        self._results = list(results)
        self.requests = []
        self.calls = []
        self.latest_diagnostic = None

    def submit(self, request, subscriber_timeout, ack_timeout):
        self.requests.append(request)
        self.calls.append((subscriber_timeout, ack_timeout))
        if not self._results:
            raise AssertionError('fake client has no result')
        result = self._results.pop(0)
        self.latest_diagnostic = result.diagnostic
        return result


def accepted_result(reason='semantic query activated', model_ready=True):
    diagnostic = PortalDiagnostic(
        state='query_accepted',
        reason=reason,
        model_ready=model_ready,
        query_id=10,
        query_version=1)
    return SubmissionResult.accepted(reason, diagnostic)


def test_parser_accepts_one_shot_query_and_defaults():
    arguments = parser().parse_args(['a red backpack'])

    assert arguments.query_text == 'a red backpack'
    assert arguments.query_id is None
    assert arguments.query_version == 1
    assert arguments.query_topic == '/semantic_search/query'
    assert arguments.diagnostics_topic == (
        '/semantic_search/perception_diagnostics')
    assert arguments.timeout == 5.0
    assert arguments.subscriber_timeout == 2.0


def test_parser_accepts_explicit_transport_options():
    arguments = parser().parse_args([
        'target',
        '--query-id', '9',
        '--query-version', '4',
        '--query-topic', '/custom/query',
        '--diagnostics-topic', '/custom/diagnostics',
        '--timeout', '1.5',
        '--subscriber-timeout', '0',
    ])

    assert arguments.query_id == 9
    assert arguments.query_version == 4
    assert arguments.query_topic == '/custom/query'
    assert arguments.diagnostics_topic == '/custom/diagnostics'
    assert arguments.timeout == 1.5
    assert arguments.subscriber_timeout == 0.0


@pytest.mark.parametrize('option,value', [
    ('--query-id', '0'),
    ('--query-version', '-1'),
    ('--timeout', '0'),
    ('--timeout', 'nan'),
    ('--subscriber-timeout', '-0.1'),
    ('--subscriber-timeout', 'inf'),
])
def test_parser_rejects_invalid_bounded_options(option, value):
    with pytest.raises(SystemExit) as caught:
        parser().parse_args(['target', option, value])

    assert caught.value.code == 2


@pytest.mark.parametrize('result,exit_code,label', [
    (accepted_result(), 0, 'accepted'),
    (SubmissionResult.rejected('invalid query'), 2, 'rejected'),
    (SubmissionResult.no_subscriber('no subscriber'), 3, 'no subscriber'),
    (SubmissionResult.timed_out('acknowledgment timeout'), 4, 'timeout'),
])
def test_one_shot_prints_result_and_returns_exit_code(
        result, exit_code, label):
    client = FakeClient(result)
    output = io.StringIO()
    request = QueryRequest.create('target', 10, 1)

    code = run_one_shot(client, request, 2.0, 5.0, output)

    assert code == exit_code
    assert client.requests == [request]
    assert client.calls == [(2.0, 5.0)]
    assert 'query_id=10' in output.getvalue()
    assert 'version=1' in output.getvalue()
    assert label in output.getvalue().lower()


def test_one_shot_warns_when_query_is_accepted_but_model_is_unavailable():
    client = FakeClient(accepted_result(model_ready=False))
    output = io.StringIO()

    code = run_one_shot(
        client, QueryRequest.create('target', 10, 1), 2.0, 5.0, output)

    assert code == 0
    assert 'warning' in output.getvalue().lower()
    assert 'model' in output.getvalue().lower()


def test_one_shot_escapes_diagnostic_control_characters():
    client = FakeClient(SubmissionResult.rejected('\x1b[31mrejected'))
    output = io.StringIO()

    run_one_shot(
        client, QueryRequest.create('target', 10, 1), 2.0, 5.0, output)

    assert '\x1b' not in output.getvalue()
    assert '\\x1b' in output.getvalue()


def test_interactive_new_revise_status_and_quit():
    source = io.StringIO(
        'red bag\n:revise black bag\n:status\n:quit\n')
    output = io.StringIO()
    client = FakeClient(accepted_result(), accepted_result())
    session = QuerySession(
        QueryIdAllocator(clock_ns=lambda: 30_000_000))

    code = run_interactive(
        client, session, 2.0, 5.0, source, output)

    assert code == 0
    assert [(r.query_id, r.query_version) for r in client.requests] == [
        (30000, 1), (30000, 2)]
    assert client.calls == [(2.0, 5.0), (2.0, 5.0)]
    assert 'black bag' in output.getvalue()
    assert 'current' in output.getvalue().lower()


def test_interactive_explicit_new_allocates_new_id():
    source = io.StringIO(':new first\n:new second\n:quit\n')
    output = io.StringIO()
    client = FakeClient(accepted_result(), accepted_result())
    session = QuerySession(
        QueryIdAllocator(clock_ns=lambda: 40_000_000))

    assert run_interactive(
        client, session, 0.0, 1.0, source, output) == 0
    assert [(r.query_id, r.query_version) for r in client.requests] == [
        (40000, 1), (40001, 1)]


def test_interactive_reports_local_errors_without_publication():
    source = io.StringIO(
        ':revise target\n:unknown target\n:new\n:status\n:quit\n')
    output = io.StringIO()
    client = FakeClient()
    session = QuerySession(QueryIdAllocator(initial_id=1))

    assert run_interactive(
        client, session, 2.0, 5.0, source, output) == 0
    assert client.requests == []
    assert output.getvalue().lower().count('error') == 3
    assert 'no current query' in output.getvalue().lower()


def test_interactive_continues_after_submission_timeout():
    source = io.StringIO('first\nsecond\n:quit\n')
    output = io.StringIO()
    client = FakeClient(
        SubmissionResult.timed_out('timeout'),
        accepted_result())
    session = QuerySession(QueryIdAllocator(clock_ns=lambda: 50_000_000))

    assert run_interactive(
        client, session, 2.0, 5.0, source, output) == 0
    assert len(client.requests) == 2
    assert 'timeout' in output.getvalue().lower()
    assert 'accepted' in output.getvalue().lower()


def test_interactive_help_and_eof_exit_cleanly():
    source = io.StringIO(':help\n')
    output = io.StringIO()
    client = FakeClient()
    session = QuerySession(QueryIdAllocator(initial_id=1))

    assert run_interactive(
        client, session, 2.0, 5.0, source, output) == 0
    assert HELP_TEXT in output.getvalue()
    assert client.requests == []


def test_interactive_keyboard_interrupt_returns_130():
    class InterruptedInput:
        def readline(self):
            raise KeyboardInterrupt

    output = io.StringIO()
    client = FakeClient()
    session = QuerySession(QueryIdAllocator(initial_id=1))

    assert run_interactive(
        client, session, 2.0, 5.0, InterruptedInput(), output) == 130
    assert 'interrupted' in output.getvalue().lower()
