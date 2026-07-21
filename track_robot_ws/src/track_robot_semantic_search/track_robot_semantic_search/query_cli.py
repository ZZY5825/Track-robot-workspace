import argparse
from collections import deque
import math
import sys
import time
from typing import Optional, TextIO

from .query_portal import (
    UINT64_MAX,
    PortalDiagnostic,
    QueryIdAllocator,
    QueryRequest,
    QuerySession,
    SubmissionResult,
    parse_diagnostic,
    parse_interactive_command,
)


DEFAULT_QUERY_TOPIC = '/semantic_search/query'
DEFAULT_DIAGNOSTICS_TOPIC = '/semantic_search/perception_diagnostics'
HELP_TEXT = (
    'Enter query text or use :new TEXT, :revise TEXT, :status, :help, '
    'or :quit.')


def _positive_uint64(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            'must be a positive uint64 integer') from exc
    if parsed < 1 or parsed > UINT64_MAX:
        raise argparse.ArgumentTypeError(
            'must be an integer in [1, {}]'.format(UINT64_MAX))
    return parsed


def _positive_finite_float(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            'must be a finite positive number') from exc
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError(
            'must be a finite positive number')
    return parsed


def _non_negative_finite_float(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            'must be a finite non-negative number') from exc
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError(
            'must be a finite non-negative number')
    return parsed


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(
        description=(
            'Submit passive language queries to semantic-search perception. '
            'Omit QUERY_TEXT to open the interactive portal.'))
    root.add_argument('query_text', metavar='QUERY_TEXT', nargs='?')
    root.add_argument('--query-id', type=_positive_uint64)
    root.add_argument('--query-version', type=_positive_uint64, default=1)
    root.add_argument('--query-topic', default=DEFAULT_QUERY_TOPIC)
    root.add_argument(
        '--diagnostics-topic', default=DEFAULT_DIAGNOSTICS_TOPIC)
    root.add_argument(
        '--timeout', type=_positive_finite_float, default=5.0,
        help='seconds to wait for a correlated acknowledgment (default: 5)')
    root.add_argument(
        '--subscriber-timeout',
        type=_non_negative_finite_float,
        default=2.0,
        help='seconds to wait for a query subscriber (default: 2)')
    return root


def _write_result(
        output: TextIO,
        request: QueryRequest,
        result: SubmissionResult) -> None:
    label = result.outcome.replace('_', ' ').upper()
    output.write(
        '{} query_id={} version={} query={!r}: reason={!r}\n'.format(
            label,
            request.query_id,
            request.query_version,
            request.query_text,
            result.reason))
    diagnostic = result.diagnostic
    if (
            result.outcome == 'accepted' and diagnostic is not None and
            diagnostic.model_ready is False):
        output.write(
            'WARNING: query accepted but the semantic model is not ready.\n')
    output.flush()


def run_one_shot(
        client,
        request: QueryRequest,
        subscriber_timeout: float,
        ack_timeout: float,
        output: TextIO) -> int:
    result = client.submit(request, subscriber_timeout, ack_timeout)
    _write_result(output, request, result)
    return result.exit_code


def _write_status(output: TextIO, client, session: QuerySession) -> None:
    current = session.current
    if current is None:
        output.write('No current query.\n')
    else:
        output.write(
            'Current query_id={} version={} query={!r}.\n'.format(
                current.query_id,
                current.query_version,
                current.query_text))
    diagnostic = getattr(client, 'latest_diagnostic', None)
    if diagnostic is None:
        output.write('No semantic-search diagnostic received.\n')
    else:
        output.write(
            'Latest diagnostic state={} model_ready={} reason={!r}.\n'.format(
                diagnostic.state,
                diagnostic.model_ready,
                diagnostic.reason))
    output.flush()


def run_interactive(
        client,
        session: QuerySession,
        subscriber_timeout: float,
        ack_timeout: float,
        input_stream: TextIO,
        output: TextIO) -> int:
    output.write('Semantic Search Query Portal (passive, no robot motion)\n')
    output.write(HELP_TEXT + '\n')
    output.flush()
    while True:
        try:
            output.write('semantic-search> ')
            output.flush()
            line = input_stream.readline()
        except KeyboardInterrupt:
            output.write('\nInterrupted.\n')
            output.flush()
            return 130
        if line == '':
            output.write('\n')
            output.flush()
            return 0
        try:
            command = parse_interactive_command(line)
            if command.kind == 'quit':
                return 0
            if command.kind == 'help':
                output.write(HELP_TEXT + '\n')
                output.flush()
                continue
            if command.kind == 'status':
                _write_status(output, client, session)
                continue
            if command.kind == 'new':
                request = session.new_query(command.text)
            elif command.kind == 'revise':
                request = session.revise_query(command.text)
            else:
                raise ValueError(
                    'unsupported interactive command {}'.format(command.kind))
        except (TypeError, ValueError) as exc:
            output.write('ERROR: {!r}\n'.format(str(exc)))
            output.flush()
            continue
        run_one_shot(
            client,
            request,
            subscriber_timeout,
            ack_timeout,
            output)


class RosQueryClient:
    def __init__(
            self,
            query_topic: str = DEFAULT_QUERY_TOPIC,
            diagnostics_topic: str = DEFAULT_DIAGNOSTICS_TOPIC) -> None:
        import rclpy
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from std_msgs.msg import String

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self._rclpy = rclpy
        self._string_type = String
        self._node = rclpy.create_node('semantic_search_query_portal')
        self._publisher = self._node.create_publisher(
            String, query_topic, qos)
        self._subscription = self._node.create_subscription(
            String, diagnostics_topic, self._on_diagnostic, qos)
        self._diagnostics = deque(maxlen=32)
        self.latest_diagnostic: Optional[PortalDiagnostic] = None

    def _on_diagnostic(self, message) -> None:
        diagnostic = parse_diagnostic(message.data)
        if diagnostic is None:
            return
        self.latest_diagnostic = diagnostic
        self._diagnostics.append(diagnostic)

    def _spin_until(self, deadline: float) -> None:
        remaining = max(0.0, deadline - time.monotonic())
        self._rclpy.spin_once(
            self._node,
            timeout_sec=min(0.05, remaining))

    def submit(
            self,
            request: QueryRequest,
            subscriber_timeout: float,
            ack_timeout: float) -> SubmissionResult:
        subscriber_deadline = time.monotonic() + subscriber_timeout
        while self._publisher.get_subscription_count() < 1:
            if time.monotonic() >= subscriber_deadline:
                return SubmissionResult.no_subscriber(
                    'no query subscriber became available')
            self._spin_until(subscriber_deadline)

        self._diagnostics.clear()
        message = self._string_type()
        message.data = request.payload
        self._publisher.publish(message)

        acknowledgment_deadline = time.monotonic() + ack_timeout
        while time.monotonic() < acknowledgment_deadline:
            self._spin_until(acknowledgment_deadline)
            while self._diagnostics:
                diagnostic = self._diagnostics.popleft()
                if not diagnostic.matches(request):
                    continue
                if diagnostic.accepted:
                    return SubmissionResult.accepted(
                        diagnostic.reason, diagnostic)
                if diagnostic.rejected:
                    return SubmissionResult.rejected(
                        diagnostic.reason, diagnostic)
        return SubmissionResult.timed_out(
            'no correlated acknowledgment arrived before timeout')

    def destroy(self) -> None:
        self._node.destroy_node()


def main(args=None) -> int:
    arguments = parser().parse_args(args)
    output = sys.stdout
    session = QuerySession(
        QueryIdAllocator(initial_id=arguments.query_id),
        initial_version=arguments.query_version)

    try:
        if arguments.query_text is not None:
            request = session.new_query(arguments.query_text)
        else:
            request = None
    except (TypeError, ValueError) as exc:
        output.write('ERROR: {!r}\n'.format(str(exc)))
        output.flush()
        return 2

    client = None
    rclpy = None
    try:
        import rclpy as rclpy_module
        rclpy = rclpy_module
        rclpy.init(args=[])
        client = RosQueryClient(
            query_topic=arguments.query_topic,
            diagnostics_topic=arguments.diagnostics_topic)
        if request is not None:
            return run_one_shot(
                client,
                request,
                arguments.subscriber_timeout,
                arguments.timeout,
                output)
        return run_interactive(
            client,
            session,
            arguments.subscriber_timeout,
            arguments.timeout,
            sys.stdin,
            output)
    except KeyboardInterrupt:
        output.write('\nInterrupted.\n')
        output.flush()
        return 130
    except Exception as exc:
        output.write(
            'ERROR: ROS query portal failed: {!r}\n'.format(str(exc)))
        output.flush()
        return 1
    finally:
        if client is not None:
            client.destroy()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
