from dataclasses import dataclass
import json
import time
from typing import Callable, Optional

from .query import normalize_query


UINT64_MAX = 2 ** 64 - 1
MAX_QUERY_TEXT_LENGTH = 512
MAX_DIAGNOSTIC_PAYLOAD_LENGTH = 4096
MAX_DIAGNOSTIC_STATE_LENGTH = 64
MAX_DIAGNOSTIC_REASON_LENGTH = 256


def _require_positive_uint64(value: int, field: str) -> None:
    if (
            not isinstance(value, int) or isinstance(value, bool) or
            value < 1 or value > UINT64_MAX):
        raise ValueError(
            '{} must be an integer in [1, {}]'.format(field, UINT64_MAX))


@dataclass(frozen=True)
class QueryRequest:
    query_text: str
    query_id: int
    query_version: int

    @classmethod
    def create(
            cls,
            text: str,
            query_id: int,
            query_version: int) -> 'QueryRequest':
        normalized = normalize_query(text)
        if len(normalized) > MAX_QUERY_TEXT_LENGTH:
            raise ValueError('query text exceeds 512 characters')
        _require_positive_uint64(query_id, 'query_id')
        _require_positive_uint64(query_version, 'query_version')
        return cls(normalized, query_id, query_version)

    @property
    def payload(self) -> str:
        return json.dumps(
            {
                'query_id': self.query_id,
                'query_text': self.query_text,
                'query_version': self.query_version,
            },
            ensure_ascii=False,
            separators=(',', ':'),
            sort_keys=True)


@dataclass(frozen=True)
class PortalDiagnostic:
    state: str
    reason: str
    model_ready: Optional[bool]
    query_id: Optional[int]
    query_version: Optional[int]

    @property
    def accepted(self) -> bool:
        return self.state == 'query_accepted'

    @property
    def rejected(self) -> bool:
        return self.state == 'query_rejected'

    def matches(self, request: QueryRequest) -> bool:
        return (
            self.query_id is not None and
            self.query_version is not None and
            (self.query_id, self.query_version) == (
                request.query_id, request.query_version))


def parse_diagnostic(payload: str) -> Optional[PortalDiagnostic]:
    if (
            not isinstance(payload, str) or
            len(payload) > MAX_DIAGNOSTIC_PAYLOAD_LENGTH):
        return None
    try:
        value = json.loads(payload)
    except (TypeError, json.JSONDecodeError):
        return None
    if not isinstance(value, dict):
        return None

    state = value.get('state')
    reason = value.get('reason')
    if (
            not isinstance(state, str) or not state or
            len(state) > MAX_DIAGNOSTIC_STATE_LENGTH or
            not isinstance(reason, str) or
            len(reason) > MAX_DIAGNOSTIC_REASON_LENGTH):
        return None

    model_ready = value.get('model_ready')
    if model_ready is not None and not isinstance(model_ready, bool):
        return None

    has_query_id = 'query_id' in value
    has_query_version = 'query_version' in value
    if has_query_id != has_query_version:
        return None

    query_id = None
    query_version = None
    if has_query_id:
        try:
            _require_positive_uint64(value['query_id'], 'query_id')
            _require_positive_uint64(
                value['query_version'], 'query_version')
        except ValueError:
            return None
        query_id = value['query_id']
        query_version = value['query_version']
    elif state in ('query_accepted', 'query_rejected'):
        return None

    return PortalDiagnostic(
        state=state,
        reason=reason,
        model_ready=model_ready,
        query_id=query_id,
        query_version=query_version)


@dataclass(frozen=True)
class SubmissionResult:
    outcome: str
    reason: str
    exit_code: int
    diagnostic: Optional[PortalDiagnostic] = None

    @classmethod
    def accepted(
            cls,
            reason: str,
            diagnostic: Optional[PortalDiagnostic] = None,
            ) -> 'SubmissionResult':
        return cls('accepted', reason, 0, diagnostic)

    @classmethod
    def rejected(
            cls,
            reason: str,
            diagnostic: Optional[PortalDiagnostic] = None,
            ) -> 'SubmissionResult':
        return cls('rejected', reason, 2, diagnostic)

    @classmethod
    def no_subscriber(cls, reason: str) -> 'SubmissionResult':
        return cls('no_subscriber', reason, 3)

    @classmethod
    def timed_out(cls, reason: str) -> 'SubmissionResult':
        return cls('timeout', reason, 4)


class QueryIdAllocator:
    def __init__(
            self,
            clock_ns: Callable[[], int] = time.time_ns,
            initial_id: Optional[int] = None) -> None:
        if initial_id is not None:
            _require_positive_uint64(initial_id, 'query_id')
        self._clock_ns = clock_ns
        self._initial_id = initial_id
        self._last_id = 0

    def next_id(self) -> int:
        if self._initial_id is not None:
            candidate = self._initial_id
            self._initial_id = None
        else:
            timestamp_ns = self._clock_ns()
            if not isinstance(timestamp_ns, int) or isinstance(
                    timestamp_ns, bool):
                raise ValueError('query ID clock must return integer nanoseconds')
            candidate = max(1, timestamp_ns // 1000)
        candidate = max(candidate, self._last_id + 1)
        _require_positive_uint64(candidate, 'query_id')
        self._last_id = candidate
        return candidate


class QuerySession:
    def __init__(
            self,
            allocator: QueryIdAllocator,
            initial_version: int = 1) -> None:
        _require_positive_uint64(initial_version, 'query_version')
        self._allocator = allocator
        self._initial_version = initial_version
        self._current: Optional[QueryRequest] = None

    @property
    def current(self) -> Optional[QueryRequest]:
        return self._current

    def new_query(self, text: str) -> QueryRequest:
        version = self._initial_version if self._current is None else 1
        request = QueryRequest.create(
            text,
            self._allocator.next_id(),
            version)
        self._current = request
        return request

    def revise_query(self, text: str) -> QueryRequest:
        if self._current is None:
            raise ValueError('there is no current query to revise')
        request = QueryRequest.create(
            text,
            self._current.query_id,
            self._current.query_version + 1)
        self._current = request
        return request


@dataclass(frozen=True)
class InteractiveCommand:
    kind: str
    text: str = ''


def parse_interactive_command(line: str) -> InteractiveCommand:
    if not isinstance(line, str):
        raise ValueError('query command must be text')
    value = line.strip()
    if not value:
        raise ValueError('query command must not be empty')
    if not value.startswith(':'):
        return InteractiveCommand('new', value)

    command, separator, text = value[1:].partition(' ')
    if command in ('status', 'help', 'quit'):
        if separator and text.strip():
            raise ValueError(':{} takes no text'.format(command))
        return InteractiveCommand(command)
    if command in ('new', 'revise'):
        if not separator or not text.strip():
            raise ValueError(':{} requires query text'.format(command))
        return InteractiveCommand(command, text.strip())
    raise ValueError('unknown command :{}'.format(command))
