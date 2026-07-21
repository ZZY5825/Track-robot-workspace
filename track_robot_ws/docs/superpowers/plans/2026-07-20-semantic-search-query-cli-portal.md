# Semantic Search Query CLI Portal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Add a bounded one-shot and interactive ROS 2 CLI portal that submits semantic text queries, correlates diagnostics, and removes the need to hand-write JSON.

**Architecture:** A pure Python protocol/session module owns validation, canonical payloads, ID allocation, diagnostics correlation, and interactive state. A thin ROS adapter owns DDS entities and bounded waits, while terminal functions operate against an injected client so normal unit tests do not require DDS.

**Tech Stack:** Python 3.8, ROS 2 Foxy `rclpy`, `std_msgs/msg/String`, `argparse`, `pytest`, `setuptools` console scripts.

## Global Constraints

- Preserve the existing `/semantic_search/query` `std_msgs/msg/String` JSON protocol.
- Publish no motion intent, `Twist`, `cmd_vel`, reset, or inspection request.
- Support one-shot and interactive operation with finite waits and deterministic exit codes.
- Enforce normalized text length at most 512 characters and positive uint64 query IDs/versions.
- Do not add non-ROS dependencies, model downloads, persistence, or RViz/Qt code in this stage.
- Preserve all unrelated untracked build, install, and calibration artifacts.
- Stop every ROS node or service started by validation before completion.

---

### Task 1: Pure query portal protocol and session state

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/query_portal.py`
- Create: `src/track_robot_semantic_search/test/test_query_portal.py`

**Interfaces:**
- Produces: `QueryRequest`, `PortalDiagnostic`, `SubmissionResult`, `QueryIdAllocator`, `QuerySession`, `parse_diagnostic(payload)`, and `parse_interactive_command(line)`.
- Consumes: existing `normalize_query(text)` from `query.py`.

- [x] **Step 1: Write failing protocol and validation tests**

Add tests proving canonical JSON, NFKC/whitespace normalization, 512-character enforcement, positive uint64 bounds, matching diagnostic correlation, and malformed/unrelated diagnostic rejection:

```python
def test_request_builds_canonical_payload():
    request = QueryRequest.create('  red\u3000ｂａｇ  ', 7, 2)
    assert request.query_text == 'red bag'
    assert request.payload == (
        '{"query_id":7,"query_text":"red bag","query_version":2}')


@pytest.mark.parametrize('text', ['', '   ', 'x' * 513])
def test_request_rejects_invalid_text(text):
    with pytest.raises(ValueError):
        QueryRequest.create(text, 1, 1)


@pytest.mark.parametrize('value', [0, -1, True, 2 ** 64])
def test_request_rejects_invalid_uint64(value):
    with pytest.raises(ValueError):
        QueryRequest.create('target', value, 1)


def test_matching_diagnostic_acknowledges_only_same_key():
    request = QueryRequest.create('target', 9, 3)
    accepted = parse_diagnostic(
        '{"state":"query_accepted","reason":"ok","model_ready":true,'
        '"query_id":9,"query_version":3}')
    stale = parse_diagnostic(
        '{"state":"query_accepted","reason":"old","model_ready":true,'
        '"query_id":8,"query_version":3}')
    assert accepted.matches(request)
    assert not stale.matches(request)
```

- [x] **Step 2: Run Task 1 tests and verify RED**

Run:

```bash
PYTHONPATH=src/track_robot_semantic_search \
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_query_portal.py
```

Expected: collection fails because `query_portal` does not exist.

- [x] **Step 3: Implement request and diagnostic value types**

Implement frozen dataclasses with these public shapes:

```python
UINT64_MAX = 2 ** 64 - 1


@dataclass(frozen=True)
class QueryRequest:
    query_text: str
    query_id: int
    query_version: int

    @classmethod
    def create(cls, text, query_id, query_version):
        normalized = normalize_query(text)
        if len(normalized) > 512:
            raise ValueError('query text exceeds 512 characters')
        _require_positive_uint64(query_id, 'query_id')
        _require_positive_uint64(query_version, 'query_version')
        return cls(normalized, query_id, query_version)

    @property
    def payload(self):
        return json.dumps({
            'query_id': self.query_id,
            'query_text': self.query_text,
            'query_version': self.query_version,
        }, ensure_ascii=False, separators=(',', ':'), sort_keys=True)


@dataclass(frozen=True)
class PortalDiagnostic:
    state: str
    reason: str
    model_ready: Optional[bool]
    query_id: Optional[int]
    query_version: Optional[int]

    def matches(self, request):
        return (self.query_id, self.query_version) == (
            request.query_id, request.query_version)
```

`parse_diagnostic` must return `None` for invalid JSON, non-object payloads,
unbounded/invalid reasons, invalid keys, or irrelevant states instead of
raising inside a ROS callback.

- [x] **Step 4: Add failing allocator and session tests**

```python
def test_allocator_is_monotonic_when_clock_repeats_or_rolls_back():
    ticks = iter([5_000_000, 5_000_000, 4_000_000])
    allocator = QueryIdAllocator(clock_ns=lambda: next(ticks))
    assert [allocator.next_id() for _ in range(3)] == [5000, 5001, 5002]


def test_session_new_and_revise_transitions():
    session = QuerySession(QueryIdAllocator(clock_ns=lambda: 20_000_000))
    first = session.new_query('red bag')
    revised = session.revise_query('black bag')
    second = session.new_query('box')
    assert (first.query_id, first.query_version) == (20000, 1)
    assert (revised.query_id, revised.query_version) == (20000, 2)
    assert (second.query_id, second.query_version) == (20001, 1)


def test_revise_requires_current_query():
    session = QuerySession(QueryIdAllocator(clock_ns=lambda: 1_000_000))
    with pytest.raises(ValueError, match='current query'):
        session.revise_query('target')
```

- [x] **Step 5: Run the new session tests and verify RED**

Run the Task 1 test file and confirm failures report missing allocator/session
behavior rather than import or fixture errors.

- [x] **Step 6: Implement allocator, session, command parser, and result types**

Implement:

```python
class QueryIdAllocator:
    def __init__(self, clock_ns=time.time_ns, initial_id=None):
        self._clock_ns = clock_ns
        self._initial_id = initial_id
        self._last_id = 0
        if initial_id is not None:
            _require_positive_uint64(initial_id, 'query_id')

    def next_id(self) -> int:
        if self._initial_id is not None:
            candidate = self._initial_id
            self._initial_id = None
        else:
            candidate = max(1, self._clock_ns() // 1000)
        candidate = max(candidate, self._last_id + 1)
        _require_positive_uint64(candidate, 'query_id')
        self._last_id = candidate
        return candidate


class QuerySession:
    def __init__(self, allocator, initial_version=1):
        _require_positive_uint64(initial_version, 'query_version')
        self._allocator = allocator
        self._initial_version = initial_version
        self._current = None

    @property
    def current(self) -> Optional[QueryRequest]:
        return self._current

    def new_query(self, text: str) -> QueryRequest:
        version = self._initial_version if self._current is None else 1
        self._current = QueryRequest.create(
            text, self._allocator.next_id(), version)
        return self._current

    def revise_query(self, text: str) -> QueryRequest:
        if self._current is None:
            raise ValueError('there is no current query to revise')
        self._current = QueryRequest.create(
            text,
            self._current.query_id,
            self._current.query_version + 1)
        return self._current


@dataclass(frozen=True)
class InteractiveCommand:
    kind: str
    text: str = ''


def parse_interactive_command(line: str) -> InteractiveCommand:
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
```

Plain text maps to `new`; supported colon commands are `new`, `revise`,
`status`, `help`, and `quit`. Empty text and unknown commands raise bounded
`ValueError`s.

- [x] **Step 7: Run Task 1 tests and verify GREEN**

Expected: all `test_query_portal.py` tests pass with no warnings.

- [x] **Step 8: Commit Task 1**

```bash
git add \
  src/track_robot_semantic_search/track_robot_semantic_search/query_portal.py \
  src/track_robot_semantic_search/test/test_query_portal.py
git commit -m "feat: add semantic query portal protocol"
```

---

### Task 2: One-shot and interactive CLI with injected ROS client

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/query_cli.py`
- Create: `src/track_robot_semantic_search/test/test_query_cli.py`

**Interfaces:**
- Consumes: all Task 1 public value types.
- Produces: `parser()`, `run_one_shot(client, request, subscriber_timeout,
  ack_timeout, output)`, `run_interactive(client, session,
  subscriber_timeout, ack_timeout, input_stream, output)`, `RosQueryClient`,
  and `main(args=None)`.

- [x] **Step 1: Write failing parser and one-shot tests**

Use a fake client implementing `submit(request, subscriber_timeout,
ack_timeout) -> SubmissionResult`:

```python
def test_parser_accepts_one_shot_query_and_defaults():
    args = parser().parse_args(['a red backpack'])
    assert args.query_text == 'a red backpack'
    assert args.timeout == 5.0
    assert args.subscriber_timeout == 2.0


def test_one_shot_prints_correlated_acceptance():
    client = FakeClient(SubmissionResult.accepted('model loaded'))
    output = io.StringIO()
    code = run_one_shot(
        client, QueryRequest.create('target', 10, 1), 2.0, 5.0, output)
    assert code == 0
    assert 'query_id=10' in output.getvalue()
    assert 'accepted' in output.getvalue().lower()
```

Parameterize results to require exit codes `2`, `3`, and `4` for rejection,
missing subscriber, and timeout.

- [x] **Step 2: Run focused CLI tests and verify RED**

Expected: collection fails because `query_cli` does not exist.

- [x] **Step 3: Implement parser and transport-independent terminal functions**

Implement an `argparse.ArgumentParser` with the positional text and exact
options from the spec. Validate finite timeout values before ROS startup.

`run_one_shot` prints one bounded result and returns its exit code.
`run_interactive` reads from an injected text stream, delegates state changes
to `QuerySession`, submits through the injected client, implements status/help,
continues after submission errors, and exits `0` on EOF or `:quit`.

- [x] **Step 4: Add failing interactive behavior tests**

```python
def test_interactive_new_revise_status_and_quit():
    source = io.StringIO(
        'red bag\n:revise black bag\n:status\n:quit\n')
    output = io.StringIO()
    client = FakeClient(SubmissionResult.accepted('ok'))
    session = QuerySession(QueryIdAllocator(clock_ns=lambda: 30_000_000))
    assert run_interactive(client, session, 2.0, 5.0, source, output) == 0
    assert [(r.query_id, r.query_version) for r in client.requests] == [
        (30000, 1), (30000, 2)]
    assert 'black bag' in output.getvalue()
```

Also test `:revise` before a query, unknown commands, EOF, local validation,
and continuing after timeout.

- [x] **Step 5: Implement the lazy-import ROS adapter**

`RosQueryClient.__init__` lazily imports `rclpy`, `std_msgs.msg.String`, and
Foxy QoS symbols; creates a reliable/volatile/depth-one publisher and
diagnostics subscription; and retains both entities for node lifetime.

`submit` must:

1. wait only until the subscriber deadline;
2. clear stale queued diagnostics;
3. publish one `String(data=request.payload)`;
4. spin in bounded slices until a correlated accepted/rejected diagnostic or
   acknowledgment deadline;
5. return a `SubmissionResult` rather than raising for expected runtime states.

- [x] **Step 6: Run Task 2 tests and verify GREEN**

Run:

```bash
PYTHONPATH=src/track_robot_semantic_search \
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_query_portal.py \
  src/track_robot_semantic_search/test/test_query_cli.py
```

Expected: all focused portal tests pass.

- [x] **Step 7: Commit Task 2**

```bash
git add \
  src/track_robot_semantic_search/track_robot_semantic_search/query_cli.py \
  src/track_robot_semantic_search/test/test_query_cli.py
git commit -m "feat: add semantic text query cli"
```

---

### Task 3: Correlated rejection, packaging, and operator documentation

**Files:**
- Modify: `src/track_robot_semantic_search/track_robot_semantic_search/perception_node.py`
- Modify: `src/track_robot_semantic_search/test/test_perception_node_contract.py`
- Modify: `src/track_robot_semantic_search/setup.py`
- Modify: `src/track_robot_semantic_search/README.md`

**Interfaces:**
- Consumes: the existing diagnostics JSON and the Task 2 console entry point.
- Produces: correlated post-parse rejection diagnostics and installed `semantic_search_query` executable.

- [x] **Step 1: Write failing perception correlation and packaging tests**

Add a source-contract test requiring `_query_callback` to initialize `query =
None`, include `query_id/query_version` in post-parse rejection metrics, and
leave malformed pre-parse rejection uncorrelated. Add a setup contract test:

```python
def test_query_cli_is_packaged_as_console_script():
    setup_source = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    assert 'semantic_search_query' in setup_source
    assert 'track_robot_semantic_search.query_cli:main' in setup_source
```

- [x] **Step 2: Run the contract tests and verify RED**

Expected: failures identify absent rejection correlation and console script.

- [x] **Step 3: Implement correlated rejection diagnostics**

Change the callback shape to:

```python
def _query_callback(self, message):
    query = None
    try:
        query = parse_query_payload(message.data)
        if len(query.query_text) > 512:
            raise ValueError('query_text exceeds the 512 character bound')
        self._active_query = query
        self._publish_diagnostics(
            'query_accepted',
            'semantic query activated',
            query_id=query.query_id,
            query_version=query.query_version,
            image_message_count=self._image_message_count,
            processing_timer_started=self._processing_timer_started)
    except ValueError as exc:
        metrics = {}
        if query is not None:
            metrics = {
                'query_id': query.query_id,
                'query_version': query.query_version,
            }
        self._publish_diagnostics('query_rejected', str(exc), **metrics)
```

Do not alter accepted-query behavior.

- [x] **Step 4: Install the console script and document operation**

Add:

```python
'semantic_search_query = '
'track_robot_semantic_search.query_cli:main',
```

Document one-shot, explicit ID/version, interactive commands, exit outcomes,
English-query guidance, topic overrides, and the passive/no-motion boundary in
the package README.

- [x] **Step 5: Run focused protocol, CLI, and perception tests**

Expected: all portal and modified perception contract tests pass.

- [x] **Step 6: Commit Task 3**

```bash
git add \
  src/track_robot_semantic_search/track_robot_semantic_search/perception_node.py \
  src/track_robot_semantic_search/test/test_perception_node_contract.py \
  src/track_robot_semantic_search/setup.py \
  src/track_robot_semantic_search/README.md
git commit -m "docs: expose semantic query portal"
```

---

### Task 4: Package build, DDS smoke test, regression, and closure

**Files:**
- Modify: `docs/superpowers/plans/2026-07-20-semantic-search-query-cli-portal.md`

**Interfaces:**
- Validates: installed console command, real ROS topic/diagnostics handshake, existing semantic-search behavior, and process cleanup.

- [x] **Step 1: Run the full semantic-search Python suite**

Source ROS Foxy and the most recent interface install, prepend the source
package to `PYTHONPATH`, disable bytecode/cache output, and run every test under
`src/track_robot_semantic_search/test`.

Expected: zero failures and zero errors.

- [x] **Step 2: Build the semantic-search package in fresh roots**

Run `colcon build` with a new query-portal-specific build/install/log root and
`--event-handlers console_direct+`. Do not reuse Stage 2G build output as build
evidence.

Expected: package builds successfully and installs `semantic_search_query`.

- [x] **Step 3: Verify installed CLI help and local validation**

Source the fresh install and run:

```bash
ros2 run track_robot_semantic_search semantic_search_query --help
ros2 run track_robot_semantic_search semantic_search_query "   "
```

Expected: help exits `0`; invalid local input exits `2` without publication.

- [x] **Step 4: Run a bounded DDS acknowledgment smoke test**

Start a temporary ROS test node that subscribes to `/semantic_search/query`,
parses the payload, and publishes a matching `query_accepted` diagnostic. Run
the installed one-shot CLI with explicit ID/version and a short timeout.

Expected: CLI exits `0`, the captured payload is canonical, and the helper node
is stopped immediately afterward.

- [x] **Step 5: Run static and repository hygiene gates**

Run:

```bash
git diff --check
python3 -m compileall -q src/track_robot_semantic_search/track_robot_semantic_search
```

Parse touched JSON/Markdown command examples where applicable, verify no
generated caches are staged, and inspect the diff for any publisher other than
the query topic.

- [x] **Step 6: Confirm ROS process cleanup**

Inspect the process table for the CLI test node, semantic-search perception,
semantic memory, visualizer, rosbag player, and ROS test services. Stop any
process created by this plan and recheck until none remain.

- [x] **Step 7: Close the plan and commit verification record**

Mark every completed checkbox, record exact test/build/DDS counts below this
task, then commit only this plan file:

```bash
git add docs/superpowers/plans/2026-07-20-semantic-search-query-cli-portal.md
git commit -m "docs: close semantic query cli plan"
```

## Verification record

Completed on 2026-07-20 on branch `feature/semantic-search-phase1`.

- Existing isolated-worktree baseline: `380 passed in 3.58s`.
- Task 1 RED: collection failed because `query_portal` did not exist.
- Task 1 GREEN: `50 passed in 0.18s`.
- Task 2 RED: collection failed because `query_cli` did not exist.
- Task 2 GREEN: portal and CLI tests `69 passed in 0.29s`.
- Task 3 RED: two expected failures for absent rejection correlation and
  console-script packaging.
- Task 3 GREEN: focused portal, CLI, and perception contract tests
  `82 passed in 0.32s`.
- Security review RED: two expected failures for oversized diagnostic input
  and unescaped terminal control characters.
- Security review GREEN: focused portal and CLI tests `71 passed in 0.27s`.
- Final source suite: `453 passed in 3.75s`.
- Final fresh install build: `1 package finished`, exit `0`; installed
  `semantic_search_query`.
- Final installed `colcon test`: `453 tests, 0 errors, 0 failures, 0 skipped`.
- Installed help exit: `0`; empty-query validation exit: `2`.
- Final DDS smoke: installed CLI exit `0` with `query_id=77`, version `3`, and
  captured canonical payload
  `{"query_id":77,"query_text":"a red backpack","query_version":3}`.
- `compileall`, `git diff --check`, and static forbidden-sink scan exited `0`.
- Process-table recheck found no CLI smoke node, semantic perception, semantic
  memory, visualizer, or rosbag player left running.

Security review verdict: **pass**. Entry points are local terminal text and ROS
diagnostics; the primary assets are task integrity and process availability.
Inputs are length/type bounded, waits are finite, diagnostics are correlated,
and terminal control characters are escaped. No shell execution, filesystem
write, secret, network fetch, motion publisher, reset call, or inspection call
was added. The remaining ROS-graph trust model is inherited from the existing
deployment and is not broadened by this CLI.

Maintainability review verdict: **approved**. Pure validation/session behavior
is isolated from the lazy-import ROS adapter, terminal behavior is tested with
an injected client, and the only new publisher is the configured semantic
query topic.
