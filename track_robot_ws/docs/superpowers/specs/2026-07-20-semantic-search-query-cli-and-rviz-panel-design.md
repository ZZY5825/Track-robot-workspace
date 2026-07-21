# Semantic Search Query CLI and Future RViz Panel Design

**Date:** 2026-07-20

**Status:** Approved

## Goal

Provide a human-friendly ROS 2 command-line portal for submitting semantic
text queries now, while fixing a compatible and safety-bounded direction for a
future RViz panel. The CLI must replace hand-written JSON commands without
claiming that active search, motion control, or the `SearchForObject` action
server already exists.

## Current boundary

`semantic_search_perception` currently accepts `std_msgs/msg/String` JSON on
`/semantic_search/query`. A valid object contains normalized non-empty
`query_text`, a non-negative `query_id`, and a non-negative `query_version`.
The perception node acknowledges accepted and rejected messages through JSON
diagnostics on `/semantic_search/perception_diagnostics`.

The workspace defines `SearchForObject.action`, but it deliberately has no
action server, controller, planner, or `cmd_vel` publisher. This feature must
not create any of those components.

## Chosen approach

Add a focused CLI inside `track_robot_semantic_search`. It will reuse the
existing topic protocol and diagnostics rather than introducing a second
transport. Pure query/session behavior will remain separate from the ROS node
adapter so it can be tested without DDS and later reused conceptually by the
RViz panel.

Alternatives rejected for this stage are:

- a shell-only wrapper around `ros2 topic pub`, because it cannot reliably
  manage identifiers or acknowledge delivery;
- an action-client portal, because no action server exists and implementing
  one would pull motion, cancellation, timeout, and safety behavior into this
  scope.

## Command-line interface

The installed executable is:

```bash
ros2 run track_robot_semantic_search semantic_search_query
```

### One-shot mode

A positional query submits one request and exits after acceptance, rejection,
missing-subscriber detection, or acknowledgment timeout:

```bash
ros2 run track_robot_semantic_search semantic_search_query \
  "a red backpack"
```

Supported options are:

- `--query-id UINT64`: explicit positive query identifier;
- `--query-version UINT64`: explicit positive version, default `1`;
- `--query-topic NAME`: default `/semantic_search/query`;
- `--diagnostics-topic NAME`: default
  `/semantic_search/perception_diagnostics`;
- `--timeout SECONDS`: finite positive acknowledgment timeout, default `5.0`;
- `--subscriber-timeout SECONDS`: finite non-negative wait for a query
  subscriber, default `2.0`.

When `--query-id` is omitted, the session derives a positive unsigned 64-bit
identifier from Unix epoch microseconds. IDs generated later in the same
process are strictly increasing even if the wall clock repeats or moves
backwards. Explicit IDs remain available for deterministic tests and operator
replay.

### Interactive mode

Omitting the positional text starts a prompt. EOF and `:quit` exit cleanly.
Commands are:

- plain non-empty text or `:new TEXT`: allocate a new query ID and use version
  `1`;
- `:revise TEXT`: retain the current query ID and increment its version;
- `:status`: print the last submitted query and latest relevant diagnostic;
- `:help`: print the command summary;
- `:quit`: exit without publishing.

`:revise` before the first successful or pending query is rejected locally.
Unknown colon commands and empty query text are rejected locally without ROS
publication. A rejected or timed-out submission still becomes the current
query for the purpose of an explicit `:revise`, because the operator may need
to correct and resubmit the same logical request with a higher version.

## Query payload and acknowledgment

The pure transport helper builds canonical compact JSON containing exactly:

```json
{"query_id":1,"query_text":"a red backpack","query_version":1}
```

Text uses the existing Unicode NFKC normalization and whitespace collapsing.
The CLI enforces the perception contract's 512-character post-normalization
bound before publication. IDs and versions must be in `[1, 2^64 - 1]`; the
CLI intentionally uses a stricter positive range than the legacy parser's
non-negative compatibility range.

The ROS adapter creates one reliable, volatile, depth-one publisher compatible
with the existing query subscriber and one reliable, volatile diagnostics
subscription. Before publishing it waits up to `--subscriber-timeout` for at
least one subscriber. After publishing it spins until it observes:

- `state == "query_accepted"` with matching `query_id` and `query_version`;
- `state == "query_rejected"` with matching `query_id` and
  `query_version`; or
- the acknowledgment deadline.

Malformed and uncorrelated diagnostics are ignored and retained only as a
bounded human-readable status reason. A diagnostic with a different or absent
query key cannot acknowledge or reject the current submission. The perception
callback will therefore include the parsed query ID/version when it rejects a
payload after successful parsing; malformed payloads remain uncorrelated.
After acceptance, a false `model_ready` value is printed as a warning but does
not rewrite the perception node's acceptance decision.

## Output and exit behavior

Human output includes the normalized query, ID, version, acknowledgment state,
and reason. It never prints feature vectors or image data.

One-shot exit codes are:

- `0`: matching `query_accepted` received;
- `2`: invalid local input or `query_rejected` received;
- `3`: no query subscriber became available;
- `4`: acknowledgment timeout;
- `130`: keyboard interrupt.

Interactive mode reports each submission result and continues after codes
`2`, `3`, or `4`. It returns `0` on `:quit` or EOF and `130` on interruption.
ROS initialization or middleware failures return `1` with a bounded error
message.

## Components and files

### Shared pure protocol

`track_robot_semantic_search/query_portal.py` owns:

- query-key validation and canonical JSON construction;
- time-seeded monotonic ID allocation;
- the interactive session state machine;
- diagnostics parsing and acknowledgment matching;
- result/status value types that contain no ROS objects.

### ROS and terminal adapter

`track_robot_semantic_search/query_cli.py` owns:

- `argparse` definitions;
- ROS publisher/subscriber construction;
- bounded subscriber and acknowledgment waits;
- one-shot and interactive terminal loops;
- mapping results to human output and exit codes.

`setup.py` installs `semantic_search_query` as a console script. The package
README documents one-shot and interactive operation, expected diagnostics,
English-query guidance for the current OpenAI CLIP model, and the absence of
active motion.

## Test strategy

Development follows red-green-refactor. Pure tests cover:

- normalization, 512-character bound, canonical payload ordering, and uint64
  validation;
- time-derived IDs under equal and backward timestamps;
- explicit ID/version behavior;
- `:new`, plain text, `:revise`, `:status`, `:help`, `:quit`, EOF, and invalid
  command transitions;
- matching, stale, malformed, accepted, rejected, and model-unready
  diagnostics;
- exit-code mapping and bounded reason formatting.

CLI tests inject a fake ROS adapter and terminal streams to prove one-shot and
interactive behavior without relying on timing-sensitive DDS. Contract tests
verify the console entry point and ROS topic defaults. An opt-in DDS integration
test may exercise real publication and diagnostics acknowledgment when the ROS
environment is available; it must not be the only coverage of core behavior.

The full existing semantic-search Python suite and package-level build/test
gate must remain green.

## Safety and operational constraints

- The CLI publishes only to the configured query topic.
- It does not publish `SearchMotionIntent`, `Twist`, or `cmd_vel`.
- It does not call reset or inspection services.
- It does not download models or modify calibration.
- It must tolerate zero subscribers and a stopped perception node without
  hanging indefinitely.
- Operators should use English queries for the current official OpenAI CLIP
  ViT-B/32 checkpoint; Unicode input is accepted by the transport, but Chinese
  retrieval quality is not claimed.

## Future RViz panel plan

The future UI belongs in a separate C++/Qt package named
`track_robot_semantic_search_rviz_plugins`. Keeping it separate avoids adding
Qt and RViz dependencies to the Python perception/runtime package.

### RViz milestone 1: passive query panel

The first panel will:

- provide a text field plus **New Query** and **Revise** buttons;
- publish the same canonical JSON to `/semantic_search/query`;
- display current query ID/version, model state, acceptance/rejection reason,
  region count, active-object count, and best-candidate state;
- subscribe to `/semantic_search/perception_diagnostics`,
  `/semantic_search/regions`, `/semantic_memory/active_objects`, and
  `/semantic_memory/best_candidate`;
- show a permanent statement that the panel performs passive observation and
  does not command robot motion.

The panel will not duplicate 3D marker rendering. Existing
`/semantic_memory/markers` remains the RViz 3D visualization source.

### RViz milestone 2: usability and saved configuration

After milestone 1 field validation, the panel may add bounded query history,
topic-name properties, timeout configuration, a selected-object detail view,
and RViz configuration persistence. Query history remains process-local unless
a separate privacy and persistence decision is approved.

### RViz milestone 3: action-client migration

Only after a separately designed and safety-reviewed `SearchForObject` action
server exists may the panel become an action client. That milestone can expose
cancel, timeout, passive/rotation phases, searched angle, and terminal result.
It must not synthesize these states from topic traffic and must not publish
motion commands itself.

## Acceptance criteria for the current implementation

The CLI stage is complete when:

1. one-shot and interactive commands publish valid canonical query payloads;
2. query IDs and versions follow the specified state transitions;
3. matching diagnostics produce deterministic acceptance, rejection, missing
   subscriber, and timeout outcomes;
4. all waits are bounded and interruption is clean;
5. documentation lets an operator submit a query without hand-writing JSON;
6. focused, existing semantic-search, package build, and applicable DDS tests
   pass; and
7. no ROS node or service started for verification remains running afterward.
