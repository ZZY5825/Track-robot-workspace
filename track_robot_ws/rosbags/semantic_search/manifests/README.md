# Semantic Search Manifests

Every manifest is immutable after a report references its SHA-256. Before
evaluation, query events, object records, trial records, and annotation-file
hashes may be appended only through semantic_search_manifest. Paths are
workspace-relative.
Train/validation/test splits are assigned by physical object, site, and date,
never adjacent frames. Legacy bags use legacy_replay_only and cannot be promoted
to a field-test split.

Generate and extend manifests with semantic_search_manifest; do not hand-edit
checksums or rosbag metadata. Annotation JSONL uses the installed annotation
schema and is validated before its hash is registered.
