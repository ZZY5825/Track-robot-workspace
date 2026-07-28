# Phase 2 manifests

No Phase 2 pilot manifest is checked in yet because this workspace contains no
closed pilot `.db3`/`.mcap` payload. A valid manifest must contain the real bag
metadata and cryptographic checksum; a placeholder would be false evidence.

Follow `docs/guides/semantic-search/phase2-recording-and-evaluation.md`. After recording and
closing the bag, generate the manifest here, add the strict `phase2` evidence
block, validate it, and register the checked annotation JSONL.
