# Architecture Diagram Sources

The DOT files in this directory are the authoritative sources for the
architecture figures embedded in the repository README. Generated SVG files
live in `docs/assets/readme/architecture/` and are committed so GitHub can
render the diagrams without an external service.

Render one diagram from the repository root:

```bash
dot -Tsvg docs/architecture/diagrams/system-overview.dot \
  -o docs/assets/readme/architecture/system-overview.svg
```

Render all diagrams:

```bash
for source in docs/architecture/diagrams/*.dot; do
  name="$(basename "${source}" .dot)"
  dot -Tsvg "${source}" \
    -o "docs/assets/readme/architecture/${name}.svg"
done
```

## Visual vocabulary

| Color | Meaning |
|---|---|
| Blue | Sensors and physical inputs |
| Violet | Perception and learned models |
| Teal | Localization, fusion, and memory |
| Amber | Planning and task decisions |
| Red | Safety and authorization |
| Graphite | Compute and actuation |
| Gray | Notes, optional paths, and non-integrated components |

Solid arrows represent runtime data flow. Dashed arrows represent control,
authorization, model-only, optional, or intentionally independent
relationships. Diagram wording must follow the implementation boundaries in
the accompanying design specification.
