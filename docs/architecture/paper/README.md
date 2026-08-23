# Paper Architecture Figures

These figures are publication-oriented counterparts to the diagrams used in
the repository README. They use compact labels, restrained colors, square
geometry, and redundant line styles so that the information remains legible
when printed in grayscale.

The DOT files in this directory are the authoritative sources. Render all
vector assets from the repository root with:

```bash
for source in docs/architecture/paper/*.dot; do
  name="$(basename "${source}" .dot)"
  dot -Tsvg "${source}" -o "docs/assets/paper/architecture/${name}.svg"
  dot -Tpdf "${source}" -o "docs/assets/paper/architecture/${name}.pdf"
done
```

The figures deliberately contain no embedded title or caption. Add those in
the manuscript so their typography and numbering follow the publication
template.

## Visual semantics

- solid arrows: primary runtime data flow;
- dashed arrows: authorization, calibration, optional, model-only, or
  intentionally disconnected relationships;
- blue: sensing and physical inputs;
- violet: learned perception;
- teal: estimation, fusion, localization, and memory;
- ochre: task decisions and planning;
- muted red: authorization and motion safety;
- gray: compute, actuation, outputs, and implementation-boundary notes.
