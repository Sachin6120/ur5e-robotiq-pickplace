# Curated Validation Evidence

This directory is the compact public evidence bundle for the validated
PlanningScene lifecycle baseline and Stage-2D planar pose regression. The
approximately 9.5 GB raw evidence store remains local because it contains
high-rate pose, contact, image, and process logs that are unsuitable for the
Git repository.

The public bundle contains:

- [validation_summary.json](validation_summary.json) — machine-readable
  Scene-A, D1, D2, and D3 metrics.
- [validation_summary.csv](validation_summary.csv) — the same four records in
  tabular form.
- [d3_clearance_analysis.md](d3_clearance_analysis.md) — the historical D3
  fixed-pad failure and the verified 2.0 mm correction.
- [provenance.sha256](provenance.sha256) — hashes of the small local source
  records and public authority documents used for this curation.

`m1_planning.csv` predates this Stage-2D bundle and remains as compact M1
planning evidence.

## Provenance and interpretation

Values were transcribed from the preserved authoritative runs identified in
each summary row and cross-checked against the current authority sections of
`PROJECT_STATE.md` and `HANDOFF.md`. Raw evidence was read but not modified.
The `evidence_reference` values are repository-relative identifiers for the
local raw archive; they are not links promised to exist in a public clone.

`yaw_error_deg` is signed perceived yaw minus spawned yaw. D3's qualification
placement error is 1.9793 mm from the PlanningScene regression. The later
2.1433 mm value belongs to a separate recording-only demo run and is not used
as a qualification metric.

The hashes do not imply that the raw files are distributed through Git. They
provide a stable mapping for maintainers who hold the local archive. No hash
was invented, and no giant directory was hashed wholesale.
