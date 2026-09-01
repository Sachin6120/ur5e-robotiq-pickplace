# Documentation Index

This directory contains both current public documentation and retained
engineering history. Historical reports are preserved because they record the
evidence and reasoning behind the current implementation; they are not all
current operating instructions.

## Public documentation

- [Project overview and demo](../README.md)
- [WSL setup notes](SETUP_WSL.md)
- [Curated validation evidence](evidence/README.md)
- [Parallel-jaw gripper design rationale](GRIPPER_REDESIGN_DESIGN.md)
- [Licensing and third-party attribution](../THIRD_PARTY_NOTICES.md)

## Current engineering state

- [Project state](../PROJECT_STATE.md)
- [Primary handoff](../HANDOFF.md)

These files begin with the current Stage-2D authority section and retain older
milestone sections below it as history.

## Engineering history

- `F3_*.md` — F3 measurement plans and audits.
- `HANDOFF_M3.md` — historical manipulation milestones M0-M5 and associated
  investigations.
- `HANDOFF_RGBD_PERCEPTION.md` — historical RGB-D perception development.
- `M-1_reference_report.md` — early merged-platform reference report.
- Dated `m3_*`, `m4_*`, `m6_*`, and related capture scripts — reproducibility
  tooling for historical experiments.

Raw runtime captures are excluded from the public branch. Where a historical
artifact was removed during release cleanup, the associated conclusion remains
in its engineering record.
