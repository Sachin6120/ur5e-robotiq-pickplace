# D3 Fixed-Side Clearance Analysis

This record summarizes the Stage-2D D3 failure mechanism and the production
correction. It reports the already validated engineering result; it does not
re-derive or modify the manipulation geometry.

## Historical 1.5 mm clearance

For D3 at +30 mm X, -30 mm Y, and +45 deg yaw, the governing fixed-side
projection of the perception error was approximately 1.5759 mm. With the
historical 1.5 mm fixed-side clearance:

```text
margin = 1.5000 mm - 1.5759 mm ≈ -0.0759 mm
```

The negative predicted margin matched physical simulation evidence:

- fixed-pad contact with the target before closure;
- an approximately 59.2 µm contact sliver;
- approximately 301 N simulated contact force;
- descent stopped after approximately 58.4% of the commanded travel; and
- cycle result `EXECUTE_FAILURE`.

Local raw evidence identifiers:

- `evidence/stage2d_pose/D3`
- `evidence/stage2d_pose/D3_retry1_diagnostics`

## Corrected 2.0 mm production clearance

The production fixed-side clearance is 2.0 mm. The conservative working model
margin is `Mmodel_working = 0.000001 mm`:

```text
remaining margin
  = 2.0000 mm - 1.5759 mm - 0.000001 mm
  ≈ +0.4241 mm
```

`Mmodel_working` is a design allowance used in the working clearance model. It
is not a measured uncertainty.

The corrected execution verified:

- fixed-pad pre-close contacts: 0;
- moving-pad pre-close contacts: 0;
- Cartesian descent fraction: 1.0000; and
- complete D3 cycle: PASS.

The definitive PlanningScene regression record is
`evidence/stage2d_pose/D3_planning_scene_20260901_0230`. The preceding
production correction and diagnostic runs are retained locally as
`D3_production_default_2mm` and `D3_clearance2mm_diag`.

## Direction naming

The physical fixed-pad clearance direction corresponds to the relevant
gripper-frame component. Historical records did not always use
"closing" and "transverse" labels consistently for that component. This
document therefore preserves historical metric names when citing them and
identifies the physical fixed-pad direction explicitly rather than silently
renaming older measurements.
