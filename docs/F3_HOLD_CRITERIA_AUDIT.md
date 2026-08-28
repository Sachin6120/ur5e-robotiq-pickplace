# F3 Hybrid-Hold Acceptance Criteria — Measurement-Semantics Audit

Status: **accepted as the basis for the next F3 design step**, 2026-08-23.

This document audits the acceptance criteria that were applied to the two
Scene-A hybrid-hold trials. It is an audit of *measurement semantics*, not of
controller behaviour, and it changes no controller, gain, physics, geometry,
perception, scene, or production configuration.

## Historical verdicts are NOT rewritten

```
H25   20260823_160017_30609   INDETERMINATE  (empty contact evidence: finger_right)
P12.5 20260823_172848_26905   INDETERMINATE  (H25 horizon evidence invalid)
F3 Scene A 20260823_130143_24582  INDETERMINATE (unreliable G0)
```

Both hybrid-hold verdicts remain recorded exactly as the harness wrote them,
and remain correct **as applications of the rules that were frozen at the
time**. This audit concerns whether those rules measured what they were
intended to measure. Nothing here converts any recorded INDETERMINATE into a
PASS, and no run is re-scored.

---

## 1. Provenance audit of the previous hybrid-hold criteria

| # | Criterion | Value | Defined in | Derivation on record | Type |
|---|---|---|---|---|---|
| 1 | F2/perception prerequisite gate | structured SUCCESS row, perceived source, Cartesian 1.0, bilateral contact, boundary-postdating artifacts | `current_run_prerequisite_gate.py` | **Yes** — constant-effort run-identity repair after `20260823_150021_4975` | instrumentation |
| 2 | Descent pre-close | 0.070000 rad | `config/scene.yaml` | **Yes** — real-collision-STL sweep against measured follower states, validated scene-by-scene | physical |
| 3 | Loaded-PID baseline duration | ≥ 2.000 s | `run_scene_A_h25.sh`, `compute_q_ref.py` | **No** | mixed |
| 4 | q_ref estimator | median over final 0.500 s | `compute_q_ref.py` | **No** | instrumentation |
| 5 | q_ref window quality | ≥100 samples, endpoints ±0.010 s, gap ≤ 0.004001 s | `compute_q_ref.py` | **No** | instrumentation |
| 6 | H0 / P0 definition | first of 10 consecutive valid rows, gap ≤ 0.004001 s | `establish_h0.py` | **No** | instrumentation |
| 7 | Horizon | H0 → H0+20.000 s | H25 README | **No** | protocol |
| 8 | W1/W2/S1/S2 | [H0,+10], [+10,+20], [+8,+10], [+18,+20] | `validate_h25_horizon.py` | **No** | protocol |
| 9 | Horizon sample count | ≥ 9000 | `validate_h25_horizon.py` | **Implicit** — 90 % of 500 Hz × 20 s = 10000 | instrumentation |
| 10 | **Horizon max terms gap** | **≤ 0.004001 s** | `validate_h25_horizon.py` (also 5, 6) | **None anywhere in the repository** | instrumentation |
| 11 | Per-row law reconstruction | `abs_tol = 1e-10` on 8 terms | `establish_h0.py`, `validate_h25_horizon.py` | **Implicit** — float64 round-trip through 17-significant-figure text | scientific |
| 12 | Clock/message skew | ≤ 0.01 s | `establish_h0.py` | **No** (controller-only validation observed ≤ 1 ms) | instrumentation |
| 13 | Native raw-stream coverage | ≥ 2 samples, endpoints ±0.100001 s, **no gap test** | `validate_stream_coverage.py` | **No** | instrumentation |
| 14 | Strict switch + exclusive ownership | STRICT=2, `activate_asap`, 5 s, sole master-effort claimant | `h25_strict_switch.py`, `verify_controller_states.py` | **Yes** — constant-effort `--switch-timeout` CLI defect repair | instrumentation |
| 15 | Contact-liveness sidecar | recorder alive + publisher observable at start and horizon end | `contact_liveness.py` | **Yes** — H25's empty-file INDETERMINATE | instrumentation |
| 16 | Retention threshold | ‖Δp_go‖ ≤ 0.005 m | `config/scene.yaml: thresholds.post_lift_slip_max_m` | **Yes** — existing M3 Gazebo ground-truth relative-slip criterion, reused without tuning | **physical, validated** |
| 17 | Force distributional rule | both settled windows' P95 **and** P99 ≤ same-run PID P95/P99, per fingertip per component | H25 README, "Force criterion review" | **Yes** — written rationale; explicitly replaced a rejected max-based rule | physical (intent) |
| 18 | Force maxima | mandatory anomaly reporting with timestamp / duration / contact count / penetration / ratios — **not** by itself FAIL | H25 README | **Yes** — DART maxima are single-sample, sampling-rate-sensitive impulses | reporting |
| 19 | Force indeterminacy | missing samples / timestamps / contact identity → INDETERMINATE | H25 README | **Yes** | instrumentation |
| 20 | Bilateral contact continuity | both fingertips in object contact through the horizon | `F3_MEASUREMENT_PLAN.md`; ablation README | **Yes** — "corroborating evidence, not the sole retention metric" | physical |
| 21 | **Quiescence set** | T_go p2p 0.5 mm/10 s; trend 0.05 mm/s; orientation 0.02 rad; all Gazebo gripper joints max\|v\| 0.02 rad/s and position range 0.01 rad | close-hold README ("frozen in the preceding review") | **Not stated there.** Numerically traceable to `gz_settle_pose_windowed` (`eps = 0.5 mm`, 10 s window, built to resolve a 0.05 mm/s creep) — `docs/HANDOFF_M3.md` | **detector-resolution, borrowed** |
| 22 | 6–12 Hz method + 80 %/90 %/50 % bands | zero-phase bandpass + Hilbert, Welch band power | ablation README | **Yes** — but declared for the **zero-effort ablation**, not for hybrid hold | analysis method |

### Two documentation defects recorded here

- Criterion 10 has **no derivation anywhere in the repository**, yet it decided
  the P12.5 experiment verdict.
- **"P0" is overloaded.** In `HANDOFF.md` it is the F2 settled-object world
  checkpoint of the P0/P1/P2/P3 sequence. In the P12.5 README ("H0/P0
  semantics") it means the P12.5 analogue of H0. The harness writes
  `H0_<RUN_ID>.txt` regardless of controller, so operationally **H0 ≡ P0**.
  Future documents must say which is meant.

### Physical vs instrumentation classification

- **Physical / scientific:** 2, 11, 16, 17, 18, 20, and the substance behind 21 and 22.
- **Instrumentation / evidence quality:** 1, 4, 5, 6, 9, 10, 12, 13, 14, 15, 19.
- **Protocol convention:** 3, 7, 8.

---

## 2. Three distinct coverage semantics

The previous criteria conflated three questions under a single word,
"coverage". They are not the same question and they do not have the same
failure mode.

| Semantic | Question it answers | Correct evidence | Correct rule | Failure means |
|---|---|---|---|---|
| **Controller-law validity** | Did the controller compute the frozen law? | every **received** row | per-sample exact reconstruction (`abs_tol` 1e-10), controller `active`, single latched `q_ref` | **FAIL** — the controller did something else |
| **Physical-state validity** | Is the robot/object trajectory adequately observed? | native `gz_joint` / `master` / `pose` / contact streams | window coverage ≥ 90 % of span, no gap > half the shortest analysis window | **INDETERMINATE** — the physics cannot be measured |
| **Diagnostic-transport completeness** | Did the redundant echo of the computation arrive intact? | terms-stream density | report delivered fraction and gap distribution; **INDETERMINATE only if the native streams cannot cover the loss** | **WARNING**, escalating only when unbacked |

Controller-law validity is a **per-sample** property. It cannot be degraded by
the number of samples that arrived — a lost message removes a witness, it does
not alter the computation that was performed and applied. Physical-state
validity is a **coverage** property and genuinely does depend on sample
density. Diagnostic-transport completeness is a property of ROS 2 middleware
delivery and of nothing else.

Applying a coverage rule to establish law validity, as criterion 10 did, is a
category error.

---

## 3. Why the 0.004001 s DiagnosticArray gap is instrumentation completeness

### What the number is

No document states its derivation, but it is reconstructible by arithmetic:
`ur5e_robotiq_description/config/controllers.yaml` sets `update_rate: 500`, so
the nominal publication period is 2.000 ms, and

```
0.004001 s = 2 x 2.000 ms + 1 us
```

It encodes "at most one consecutively missed publication", with 1 µs of
floating-point slack. That is a defensible **transport-completeness** bound. It
is not a bound on anything physical. The `DiagnosticArray` topic carries an
echo of a computation the controller has already performed and written to the
command interface; it is not an independent observation of the robot.

### Three structural problems, independent of any run's outcome

1. **Internally inconsistent with criterion 9 in the same file.**
   `validate_h25_horizon.py` accepts ≥ 9000 of ~10001 rows — up to 1000 missing
   messages, 10 % loss. The spacing rule on the next line rejects **2** missing
   messages. The count rule states an intent of 90 % tolerance; the spacing rule
   silently overrides it with ~99.98 % tolerance. Both cannot be the design intent.

2. **One number applied to three different transports.** `/joint_states` at
   500 Hz (criterion 5) and the controller's own `DiagnosticArray` (6, 10) have
   different publishers, QoS and loss characteristics.

3. **Stricter than the rule guarding the physical streams — inverted.**
   `validate_stream_coverage.py` computes `max_spacing_s`, records it, and
   **never tests it**. In run `20260823_172848_26905` the native pose stream
   passed with a **17 ms** gap — four times the bound that disqualified the
   diagnostic stream. The streams carrying the physics have effectively no gap
   requirement; the stream carrying a redundant echo has the strictest one.

A further consequence: had P12.5 lost 900 *scattered* messages it would have
passed criterion 9 and 10 both; losing 2 *adjacent* ones failed it. A rule whose
verdict depends on the clustering rather than the quantity of lost redundant
messages is not measuring evidence sufficiency.

### What actually happened in `20260823_172848_26905`: nothing was lost

Two intervals of **5.000 ms** (83.429 → 83.434 and 85.433 → 85.438) exceeded the
bound. The full gap histogram over the horizon, in milliseconds, is:

```
0 ms:  5    1 ms: 36    2 ms: 9920    3 ms: 34    4 ms: 3    5 ms: 2
```

10000 intervals, summing to exactly 20.000 s, across **10001 received rows
against 10001 expected** (500 Hz × 20.000 s + 1). **No `DiagnosticArray` message
was lost.** The long gaps are publication-timestamp jitter — the recorder stamps
each row from the most recent `/clock` sample, whose own maximum deviation from
the message stamp was 2 ms — and every 3–5 ms gap is compensated by an adjacent
0–1 ms gap.

Criterion 10 therefore did not reject the P12.5 experiment over missing data. It
rejected it over **±3 ms of jitter in how a redundant echo was time-stamped**,
while the row count was exact and every one of the 10001 rows independently
reconstructs the frozen law, with the controller `active` throughout, one latched
`q_ref`, and zero saturation.

### The loss that did not occur would have been covered anyway

The native Gazebo joint stream independently covers every missing instant:

- `gz_joint`: 20021 samples over the horizon, **1.000 ms maximum gap** — twice
  the diagnostic stream's resolution, with no gap anywhere.
- Agreement across the 10001 co-timestamped rows: `|Δq|` mean 3.6e-07
  (max 2.5e-04); `|Δq̇|` median 2.5e-10, p95 4.8e-10, p99 5.0e-10. 0.16 % of
  rows (16 of 10001) differ by more than 1e-3 in velocity, maximum 6.25e-02 —
  a small discrepant population, none of it at the gap instants.

Because the law is a pure static function of `(q_ref, q, q̇)` with `q_ref`
constant and latched, τ at the eight unwitnessed instants is reconstructible
from the native stream alone:

```
gap 83.429 -> 83.434                gap 85.433 -> 85.438
  83.430  tau = +0.385335745          85.434  tau = +0.646335849
  83.431  tau = +0.302185097          85.435  tau = +0.681685951
  83.432  tau = +0.341331779          85.436  tau = +0.648528685
  83.433  tau = +0.358750814          85.437  tau = +0.683875434
```

All eight are bounded, continuous with their recorded neighbours, inside the
horizon range [0.232, 0.821] N·m, and nowhere near the ±50 N·m saturation.
Nothing happened in those 8 ms.

**Conclusion.** The two gaps invalidate neither controller-law validity nor
physical-state validity, and — the row count being exact — they do not even
establish a transport loss. The recorded INDETERMINATE is a correct application
of the frozen rule, and the rule measures the wrong thing.

---

## 4. Why the strict force-percentile comparison operates below the measured reference uncertainty

### The rule's own stated intent

The H25 README's "Force criterion review" rejected an earlier rule — "any
instantaneous post-switch force maximum above the same-run PID maximum is FAIL"
— on the explicit ground that "Gazebo/DART contact maxima are single-sample,
sampling-rate-sensitive impulses", and that such a rule "would conflate an
impulse/anomaly detector with loaded-grasp stability". The replacement uses the
unchanged PID controller as the empirical reference and "adds no absolute force
threshold or arbitrary multiplier". That design intent is correct and is kept.

### The defect

`P95_hold > P95_PID` with no tolerance treats a point estimate drawn from
~2000 serially-correlated samples as if it were exact. It therefore
reintroduces the very sensitivity the rule was written to remove, one order
statistic down.

Measured from the recorded samples (moving-block bootstrap, block length 100,
honest under the strong serial correlation of a ~1 kHz contact stream):

| | P12.5 left normal | H25 left normal |
|---|---|---|
| PID reference P95 | 13.2174 N | 13.1285 N |
| adjacent order-statistic spacing at the 95th pct | 0.0018 N | 0.0077 N |
| block-bootstrap 95 % CI on the reference | [13.093, 13.316], width **0.2233 N** | [13.036, 13.245], width **0.2015 N** |
| **within-condition control** — P95 of the PID baseline's first half vs its second half, same controller, same window | **0.0282 N (0.21 %)** | **0.0124 N (0.09 %)** |
| the recorded exceedance | **+0.0118 N (+0.09 %)** | +0.0124 N (+0.09 %) |

**The rule's discrimination threshold lies below its own within-condition
control.** Splitting the *unchanged PID controller's own baseline* in half
produces a larger P95 difference (0.0282 N) than the exceedance that failed
P12.5 (0.0118 N). For H25 the two are numerically identical to three decimals.
The exceedances are 0.053× and 0.062× the width of the 95 % CI on the reference
they are compared against.

This is the third instance of a failure shape this project has already
diagnosed twice: `stall_velocity_threshold: 0.001` was "set below this sim's
actual noise floor, not just tuned conservatively", and `gz_settle_pose`'s `eps`
could not distinguish rest from creep.

### The correction, and the proof that it is not tuning

Band = `max(within-condition half-window control, half the block-bootstrap 95 %
CI on the reference)`, both computed from that run's own PID baseline. Applied
identically to all 16 comparisons (2 fingertips × normal/tangential × P95/P99 ×
S1/S2) of both runs:

| | strict rule | banded rule |
|---|---|---|
| **P12.5** | 1/16 FAIL | **0/16 FAIL** |
| **H25** | 7/16 FAIL | **1/16 FAIL** — right normal P95 S2, **+0.697 N (+4.63 %)**, 1.353× the CI width |

The band clears every sub-resolution artifact in both runs while **preserving
H25's one genuine force excursion**. A correction that made both runs pass would
be tuning. This one does not.

---

## 5. Why the borrowed close-and-hold quiescence set is not an appropriate hybrid-hold gate

### Provenance: a detector resolution floor, used as an acceptance bound

`eps = 0.5 mm` over a 10 s window is `gz_settle_pose_windowed`'s **detector
sensitivity floor**, chosen in `docs/HANDOFF_M3.md` precisely because
"0.05 mm/s × 10 s = 0.5 mm, no longer under `eps`". Its semantics are *"motion
below this is indistinguishable from rest by this detector"*. Using it as an
acceptance bound inverts the meaning: it requires an actively-servoed,
contact-loaded assembly to be **unmeasurable**, not to be **stable**.

Classification: the numerical correspondence and shared source document make
this **likely**; the close-hold README cites only "the preceding review", so it
is not **confirmed**.

### The velocity bound is measurably unusable, independent of provenance

Applied to the **unchanged PID controller's own baseline** in run
`20260823_172848_26905` (65.630–67.630 s), Gazebo ground truth:

| joint | n | median \|v\| | p95 | p99 | max | % > 0.02 rad/s |
|---|---|---|---|---|---|---|
| left_knuckle | 2001 | 0.0834 | 0.1000 | 0.1000 | 0.1014 | **96.7 %** |
| left_inner_knuckle | 2001 | 0.0604 | 0.1713 | 0.2143 | 0.2504 | **83.6 %** |
| right_knuckle | 2001 | 0.1000 | 0.1000 | 0.1000 | 0.1000 | **100.0 %** |
| right_inner_knuckle | 2001 | 0.1000 | 0.1171 | 0.2123 | 0.2414 | **99.5 %** |

Two independent reasons this channel cannot support a 0.02 rad/s bound:

- **Saturation (confirmed).** All six linkage joints carry
  `<limit velocity="0.1">` — the deliberate 2026-08-10 override in
  `ur5e_robotiq_description/urdf/vendor/robotiq_2f_85_macro.urdf.xacro`
  (lines 460, 474, 505, 516). The right knuckle readback sits at exactly
  0.1000 rad/s for 100 % of PID-baseline samples, the master for 96.7 %. A
  channel pinned at its limit cannot resolve differences at 0.02 rad/s under
  any controller.
- **Documented noise floor.** At a genuine contact-loaded physical stall,
  `docs/HANDOFF_M3.md` records median 0.008, **p95 0.025**, p99 0.245,
  max 0.264 rad/s. The measured p95 at rest already exceeds the bound. The
  0.22–0.25 rad/s follower peaks in both H25 and P12.5 sit exactly on that
  historically measured spike population — they are the simulator's known
  noise, not controller behaviour.

### The translation bound fails the same test, more mildly

The unchanged PID baseline shows **0.4745 mm** peak-to-peak over **2 s**,
against a bound of 0.5 mm over **10 s**. The reference controller does not meet
it either. A criterion the incumbent fails cannot discriminate candidates.

### What survives

**Trend ≤ 0.05 mm/s** and **orientation ≤ 0.02 rad** are dimensionally
appropriate, unsaturated, and largely met by both runs (trend 0.0060–0.0964
mm/s; orientation 0.003–0.009 rad). They are retained. The translation
peak-to-peak and joint-velocity bounds are dropped.

### The zero-effort ablation bands are also mismatched

The 80 % / 90 % / 50 % classification thresholds (criterion 22) were declared to
classify a **zero-effort ablation** — whether an oscillation is
controller-sustained or passive once the actuator is switched off. They are not
acceptance criteria for a controller that is deliberately still active.
Reporting the ratios is correct; applying the labels is a category error. No
harm has resulted, because they have not yet been applied.

---

## 6. Corrected G1–G10 readiness criteria

Nothing below is loosened because a run missed it. Each change carries an
independent justification from prior project semantics, measurement resolution,
physics, or instrumentation behaviour.

| # | Criterion | Rule | Verdict on failure | Provenance / justification | Changed? |
|---|---|---|---|---|---|
| **G1** | Retained grasp | bilateral fingertip/object contact continuous through the evaluated interval; max gap ≤ 50 ms | FAIL | criterion 20 | no |
| **G2** | Bounded wrist-relative motion | window-median ‖Δp_go‖ from the pre-lift reference ≤ **5.000 mm**, **and** maximum instantaneous ‖Δp_go‖ ≤ 5.000 mm | FAIL | `thresholds.post_lift_slip_max_m`, criterion 16 | adds the instantaneous form, strictly **stronger** |
| **G3** | Controller-law validity | every received row reconstructs the frozen law at `abs_tol` 1e-10, controller `active`, single latched `q_ref` | FAIL | criterion 11 | no |
| **G4** | No divergent joint/controller behaviour | per-window master and follower **position range** and **means** stationary between comparable windows (no monotonic drift); commanded effort non-saturating | FAIL | replaces criterion 21's velocity bound | **yes** — §5: the velocity channel is saturated at the frozen 0.1 rad/s limit and its measured noise p95 (0.025) already exceeds the 0.02 bound. Position range is neither saturated nor at its noise floor |
| **G5** | Limit-cycle suppression | 6–12 Hz **band power** relative to the same run's PID baseline, on T_go.X, master velocity, and summed fingertip normal/tangential force — reported for all four; used to **rank** candidates | ranking, not pass/fail | criterion 22's *method*, not its ablation labels | **yes** — §5: the 80/90/50 bands were declared for a zero-effort ablation, and no validated absolute band exists |
| **G6** | Contact-force acceptability | both settled windows' P95 and P99 ≤ same-run PID value **+ band**, band = `max(within-condition half-window control, half the block-bootstrap 95 % CI on the reference)`, both from that run's own PID baseline | FAIL | criterion 17's stated intent | **yes** — §4: derived from the reference's own measured variance; adds no absolute threshold and no arbitrary multiplier; still fails H25 |
| **G7** | Force-anomaly reporting | maxima with timestamp, contact count, penetration, ratio to same-run PID maximum | mandatory report | criterion 18 | no |
| **G8** | Physical-state coverage | native pose / joint / contact streams ≥ 90 % of each analysis window, no gap > half the shortest analysis window | INDETERMINATE | §2 | **yes** — `validate_stream_coverage.py` currently applies **no** gap test at all; strictly **stronger** |
| **G9** | Diagnostic-transport completeness | report delivered fraction and gap distribution | WARNING; INDETERMINATE only if the native streams cannot cover the loss | §2, §3 | **yes** — replaces criterion 10, on the internal inconsistency with criterion 9, the inverted strictness versus G8, and the demonstrated reconstructibility |
| **G10** | Prerequisites | F2 gate, strict switch, exclusive ownership, contact liveness | INDETERMINATE | criteria 1, 14, 15 | no |

Retained unchanged from the previous set and not restated above: criterion 2
(0.070000 rad pre-close, explicitly not to be tuned), criterion 19, and the
trend/orientation components of criterion 21.

**Freezing requirement.** G1–G10 must be frozen in writing, with these
derivations, before they gate any experiment. The absence of any recorded
derivation for criterion 10 is what made this audit necessary; that failure mode
must not recur.

---

## 7. Scope and standing limitations

- This audit rests on n = 1 per gain, Scene A only, Scenes B–D not run.
- It establishes nothing about behaviour under acceleration. Both hybrid-hold
  runs measured constant-load static hold only.
- The 0.16 % velocity-discrepancy population between the diagnostic and native
  streams (§3) is recorded but not explained.
- P12.5's advantage on G5 and G6, and its ~2× disadvantage in static positional
  wander, are both measured at constant load and neither generalises to the lift
  transient without evidence.

## 8. Related documents

- `docs/F3_MEASUREMENT_PLAN.md` — the frozen F3 lift measurement plan (G0/L1/L2,
  the 5 mm threshold, verdict semantics).
- `docs/F3_P12_5_LIFT_PLAN.md` — the predeclared next experiment, which applies
  G1–G10.
- `evidence/f3_h25_hybrid_hold_infrastructure/README.md` — force-criterion review.
- `evidence/f3_p12_5_hybrid_hold/README.md` — P12.5 experiment definition.
- `evidence/f3_close_hold_scene_A_20260823_132557/README.md` — origin of the
  borrowed quiescence set.
- `docs/HANDOFF_M3.md` — `gz_settle_pose_windowed` and the measured
  gripper-velocity noise floor.
