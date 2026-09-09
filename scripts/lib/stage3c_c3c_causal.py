#!/usr/bin/env python3
"""STAGE-3C C3C Phase-2 causal-order analysis from PRESERVED evidence only.

Pure, offline, read-only. No simulator, no process launch, no manipulation,
no modification of any evidence file. Everything here is derived from ONE
artifact -- ``m3_grasp.log`` -- and from ONE observer: the log-line order of a
single process (``m3_grasp``), whose lines are emitted by one rcutils logger on
one monotonic wall clock. Ordering authority is that line order; the logger
stamps are only a monotonicity cross-check on the same single clock.

WHY A NEW GATE
--------------
The original reanalysis gate for the Phase-2 causal requirement was::

    phase2_began_after_attempt1_accepted =
        original['c3c_causal']['t_phase2_begin_sim'] is not None

which tests only PRESENCE of a number, not any ordering. That is too weak to
support a formal C3C closeout claim, so this module replaces it with an
explicit ordering computation.

WHAT IS ACTUALLY PROVABLE WITHOUT A RERUN
-----------------------------------------
``PHASE2_TRANSITION_BEGIN`` is a HARNESS event. The preserved evidence
contains no timestamped harness event ledger for it: ``phase1_despawn.log``,
``obstacle_phase2_spawn.log`` and ``gate_phase1_despawn.log`` hold only
``data: true``; ``gz_pose_stream.csv`` is wall-stamped and tracks robot links,
not the obstacle. The only harness-side number is
``qualification_results.json``'s ``c3c_causal.t_phase2_begin_sim``. That value
is in the SAME nominal ROS sim-time domain as m3_grasp's own sim timestamps --
this is deliberately NOT described as a different clock domain -- but it is a
different NODE's observation of that domain, sampled with its own callback
servicing latency, so the two readings are asynchronous snapshots rather than
two points on one observed timeline and cannot order events (see
``harness_clock_lag`` below). It is deliberately excluded from every gate.

What IS available is the transition's footprint inside ``m3_grasp.log``, in
the coordinator's own single-process log. The harness's Phase-2 transition
begins by DESPAWNING the Phase-1/HOLD obstacle, which stops the
``/collision_object`` update stream feeding the PlanningScene; it then
respawns the same-named entity in the Phase-2 region. Both edges are visible
in the C1 monitor's per-tick telemetry:

  * a tick reporting a SMALL ``scene_age_ms`` proves the obstacle update
    stream was STILL LIVE at that line -- i.e. the despawn had not yet
    happened, i.e. ``PHASE2_TRANSITION_BEGIN`` had not yet occurred;
  * a later tick reporting ``scene_stale=1`` proves the stream had stopped;
  * a later ``FUTURE_PATH_INVALID`` naming ``dynamic_obstacle_0`` proves the
    entity was back, in the Phase-2 region, on the attempt-1 path.

The third bullet alone would only show an EFFECT after the goal acceptance,
which does not by itself place the CAUSE after it. The first bullet is what
closes that gap: it is positive evidence, at a log line strictly AFTER the
attempt-1 goal acceptance, that the transition had not yet started.

Hence the ordering this module computes and gates::

    CANDIDATE_VALIDATION_OK
      < FJT_GOAL_ACCEPTED attempt=1
      < last tick still showing a LIVE obstacle stream   (transition not yet begun)
      < first tick showing the stream INTERRUPTED        (despawn observed)
      <= FUTURE_PATH_INVALID naming dynamic_obstacle_0   (Phase-2 respawn observed)

all as line indices in one file from one process. No arithmetic across
separately-observed node clocks is performed, and none is required.
"""
import re

ANSI_ESCAPE = re.compile(r'\x1b\[[0-9;]*m')

# One monitor tick period is ~100 ms (rate_target_hz=10). An obstacle update
# stream that has already stopped cannot yield an age well below one tick
# period, so an age at or under this bound is positive evidence that the
# stream was still being fed at that line. The steady state actually observed
# in the preserved attempt-1 window is 9-12 ms, an order of magnitude below.
LIVE_STREAM_MAX_AGE_MS = 50.0

# The monitor's own staleness authority, echoed on every tick as scene_stale.
_TICK_RE = re.compile(r'M3 C1 TRANSPORT_MONITOR_TICK\b')
_MONITOR_START_RE = re.compile(r'M3 C1 TRANSPORT_MONITOR_START\b')
_FUTURE_INVALID_RE = re.compile(r'M3 C1 TRANSPORT_MONITOR_FUTURE_PATH_INVALID\b')
_LOG_STAMP_RE = re.compile(r'\[(\d+\.\d+)\]')
_PROCESS_RE = re.compile(r'^\[([A-Za-z0-9_.\-]+)\]')


def _clean(line):
    return ANSI_ESCAPE.sub('', line)


def _field(clean_line, key):
    m = re.search(r'\b' + re.escape(key) + r'=("([^"]*)"|(\S+))', clean_line)
    if not m:
        return None
    return m.group(2) if m.group(2) is not None else m.group(3)


def _stamp(clean_line):
    m = _LOG_STAMP_RE.search(clean_line)
    return float(m.group(1)) if m else None


def _process(clean_line):
    m = _PROCESS_RE.match(clean_line)
    return m.group(1) if m else None


def _event(index, line):
    clean = _clean(line)
    return dict(line_index=index, log_stamp=_stamp(clean), process=_process(clean),
                text=clean.rstrip())


def analyze_phase2_causal_order(log, live_stream_max_age_ms=LIVE_STREAM_MAX_AGE_MS):
    """Computes the Phase-2 causal-order proof from a raw m3_grasp.log.

    Returns a dict carrying every anchor line it used (index, log stamp,
    emitting process, raw text) plus the derived booleans, so the result is
    auditable against the preserved log without re-running this code.
    """
    lines = log.splitlines()

    candidate_validation = None
    attempt1_accepted = None
    attempt1_monitor_start = None
    for i, line in enumerate(lines):
        clean = _clean(line)
        if candidate_validation is None and 'M3 C3 CANDIDATE_VALIDATION_OK' in clean:
            candidate_validation = _event(i, line)
        if (attempt1_accepted is None and 'M3 C3 FJT_GOAL_ACCEPTED' in clean and
                _field(clean, 'attempt') == '1'):
            attempt1_accepted = _event(i, line)
        if _MONITOR_START_RE.search(clean):
            attempt1_monitor_start = _event(i, line)  # last one wins = attempt 1's

    result = dict(
        candidate_validation=candidate_validation,
        attempt1_accepted=attempt1_accepted,
        attempt1_monitor_start=attempt1_monitor_start,
        live_stream_max_age_ms=live_stream_max_age_ms,
        # Stated explicitly so the artifact records the methodology, not just
        # the verdict.
        evidence_source='m3_grasp.log raw line order (single process, single logger clock)',
        uses_cross_node_clock_timestamps=False,
        uses_harness_sim_timestamp=False,
    )

    if candidate_validation is None or attempt1_accepted is None:
        result.update(
            ticks_after_attempt1=[],
            last_live_stream_tick=None,
            first_interrupted_tick=None,
            phase2_respawn_evidence=None,
            ordering=None,
            phase2_causal_order_ok=False,
            reason='missing CANDIDATE_VALIDATION_OK and/or FJT_GOAL_ACCEPTED attempt=1',
        )
        return result

    after = attempt1_accepted['line_index']

    ticks = []
    for i, line in enumerate(lines):
        if i <= after:
            continue
        clean = _clean(line)
        if not _TICK_RE.search(clean):
            continue
        age = _field(clean, 'scene_age_ms')
        stale = _field(clean, 'scene_stale')
        ticks.append(dict(
            line_index=i, log_stamp=_stamp(clean), process=_process(clean),
            tick=_field(clean, 'tick'),
            scene_age_ms=float(age) if age is not None else None,
            scene_stale=stale,
            skipped=_field(clean, 'skipped')))
    result['ticks_after_attempt1'] = ticks

    # First tick, after the attempt-1 goal acceptance, whose obstacle data is
    # STALE -- the despawn edge of the Phase-2 transition.
    first_interrupted = next((t for t in ticks if t['scene_stale'] == '1'), None)

    # Last tick BEFORE that interruption whose obstacle data is demonstrably
    # live -- positive evidence, after the goal acceptance, that the
    # transition had NOT yet begun.
    live_candidates = [
        t for t in ticks
        if t['scene_age_ms'] is not None and
        t['scene_age_ms'] <= live_stream_max_age_ms and
        (first_interrupted is None or t['line_index'] < first_interrupted['line_index'])]
    last_live = live_candidates[-1] if live_candidates else None

    # The respawn edge: the obstacle is back, in the Phase-2 region, invalidating
    # the attempt-1 path.
    respawn = None
    for i, line in enumerate(lines):
        if i <= after:
            continue
        clean = _clean(line)
        if _FUTURE_INVALID_RE.search(clean) and 'dynamic_obstacle_0' in clean:
            respawn = _event(i, line)
            respawn['collision_pairs'] = _field(clean, 'collision_pairs')
            break

    result.update(
        last_live_stream_tick=last_live,
        first_interrupted_tick=first_interrupted,
        phase2_respawn_evidence=respawn)

    anchors = [
        ('candidate_validation', candidate_validation),
        ('attempt1_accepted', attempt1_accepted),
        ('last_live_stream_tick', last_live),
        ('first_interrupted_tick', first_interrupted),
        ('phase2_respawn_evidence', respawn),
    ]
    if any(a is None for _, a in anchors):
        result.update(
            ordering=None, phase2_causal_order_ok=False,
            reason='preserved log does not contain every required ordering anchor')
        return result

    indices = [a['line_index'] for _, a in anchors]
    stamps = [a['log_stamp'] for _, a in anchors]
    processes = [a['process'] for _, a in anchors]

    same_process = len(set(processes)) == 1 and processes[0] is not None
    line_order_ok = all(a < b for a, b in zip(indices, indices[1:]))
    # Non-decreasing rather than strictly increasing: two lines emitted inside
    # the same microsecond are ordered by line index, which is the authority
    # here. The stamp check exists only to catch a clock going BACKWARDS.
    stamp_order_ok = (all(s is not None for s in stamps) and
                      all(a <= b for a, b in zip(stamps, stamps[1:])))

    result['ordering'] = dict(
        names=[n for n, _ in anchors],
        line_indices=indices,
        log_stamps=stamps,
        processes=processes,
        same_process=same_process,
        line_order_strictly_increasing=line_order_ok,
        log_stamp_order_non_decreasing=stamp_order_ok)
    result['phase2_causal_order_ok'] = bool(same_process and line_order_ok and stamp_order_ok)
    result['reason'] = '' if result['phase2_causal_order_ok'] else 'ordering anchors out of order'
    return result


def harness_clock_lag(log, harness_t_phase2_begin_sim):
    """Quantifies -- as an OBSERVATION, never as a gate -- why the harness's
    Phase-2 sim timestamp can numerically precede m3_grasp's own sim timestamp
    for the attempt-1 goal acceptance.

    Both numbers are ROS sim time: the SAME nominal time domain, driven by the
    same /clock publisher. The discrepancy is NOT a clock-domain mismatch. It
    is that each number is a different NODE's locally-observed snapshot of that
    one domain, and a ROS node's sim clock only advances when THAT node
    processes a /clock message:

      * m3_grasp's node is spun continuously by a dedicated
        SingleThreadedExecutor thread (m3_grasp.cpp), so node_->now() tracks
        /clock closely. It logs the attempt-1 acceptance sim time directly as
        ``M3 C0 TRANSPORT_EXECUTOR t_fjt_goal_accept=<sim>``.
      * the harness's ``c1b_monitor_node`` is created with
        use_sim_time=True (scripts/test_stage3c_c1b.py) and is serviced by
        ``rclpy.spin_once(monitor, timeout_sec=0.05)`` ONCE per loop
        iteration -- one callback per call -- in a loop that also reads the
        whole log file and sleeps 0.1 s, while a high-rate
        ``/model/dynamic_obstacle/pose`` subscription competes for those same
        single-callback opportunities. Its clock is therefore a LAGGING
        snapshot of sim time.

    So the harness number is a lower bound that trails reality by its own
    servicing latency. Differencing two independently-sampled node-local
    snapshots of sim time measures that latency, not elapsed time between the
    events, so it cannot order them in either direction. Ordering authority
    stays with same-process event/control-flow order. Returns the measured lag
    for the record.
    """
    accept_sim = None
    for line in log.splitlines():
        clean = _clean(line)
        if 'M3 C0 TRANSPORT_EXECUTOR' in clean and 't_fjt_goal_accept=' in clean:
            accept_sim = float(_field(clean, 't_fjt_goal_accept'))
    if accept_sim is None or harness_t_phase2_begin_sim is None:
        return dict(measurable=False)
    return dict(
        measurable=True,
        m3_grasp_t_fjt_goal_accept_sim=accept_sim,
        harness_t_phase2_begin_sim=harness_t_phase2_begin_sim,
        harness_clock_lag_s=accept_sim - harness_t_phase2_begin_sim,
        note=('same ROS sim-time domain, two different nodes observing it; the '
              'harness node consumes /clock one callback per loop iteration and '
              'therefore lags. This is node-local clock-observation latency, NOT '
              'a clock-domain mismatch, NOT a causality violation, and it is not '
              'used by any gate.'))
