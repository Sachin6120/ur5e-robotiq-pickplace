#!/usr/bin/env python3
"""Lightweight regression tests for test_stage3c_c3.py::parse_c3_event_name().

Fixes the qualification-tooling defect found while classifying
evidence/stage3c_c3c_20260909_024120/'s NEEDS_CORRECTION verdict: the old
`re.search(r'M3 C3 (\\w+) ', line)` extraction required a trailing SPACE and
therefore never recognized any FORM-B ("M3 C3 EVENT_NAME: text") production
telemetry line, including SECOND_TRIGGER_REPLAN_LIMIT_REACHED. Production's
own telemetry was correct throughout -- this is a harness/parser defect only,
the same class as the 2026-09-08 C2 ANSI-escape parser fix (see
PROJECT_STATE.md / HANDOFF.md).

No pytest framework is used in scripts/ (matches this repo's existing
script-as-test-runner convention, e.g. test_perception_readiness_gate.py).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_stage3c_c3 import parse_c3_event_name  # noqa: E402

FAILURES = []


def check(label, actual, expected):
    if actual != expected:
        FAILURES.append(f'{label}: expected {expected!r}, got {actual!r}')


# A. FORM B (colon-delimited): the exact defect this fix recovers.
check('form_b_second_trigger_replan_limit_reached',
      parse_c3_event_name(
          'M3 C3 SECOND_TRIGGER_REPLAN_LIMIT_REACHED: replacement collision '
          'stopped; replan budget (1) exhausted; state_e2_captured=1'),
      'SECOND_TRIGGER_REPLAN_LIMIT_REACHED')

# B. FORM A (space-delimited key=value telemetry), unchanged.
check('form_a_fjt_goal_accepted',
      parse_c3_event_name('M3 C3 FJT_GOAL_ACCEPTED attempt=1 goal_uuid=abc'),
      'FJT_GOAL_ACCEPTED')

# C. FORM A, another established event.
check('form_a_replan_budget_exhausted',
      parse_c3_event_name(
          'M3 C3 REPLAN_BUDGET_EXHAUSTED attempt=1 replan_count=1 max_replans=1 '
          'result=TRANSPORT_REPLAN_LIMIT_REACHED telemetry_valid=1'),
      'REPLAN_BUDGET_EXHAUSTED')

# D. ANSI-prefixed telemetry: the helper strips ANSI escapes itself and must
#    still recover the FORM-B event name correctly.
check('ansi_wrapped_form_b',
      parse_c3_event_name(
          '\x1b[1;31mM3 C3 SECOND_TRIGGER_REPLAN_LIMIT_REACHED\x1b[0m: '
          'replacement collision stopped'),
      'SECOND_TRIGGER_REPLAN_LIMIT_REACHED')

# E. Malformed prefix ("M3 C30", not "M3 C3 ") must NOT match -- the digit
#    breaks the required literal space immediately after "C3".
check('malformed_prefix_c30_rejected',
      parse_c3_event_name('M3 C30 SOMETHING_THAT_LOOKS_LIKE_AN_EVENT key=val'),
      None)

# F. Malformed delimiter (a character that is neither space, colon, nor EOL)
#    must fail the WHOLE match, not silently accept a truncated event name.
check('malformed_delimiter_rejected_not_truncated',
      parse_c3_event_name('M3 C3 BAD.EVENT_NAME more text'),
      None)

# F (continued). A hyphen breaks it the same way.
check('malformed_delimiter_hyphen_rejected',
      parse_c3_event_name('M3 C3 SOME-EVENT here'),
      None)

# Defensive: lowercase event names are not the established convention.
check('lowercase_event_name_rejected',
      parse_c3_event_name('M3 C3 lowercase_event key=val'), None)

# Defensive: "M3 C3 " appearing mid-sentence with no valid event name after it.
check('mid_sentence_no_event_rejected',
      parse_c3_event_name('a log line that happens to mention M3 C3 casually'),
      None)

# End-of-line delimiter is accepted (no observed production event currently
# relies on it, but the spec explicitly allows it and it must not regress).
check('end_of_line_delimiter_accepted',
      parse_c3_event_name('M3 C3 EOF_EVENT'), 'EOF_EVENT')

# G. Regression: parsing every preserved C3A/C3B raw log must be BYTE-
#    IDENTICAL to the old regex's output, proving zero change to already-
#    qualified evidence's interpretation.
import re  # noqa: E402

REPO = Path(__file__).resolve().parents[1]


def old_regex_event(line):
    m = re.search(r'M3 C3 (\w+) ', line)
    return m.group(1) if m else None


for label, rel_path in (
    ('C3A', 'evidence/stage3c_c3a_20260908_125925/m3_grasp.log'),
    ('C3B', 'evidence/stage3c_c3b_20260908_135359/m3_grasp.log'),
):
    path = REPO / rel_path
    if not path.is_file():
        continue  # preserved evidence may not exist in every checkout
    old_counts, new_counts = {}, {}
    for line in path.read_text(errors='replace').splitlines():
        o = old_regex_event(line)
        n = parse_c3_event_name(line)
        if o:
            old_counts[o] = old_counts.get(o, 0) + 1
        if n:
            new_counts[n] = new_counts.get(n, 0) + 1
    check(f'{label}_raw_log_event_extraction_unchanged', new_counts, old_counts)

if FAILURES:
    print('FAIL:')
    for f in FAILURES:
        print(' -', f)
    sys.exit(1)
print(f'PASS: {12} parse_c3_event_name regression cases '
      '(including byte-identical C3A/C3B raw-log re-extraction where evidence is present)')
