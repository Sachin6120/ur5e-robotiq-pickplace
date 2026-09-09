#!/usr/bin/env python3
"""Lightweight pure-function tests for the Stage-3C C3C perception-readiness
Gate A predicate (test_stage3c_c3.py::classify_topic_type_poll), added
during the STAGE-3C C3C harness correction task. No subprocess/ROS mocking:
the predicate is pure (resolved_type, expected_type) -> outcome, matching
this repo's existing script-as-test-runner convention (no pytest framework
in scripts/). Process-death and timeout behavior are exercised live by the
harness itself (Gate A's process-liveness check, Gate B's bounded subprocess
timeout) -- not re-mocked here.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_stage3c_c3 import classify_topic_type_poll, PERCEPTION_EXPECTED_TYPE  # noqa: E402

FAILURES = []


def check(label, actual, expected):
    if actual != expected:
        FAILURES.append(f'{label}: expected {expected!r}, got {actual!r}')


# Unresolved topic/type (not yet published, or ros2 topic type failed):
# WAITING, never a failure while still inside the timeout.
check('unresolved_topic_is_waiting',
      classify_topic_type_poll(None, PERCEPTION_EXPECTED_TYPE), 'WAITING')

# Resolved to exactly the expected production type: graph-ready.
check('expected_type_is_ready',
      classify_topic_type_poll(PERCEPTION_EXPECTED_TYPE, PERCEPTION_EXPECTED_TYPE), 'READY')

# Resolved to a different type: an explicit taxonomy failure, not WAITING.
check('wrong_type_is_mismatch',
      classify_topic_type_poll('std_msgs/msg/String', PERCEPTION_EXPECTED_TYPE),
      'PERCEPTION_TOPIC_TYPE_MISMATCH')

# Empty-string resolution never reaches this predicate in the real code path
# (_resolve_topic_type() normalizes empty stdout to None first), but the
# predicate itself has no None-vs-empty special case, so it correctly falls
# through to MISMATCH rather than silently treating it as WAITING.
check('empty_string_is_mismatch_not_silently_waiting',
      classify_topic_type_poll('', PERCEPTION_EXPECTED_TYPE), 'PERCEPTION_TOPIC_TYPE_MISMATCH')

if FAILURES:
    print('FAIL:')
    for f in FAILURES:
        print(' -', f)
    sys.exit(1)
print(f'PASS: {4} perception-readiness Gate A predicate cases')
