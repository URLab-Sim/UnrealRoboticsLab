"""Run the round-trip differential over the MuJoCo corpus and judge the result.

The net itself is ``tests/test_differential.py``: for every corpus model it
parses with ProtoSpec's reader, writes the document back out, loads the written
text with ``mj_loadXML`` and field-diffs that ``mjModel`` against a stock load of
the original. This module is only the verdict, kept in one place so the
PowerShell and shell entry points cannot drift apart in what they accept.

The verdict has two halves, and both must hold:

* every failure is one of the recorded allowed failures below, and
* the pipeline actually ran -- at least ``_PARITY_FLOOR_IDENTICAL`` models came
  back identical. A suite that skipped its subject wholesale reports zero
  failures, and zero failures without the work is not a pass.

Exit codes: 0 the net holds, 1 it does not, 2 the harness could not run at all.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# The two failures this branch inherited and the owner recorded rather than
# scheduled. A third is a regression and stops the net.
#
# Both describe one defect. many_dependencies.xml is a real round-trip fidelity
# defect in the retained reader and writer: it round-trips to a model that will
# not load ("no tree found for constraint 0"). It is a genuine MuJoCo test model
# and not a corpus artifact, and it must not be closed as one. The aggregate
# floor fails because of it and clears when it does.
ALLOWED_FAILURES = frozenset(
    {
        "tests/test_differential.py::test_roundtrip_matches_mujoco"
        "[test/xml/testdata/many_dependencies.xml]",
        "tests/test_differential.py::test_xml_parity_floor",
    }
)


class _Outcomes:
    """Collects one verdict per test from pytest's own reports."""

    def __init__(self) -> None:
        self.failed: set[str] = set()
        self.passed: set[str] = set()

    def pytest_runtest_logreport(self, report) -> None:
        if report.failed:
            self.failed.add(report.nodeid.replace("\\", "/"))
        elif report.when == "call" and report.passed:
            self.passed.add(report.nodeid.replace("\\", "/"))


def main(argv: list[str]) -> int:
    try:
        import pytest
    except ImportError:
        print("corpus net: pytest is not installed; run through uv", file=sys.stderr)
        return 2

    outcomes = _Outcomes()
    pytest.main(
        ["-q", "--no-header", "tests/test_differential.py", *argv],
        plugins=[outcomes],
    )

    # The floor lives with the test that measures it; importing it here keeps one
    # number in the tree rather than two that can disagree.
    module = sys.modules.get("tests.test_differential") or sys.modules.get("test_differential")
    if module is None:
        print("corpus net: the differential module was never collected", file=sys.stderr)
        return 2
    identical = module._STATS["identical"]
    floor = module._PARITY_FLOOR_IDENTICAL

    unexpected = sorted(outcomes.failed - ALLOWED_FAILURES)
    healed = sorted(ALLOWED_FAILURES & outcomes.passed)

    print()
    print("=== corpus net ===")
    print(f"  identical            {identical} (floor {floor})")
    print(f"  allowed failures     {len(outcomes.failed & ALLOWED_FAILURES)} of {len(ALLOWED_FAILURES)}")
    print(f"  unexpected failures  {len(unexpected)}")

    verdict = 0
    if unexpected:
        print("  FAIL: failures outside the recorded set:")
        for nodeid in unexpected:
            print(f"    {nodeid}")
        verdict = 1
    if identical < floor:
        print(f"  FAIL: only {identical} models round-tripped identically, floor is {floor}")
        verdict = 1
    if healed:
        # Not a failure: a recorded failure that now passes is good news, and the
        # net says so rather than staying silent, because the allowed set is then
        # one entry too long and someone has to shorten it.
        print("  NOTE: recorded failures that now pass; shorten the allowed set:")
        for nodeid in healed:
            print(f"    {nodeid}")
    if verdict == 0:
        print("  PASS")
    return verdict


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
