"""Structural gate: ProtoSpec never edits the MuJoCo checkout it lives in.

`src/xml/mjcf.schema` and everything else outside `protospec/` is read-only
source of truth. Anything the schema states wrongly for our purposes is
corrected in :mod:`protospec_gen.overlay`, which is drift-gated so the
correction retires itself when upstream changes the declaration -- there is
never a reason to edit upstream instead.

This asserts that mechanically, on both halves of "edited": uncommitted changes
in the working tree, and changes this branch has already committed relative to
its merge base with upstream. A violation fails here the moment it is
introduced, rather than surfacing as a rebase conflict at the next MuJoCo bump.

ProtoSpec is meant to be usable outside this fork, so the gate skips with a
stated reason when it cannot establish a baseline: no git, not a repository, a
shallow clone (no merge base to diff against), or no upstream remote ref.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

# Candidate upstream branches. Every one that resolves contributes a fork
# point; the tightest of them is the baseline (see :func:`checkout`).
UPSTREAM_REFS = ("origin/main", "origin/master", "upstream/main",
                 "upstream/master")

# Everything except ProtoSpec's own directory, as a git pathspec.
OUTSIDE_PROTOSPEC = (".", ":(exclude)protospec")


def _git(root: Path, *args: str) -> str | None:
    """`git *args` in `root`; None when git is absent or the command fails."""
    try:
        done = subprocess.run(("git", *args), cwd=root, capture_output=True,
                              text=True, timeout=30)
    except (OSError, subprocess.SubprocessError):
        return None
    return done.stdout.strip() if done.returncode == 0 else None


@pytest.fixture(scope="module")
def checkout() -> tuple[Path, str]:
    """(repository root, baseline commit) for the checkout ProtoSpec lives in.

    The baseline is this branch's fork point on upstream. It is taken as the
    *tightest* of the fork points the configured upstream branches give, rather
    than the first ref that happens to resolve, because a fork's own mirror of
    upstream (``origin/main`` when ``origin`` is the fork) can trail the commit
    the branch is really based on. Diffing from the trailing one would report
    every upstream commit in between as a ProtoSpec edit. The tightest fork
    point is the true base, so the diff from it is exactly what this branch
    changed -- which is the question the gate is asking.
    """
    here = Path(__file__).resolve().parent.parent
    root = _git(here, "rev-parse", "--show-toplevel")
    if root is None:
        pytest.skip("not a git checkout (or git unavailable)")
    if _git(here, "rev-parse", "--is-shallow-repository") == "true":
        pytest.skip("shallow clone: no merge base to diff against")
    root_path = Path(root)

    baseline: str | None = None
    for ref in UPSTREAM_REFS:
        if _git(root_path, "rev-parse", "--verify", "--quiet", ref) is None:
            continue
        base = _git(root_path, "merge-base", ref, "HEAD")
        if not base:
            continue
        # Keep `base` when it descends from the best so far, i.e. when it is
        # the later of the two fork points.
        if baseline is None or _git(
            root_path, "merge-base", "--is-ancestor", baseline, base
        ) is not None:
            baseline = base
    if baseline is None:
        pytest.skip(
            f"no upstream ref configured (looked for {', '.join(UPSTREAM_REFS)})"
        )
    return root_path, baseline


def test_working_tree_outside_protospec_is_clean(checkout):
    root, _baseline = checkout
    dirty = _git(root, "status", "--porcelain", "--", *OUTSIDE_PROTOSPEC)
    assert dirty is not None, "git status failed"
    assert not dirty, (
        "the MuJoCo checkout is read-only source of truth, but these paths "
        "outside protospec/ have uncommitted changes:\n" + dirty +
        "\nRevert them. A schema declaration ProtoSpec needs stated "
        "differently belongs in protospec_gen.overlay (ATTR_TYPE_OVERRIDES "
        "and friends), which is drift-gated against the schema."
    )


def test_branch_commits_nothing_outside_protospec(checkout):
    root, baseline = checkout
    changed = _git(root, "diff", "--name-only", baseline, "HEAD", "--",
                   *OUTSIDE_PROTOSPEC)
    assert changed is not None, f"git diff against {baseline} failed"
    assert not changed, (
        f"this branch commits changes outside protospec/ relative to its "
        f"upstream base {baseline}:\n"
        + changed +
        "\nDrop or revert those commits. Corrections to what the schema "
        "declares belong in protospec_gen.overlay, not in the schema."
    )
