# Bumping MuJoCo

How to move URLab onto a newer MuJoCo.

!!! danger "`third_party/MuJoCo/src` is URLab's fork, not upstream. A bump is a REBASE, and getting it wrong looks like success."

    The submodule tracks URLab's MuJoCo fork, whose `protospec` branch is upstream
    **plus one directory**: `protospec/`, carrying the MJCF schema front end, the
    generator, and the C++ SDK that `Source/URLab/*/MuJoCo/Gen` is compiled
    against.

    So a bump is: **rebase the fork's `protospec` branch onto the new upstream
    commit, then move the gitlink to the rebased tip.** Never `git checkout` an
    upstream SHA or tag in the submodule.

    Pointing the gitlink at an upstream commit **does not fail**. It fails
    silently, and every signal you would normally trust says the bump worked:

    | What happens | What you see |
    |---|---|
    | `protospec/` vanishes from the submodule | nothing |
    | `third_party/MuJoCo/build.ps1` finds no `src/protospec/lib`, skips it | one skipped line in a long build log |
    | `URLab.Build.cs` defines `URLAB_PROTOSPEC=0` (`URLab.Build.cs:162`) | one warning in a long build log |
    | the entire generated document profile compiles out | **build succeeds** |
    | the drift gate in `build_and_test.{ps1,sh}` needs `protospec_gen`, which is gone, so it skips itself | **no gate failure** |
    | the automation suite's ProtoSpec tests compile out with it | **tests pass** |

    A green build and a green suite are therefore **not** evidence the pipeline is
    present. Verify it explicitly — see [Prove the pipeline is
    live](#prove-the-pipeline-is-live) — every time.

    One more trap in the same family: `third_party/MuJoCo/build.ps1` runs
    `git submodule update --force`, which follows the **index** gitlink. If you
    have moved the submodule but not staged the gitlink, the build resets your
    submodule back, dropping `protospec/`, with the symptoms above. **Stage the
    gitlink before you build.**

    And its mirror image, which fails loudly rather than silently: UBT's own
    drift check (`URLab.Build.cs:401`) reads `git ls-tree HEAD`, so it compares
    the submodule against the **committed** gitlink, not the staged one. Stage
    it and the third-party build works; leave it uncommitted and the editor
    build refuses with `MuJoCo submodule drift: URLab expects <old SHA>`. The
    two checks read different places, so the gitlink has to be both staged and
    committed before the UE build — which is why the commit below happens
    before `build_and_test`, not after it.

## Procedure

### 1. Read the upstream changelog slice

MuJoCo maintains `doc/changelog.rst`. Read every entry between the current pin
and the target before touching code — breaking changes land regularly and the
blast radius is worth knowing up front rather than at link time.

```bash
gh api "repos/google-deepmind/mujoco/contents/doc/changelog.rst?ref=<TARGET>" \
    --jq '.content' | base64 -d
```

Pay particular attention to the **Breaking API changes** admonition in each
release's `General` section, to removed or renamed `mjs_*` entry points, and to
anything about plugin packaging or CMake targets (a DLL-packaging change is what
made the 3.7.0 bump crash URLab silently during module init).

### 2. Rebase the fork

In the fork checkout (not the submodule working tree, if you keep a separate
clone):

```bash
git fetch upstream
git rebase <new-upstream-commit> protospec
```

The branch commits nothing outside `protospec/`, and its own
`tests/test_upstream_untouched.py` enforces that, so the rebase should be
conflict-free. **A conflict means someone edited MuJoCo itself** — that is the
bug to fix, not to resolve away.

Then run the fork's own gates before moving the gitlink:

```bash
cd protospec
uv run python -m protospec_gen.emit --check
uv run pytest
```

A schema change surfaces here first: the overlay's drift gates
(`ATTR_TYPE_OVERRIDES`, `UE_KIND`, `UE_QUAT_ATTRS`, `ANGLE_ATTRS`, the
`reading=custom` / `writing=custom` waivers) fail by name rather than going
stale. Each failure names the exact overlay entry to edit or delete.

### 3. Move the gitlink and stage it

```bash
cd third_party/MuJoCo/src
git checkout protospec && git pull --ff-only     # the REBASED branch
cd ../../..
git add third_party/MuJoCo/src                   # BEFORE building; see the warning above
```

### 4. Rebuild third-party

```bash
cd third_party/MuJoCo
./build.ps1          # or ./build.sh on Linux/macOS
```

Confirm the install carries ProtoSpec:

```bash
ls third_party/install/protospec/sdk/protospec/classes.h
grep '#define mjVERSION_HEADER' third_party/install/MuJoCo/include/mujoco/mujoco.h
```

### 5. Regenerate the UE profile

```bash
./Scripts/regen_ue_profile.ps1        # or .sh
```

A schema change moves the generated profile, so expect `Source/URLab/*/MuJoCo/Gen`
to change. Commit that output; it is checked in.

### 6. Commit the gitlink and the regenerated profile

UBT reads the **committed** gitlink, so this has to happen before the editor
build, not after it:

```bash
git add third_party/MuJoCo/src Source/
git commit -m 'Bump MuJoCo to <SHA> and ...'
```

Amend this commit as the rest of the bump lands. The whole bump stays one
commit — see [Commit](#8-commit).

### 7. Build and test

Use the wrapper, not a bare UBT invocation — it runs the drift gate first and
prints the summary block the PR template wants.

```powershell
.\Scripts\build_and_test.ps1 -Engine 'C:\Program Files\Epic Games\UE_5.7' `
                             -Project 'C:\path\to\your.uproject'
```

The gate exits 4 on generated-profile drift. **Confirm you saw it run**: the line
`>>> Profile drift gate: regen_ue_profile.ps1 -Check` must appear. If it did not,
the submodule is not checked out, which is the failure this page is about.

### 8. Prove the pipeline is live

Do this before you believe the green build. All three must hold:

1. **The submodule carries the fork.**
   ```bash
   ls third_party/MuJoCo/src/protospec/protospec_gen/emit_ue.py
   git -C third_party/MuJoCo/src log --oneline -1
   ```
   The log line must be a fork commit, not an upstream one.

2. **`URLAB_PROTOSPEC` is 1.** It is 0 whenever `third_party/install/protospec`
   is missing (`URLab.Build.cs:152-162`). A clean rebuild prints the warning
   text from that branch when it is off; the absence of that warning, plus the
   presence of the install directory, is the check.

3. **The drift gate actually executed.** Its own output line, above. A skipped
   gate is indistinguishable from a passing one in the exit code.

### 9. Finish the commit

The whole bump is **one commit**: gitlink, regenerated `Gen/`, any API
migrations, any overlay edits, any tests. An intermediate state where the
gitlink moved but call sites still use old signatures does not compile, breaks
`git bisect`, and leaves the tree unbuildable. Step 6 opened that commit
because UBT would not build without it; amend the rest into it rather than
stacking follow-ups.

```bash
git add -A && git commit --amend --no-edit
```

## Checklist

- [ ] Fork branch rebased, not checked out at an upstream SHA.
- [ ] `uv run pytest` and `emit --check` green inside `protospec/`.
- [ ] Gitlink staged **before** the third-party build, committed **before** the UE build.
- [ ] `third_party/install/protospec/` exists after the build.
- [ ] `Scripts/regen_ue_profile.*` re-run; `Gen/` changes committed.
- [ ] `build_and_test.*` printed the drift-gate line and exited 0.
- [ ] `URLAB_PROTOSPEC` is 1 (see step 8).
- [ ] A representative imported model still loads and simulates in the editor.

## Related

- [Building from Source](building.md): dependency drift checks and the build gate.
