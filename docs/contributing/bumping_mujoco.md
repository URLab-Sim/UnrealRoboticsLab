# Bumping MuJoCo

How to move URLab onto a newer MuJoCo.

The engine is a submodule, `third_party/MuJoCo/src`. Everything URLab derives
from it — the generated document profile, the compiled-model goldens, the
hand-written translation of MuJoCo's own reader — is pinned to that commit, and
each derived thing has a different failure style when the pin moves. This page
is ordered so the loud failures happen before the silent ones.

!!! info "Where ProtoSpec lives"

    **The plugin's `protospec/` is the live tree.** The generator, the overlay,
    and the SDK that `Source/URLab/*/MuJoCo/Gen` is compiled against are all
    there, in the plugin, tracked in the plugin's history.

    The MuJoCo submodule contains a directory also called `protospec/`. It is
    **not consumed** — `.gitmodules` says so, and that is the authoritative
    statement — but it still holds a runnable generator with its own, older
    overlay. Running that copy produces plausible output from the wrong tables.
    Check which tree you are in before running anything: the live one is at the
    plugin root, not under `third_party/`.

    What the submodule *does* supply is the grammar and the C API:
    `src/xml/mjcf.schema`, `doc/generate/mjcf_schema.py`, and the headers under
    `include/mujoco/`. The generator reads all of them straight from the
    submodule working tree (`frontend.py`, `mujoco_src()`), which is why the
    submodule has to be at the new commit before anything is regenerated.

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

Read it for MJCF changes too, not just API changes. Sections 6 and 7 below are
the parts of the bump the tooling cannot do for you, and the changelog is the
only advance warning either of them gets.

### 2. Move the submodule and stage the gitlink

```bash
cd third_party/MuJoCo/src
git fetch origin && git checkout <target>
cd ../../..
git add third_party/MuJoCo/src
```

Stage it now. Two different checks read the gitlink from two different places
and both have to agree:

- `third_party/MuJoCo/build.ps1` runs `git submodule update --force`, which
  follows the **index**. Build with the gitlink unstaged and it resets your
  submodule back under you.
- UBT's drift check (`URLab.Build.cs`) reads `git ls-tree HEAD`, so it compares
  against the **committed** gitlink and refuses the editor build with
  `MuJoCo submodule drift: URLab expects SHA ...`.

So the gitlink is staged before the third-party build and committed before the
UE build. Step 5 opens that commit for exactly that reason.

### 3. Rebuild the third-party install, then ProtoSpec

Order matters. The generator parses the **submodule's** headers while the
build links the **staged install**, so an install that is one version behind
produces a profile that compiles against nothing.

```bash
cd third_party && ./build_all.ps1        # or ./build_all.sh
cd ../protospec && ./build.ps1           # or ./build.sh — NOT part of build_all
```

`protospec/build.ps1` is deliberately not a step of the MuJoCo build, so it is
also easy to forget. Confirm both landed:

```bash
ls third_party/install/protospec/sdk/protospec/classes.h
grep '#define mjVERSION_HEADER' third_party/install/MuJoCo/include/mujoco/mujoco.h
```

If `third_party/install/protospec/` is missing, `URLab.Build.cs` defines
`URLAB_PROTOSPEC=0`, the whole generated profile compiles out, and the build
**succeeds** with one warning in a long log. See
[Prove the pipeline is live](#9-prove-the-pipeline-is-live).

### 4. Regenerate and answer the gates

```bash
cd protospec && uv run pytest && uv run python -m protospec_gen.emit --check
cd .. && ./Scripts/regen_ue_profile.ps1        # or .sh
```

A schema change moves the generated profile, so expect `Source/URLab/*/MuJoCo/Gen`
to change. It is checked in; commit it.

This is the strongest part of the bump: the gates fail **by name**, and each
failure names the exact overlay entry to edit or delete. Two families:

**Removal and change.** An overlay entry naming an element, enum, keyword or
attribute the schema no longer declares; a `reading=custom` / `writing=custom`
facet with no handler and no waiver; an `ATTR_TYPE_OVERRIDES` row correcting a
declaration that has changed under it; a moved or retyped `mjs*` field. All in
`protospec/protospec_gen/overlay.py` and `overlay_ue.py`.

**Addition.** Three classes of upstream addition would otherwise pass every
removal gate and land silently wrong, so each has its own gate
(`frontend.py`):

| Addition | If unclassified | Gate |
|---|---|---|
| a repeatable child of an interleaved section (`body`, `tendon`, `spatial`, `equality`, `actuator`, `sensor`) left out of its `INTERLEAVE` row | written as a separate list after the ordered one, shifting every id in that family | strict, no waiver |
| a new angle-valued attribute not in `ANGLE_ATTRS` | read as radians whatever the document says: wrong by 57.3x | heuristic; waive in `NOT_ANGLE` |
| a new dynamic reference not in `TARGET_FROM` | a plain string no referrer scan or rename fixup can see | heuristic; waive in `NOT_TARGET` |

The two heuristic gates fire only on attributes that are *new*, which is
computed against `protospec/protospec_gen/classified_attrs.json`. Once every
new attribute is either classified or waived with a reason, refresh that
baseline and commit it with the rest:

```bash
cd protospec && uv run python -m protospec_gen.frontend --update-baseline
```

Refresh it **after** answering the gates, never before: the baseline is what
makes "new" mean anything, and refreshing first is how an unclassified
attribute becomes permanently invisible.

### 5. Commit the gitlink, the profile and the baseline

UBT reads the **committed** gitlink, so this happens before the editor build:

```bash
git add third_party/MuJoCo/src Source/ protospec/
git commit -m 'Bump MuJoCo to <SHA> and ...'
```

Amend this commit as the rest of the bump lands — see [Finish the
commit](#11-finish-the-commit).

### 6. Re-read the upstream citations

About 2,000 lines of `Source/URLab/Private/MuJoCo/Spec/` hand-mirror MuJoCo's
own MJCF reader, so that URLab's component tree produces the same `mjSpec` the
reader would have produced from the equivalent document. Those transcriptions
cite upstream by file and line. Nothing checks that a citation still describes
the code it names, so this step is manual and it is the step most likely to be
skipped.

Regenerate the review list rather than trusting this one — the grep is the
list, so it cannot rot:

```bash
grep -rn 'xml_native_reader\.cc\|user_objects\.cc\|user_model\.cc\|user_mesh\.cc\|user_api\.cc' Source/URLab
```

At the time of writing that is six files: `MjSpecWriteHooks.cpp` (five sites),
`MjSceneSpec.cpp`, `MjFromtoFold.cpp` and `.h`, `MjSpecBuildContext.h`,
`MjAssetSink.h`. For each, open the cited upstream location in the **new**
submodule and confirm the rule still reads the way the comment says. A rule
that moved and was not followed produces a model that compiles and is wrong.

### 7. Retest the behavioral assumptions

These are upstream behaviours URLab depends on that no gate expresses. Each has
a known failure shape, so each is checked deliberately:

- **The compile-before-serialize workaround** (`MjSceneSpec.cpp`,
  `SaveDebugArtifacts`). MuJoCo's writer serializes a *compiled* spec, and a
  spec copy carries no compile with it; handing `mj_saveXMLString` an
  uncompiled copy is an access violation rather than a refusal, so the copy is
  compiled first. If upstream starts refusing cleanly, the extra compile can
  go; if the access violation moves, the crash is in debug-artifact saving.
- **The VFS case-insensitive basename fallback** (`MjSceneSpec.cpp`,
  `MjAssetSink.h`). The whole asset-namespacing scheme is built on MuJoCo
  resolving a VFS entry by basename, case-insensitively.
- **Derive-then-prefix order for unnamed assets** (`MjSceneSpec.cpp`). MuJoCo
  derives an unnamed asset's name from its file before URLab's prefix is
  applied; a change in order renames every unnamed asset in a scene.
- **`mjs_attach` corruption on failure.** A failed attach leaves the target
  spec unusable. URLab aborts and discards the whole build on failure and never
  retries. If upstream ever makes attach failure recoverable, that path can be
  simplified — but do not weaken it on assumption.
- **`mjCModel::CopyList` silently dropping unresolved-reference elements**
  (cited in `MjSpecWriteHooks.cpp`).

### 8. Build, test, and run the nets

```powershell
.\Scripts\build_and_test.ps1 -Engine 'C:\Program Files\Epic Games\UE_5.7' `
                             -Project 'C:\path\to\your.uproject'
```

The wrapper runs the profile drift gate first and prints the summary block the
PR template wants. **Confirm you saw the gate line**,
`>>> Profile drift gate: regen_ue_profile.ps1 -Check`. If it is absent the
script prints a `WARNING:` naming what it could not find — `uv` off PATH, the
generator missing — and continues without gating. Exit 4 is drift.

Then run the corpus net, which `build_and_test` does **not** run for you. It
round-trips every model in MuJoCo's own corpus through URLab's reader and
writer and field-diffs the resulting `mjModel` against a stock load:

```powershell
.\protospec\corpus_net.ps1        # or ./protospec/corpus_net.sh
```

It exits non-zero on anything outside its recorded allowed-failure list
(`protospec/tools/corpus_net.py`). A new entry in that list is a decision, not
a formality.

### 9. Prove the pipeline is live

Do this before you believe a green build. A green build and a green suite are
not evidence the generated profile is present: with `URLAB_PROTOSPEC=0`
everything that would have failed compiles out instead, and the automation
suite's ProtoSpec tests go with it.

1. **`third_party/install/protospec/` exists** and `URLAB_PROTOSPEC` is 1. The
   build prints a `URLab: ProtoSpec is not installed ...` line when it is 0;
   the absence of that line plus the presence of the directory is the check.
2. **The drift gate executed** — its own output line, above. A skipped gate and
   a passing gate have the same exit code.
3. **A representative imported model still loads and simulates** in the editor.

### 10. The goldens

`Content/TestData/goldens/` holds compiled `mjModel` files (`.mjb`).
`mj_loadModel` validates the MuJoCo version in the `.mjb` header, so **any
release bump makes every golden fail to load**.

`Content/TestData/goldens/CAPTURE.json` records the MuJoCo version and
submodule SHA the goldens were captured at, and the golden test reads it before
loading anything, so a version bump gives you an explanatory failure rather
than a message indistinguishable from file corruption. When it fires:

1. Confirm the live compile-parity check is green first. It compares URLab's
   compiled output against `mj_loadXML` of the same authored file on every run,
   with no recording involved, so it is the thing that says the new output is
   *correct*. Recapturing goldens without it records whatever the code does
   that day, regressions included.
2. Delete the goldens (capture mode writes only files that are **absent**, so
   recapture is delete-then-run).
3. Re-run the suite with `URLAB_CAPTURE_GOLDENS=1`.
4. Review the diff. `CAPTURE.json` is rewritten by capture mode; the `.mjb`
   files are binary, so the live check in step 1 is the review.

A golden that changes when the MuJoCo version did **not** change is a
regression, and stops the bump.

### 11. Finish the commit

The whole bump is **one commit**: gitlink, regenerated `Gen/`, the classified
baseline, any API migrations, any overlay edits, recaptured goldens, any tests.
An intermediate state where the gitlink moved but call sites still use old
signatures does not compile, breaks `git bisect`, and leaves the tree
unbuildable. Step 5 opened that commit because UBT would not build without it;
amend the rest into it rather than stacking follow-ups.

```bash
git add -A && git commit --amend --no-edit
```

## Checklist

- [ ] Changelog slice read, including MJCF changes.
- [ ] Gitlink staged **before** the third-party build, committed **before** the UE build.
- [ ] `third_party/build_all.*` **and** `protospec/build.*` both re-run; `third_party/install/protospec/` exists.
- [ ] `uv run pytest` and `emit --check` green inside `protospec/`.
- [ ] Every overlay gate answered by editing a table, not by loosening a gate.
- [ ] Every new attribute classified or waived with a reason; `classified_attrs.json` refreshed **after**.
- [ ] `Scripts/regen_ue_profile.*` re-run; `Gen/` changes committed.
- [ ] Every upstream citation re-read against the new sources (regenerate the list by grep).
- [ ] The behavioral assumptions in step 7 retested.
- [ ] `build_and_test.*` printed the drift-gate line and exited 0.
- [ ] `corpus_net.*` run, and its allowed-failure list unchanged or deliberately changed.
- [ ] `URLAB_PROTOSPEC` is 1 (step 9).
- [ ] Goldens: live parity check green first, then recaptured; `CAPTURE.json` updated.
- [ ] A representative imported model still loads and simulates in the editor.

## Related

- [Architecture: the mjSpec pipeline](../architecture_mjspec.md): what the
  gates, goldens and hooks in this page are protecting.
- [Building from Source](building.md): dependency drift checks and the build gate.
