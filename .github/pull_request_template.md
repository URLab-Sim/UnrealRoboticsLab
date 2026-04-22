<!--
Thanks for contributing. A maintainer will review this PR and may run it
locally, but we ask every PR to ship with proof that it builds cleanly
and the full URLab automation suite passes on your machine.
-->

## Summary

- <what changed, 1-3 bullets>

## Motivation

<why this change is needed — the problem it solves or the goal it enables>

## Linked issue

Fixes #  /  Related to #

## Build + test evidence

<!--
Run the helper script from the plugin root and paste its final summary
block below — it captures the build status, pass/total test counts, the
git HEAD the run was made from, and a SHA-256 of the log so the run can
be verified against the attached log if needed.

bash (git-bash, WSL):
  ./Scripts/build_and_test.sh \
      --engine "/c/Program Files/Epic Games/UE_5.7" \
      --project "C:/path/to/your.uproject"

PowerShell:
  .\Scripts\build_and_test.ps1 `
      -Engine  'C:\Program Files\Epic Games\UE_5.7' `
      -Project 'C:\path\to\your.uproject'

Close the Unreal editor before running — live-coding holds the build
mutex and the cmd-line tester silently returns an empty log.
-->

```
=== URLab build+test summary ===
Timestamp : <UTC>
Git HEAD  : <short-sha> (<branch>)
Engine    : <UE_X.Y>
Deps      : mj=<sha7> coacd=<sha7> zmq=<sha7>
Build     : <Succeeded|Failed>
Tests     : <pass> / <total> passed (<fail> failed)
Log sha256: <hash>
================================
```

## Manual verification steps

<!--
What a reviewer should do to spot-check this in the editor:
 - Any MJCF / asset that's useful (attach or link)
 - PIE steps (what to click, what to expect)
 - Before/after screenshots or a short clip for UI / viewport changes
-->

> **Heads-up:** URLab avoids shipping `.uasset` files where possible so the plugin stays portable across UE versions (engine content is loaded at runtime instead). If your change adds one, call it out in the summary so we can discuss — it's not a blocker, just something we try to minimise.

## Checklist

- [ ] Builds locally against UE 5.7+
- [ ] Full `URLab.*` automation suite passes
- [ ] Docs updated for user-facing changes
