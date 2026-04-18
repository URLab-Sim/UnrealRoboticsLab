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

## Proof of compilation

<!--
Paste the tail of your UnrealBuildTool / Build.bat output, enough to show
"Result: Succeeded". MuJoCo ASAN_* macro-redefinition warnings are expected
and can stay in the snippet.
-->

```
```

## Test evidence

<!--
Paste the tail of the URLab automation run. We're looking for the pass
count and any failures, e.g.
  grep -cE "Result=\{Success\}" ...
  159
  (no failures)
-->

```
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
