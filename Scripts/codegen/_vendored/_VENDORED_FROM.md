# Vendored sources

`sync_vendored.py` reads this manifest and rewrites it deterministically
on every run (stable column order, sorted by `local_dest`). Re-running on
the same upstream SHA produces zero diff — keeps the manifest out of
merge-conflict territory.

To bump a vendored file, edit its `upstream_sha` (or replace with `main`
for an unpinned re-fetch), then run:

```
python Scripts/codegen/sync_vendored.py
```

| file | upstream_url | upstream_sha | retrieved_date | license | local_dest |
| ---- | ------------ | ------------ | -------------- | ------- | ---------- |
| mjspecmacro.h | https://raw.githubusercontent.com/google-deepmind/mujoco/{sha}/include/mujoco/mjspecmacro.h | main | 2026-05-29 | Apache-2.0 | Scripts/codegen/_vendored/mjspecmacro.h |
