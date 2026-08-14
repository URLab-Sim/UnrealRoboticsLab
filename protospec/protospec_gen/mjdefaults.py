"""MuJoCo's own initialised defaults, as a second source for the schema layer.

`mjcf.schema` states a default with `=` for a minority of attributes -- 209 of
1410 stored fields at the pin. The rest are not undefaulted: MuJoCo initialises
them in `mjs_defaultGeom` and its siblings (`src/user/user_init.c`, plus
`mj_defaultOption` / `mj_defaultVisual` / `mj_defaultStatistic` /
`mj_defaultLROpt` reached through `mjs_defaultSpec`), which is the layer the
compiler actually merges against. Reading only the schema's `=` rows leaves an
editor unable to answer "what will the compiler use for this attribute", which
is the whole question a details panel exists to answer.

**Where the values come from.** A generated C++ probe, `mjs_defaults_probe.cc`,
built and run against the pinned MuJoCo by `tools/refresh_mj_defaults.py`. It
default-constructs each mjs struct through MuJoCo's own initialiser and prints
the fields the spec-write layer already binds attributes to. The alternative --
parsing `user_init.c` -- was rejected on two counts: it sees only the
assignments written there, so every `<option>`, `<visual>`, `<statistic>` and
`<lengthrange>` default (initialised in `engine_init.c` through calls
`user_init.c` merely makes) would be invisible; and it would have to reimplement
C struct layout and initialisation to know what a field holds after a `memset`
followed by a partial assignment. The compiler already does both, exactly right.

**Where they are applied.** :mod:`protospec_gen.frontend`, as a fill-only layer
under the schema: an attribute with an `=` default in `mjcf.schema` keeps it,
because the schema is the authored contract. Only an attribute the schema leaves
silent takes MuJoCo's struct value.

**What is skipped, and why it is recorded rather than dropped.** Three cases
never produce a display value:

* `dynamic` -- the field is an `mjString*` / `mjDoubleVec*` / similar owning
  pointer, whose default-constructed state is empty. "Empty" is what unset
  already looks like, so a default of `""` would be noise, not information.
* `sentinel` -- the value is not a value: `mjNAN` (a computed default, e.g.
  `geom.mass`), or a negative marker on an attribute the schema itself declares
  non-negative (`compiler.settotalmass = -1` meaning "use the authored masses").
  Where the schema spells that marker as a keyword the enum path resolves it;
  where it does not, nothing is invented.
* `unmapped` -- an enum field holding an integer no schema keyword names.

The skips are written into the data file beside the defaults, because the pair
is what the drift gate checks: `defaults` and `skipped` together must be exactly
the attributes the spec-write layer can reach, so an attribute that stops being
covered fails generation by name instead of quietly losing its row.

Refresh (needs CMake and a compiler; the pinned MuJoCo install is enough)::

    uv run python tools/refresh_mj_defaults.py
"""

from __future__ import annotations

import json
import os

__all__ = ["DEFAULTS_PATH", "load", "coverage", "DefaultsError"]

_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULTS_PATH = os.path.join(_HERE, "mujoco_defaults.json")

# Every reason a probed field yields no default. Closed on purpose: a refresh
# that invents a fourth is a change to what "no default" means, and has to say
# so here.
SKIP_REASONS = ("dynamic", "sentinel", "unmapped")


class DefaultsError(Exception):
    """The probed defaults no longer match the schema they are read against."""


def load(path: str | None = None) -> dict:
    """The probed defaults, as `{"mujoco": sha, "defaults": {...},
    "skipped": {...}}`.

    Missing file is an error rather than an empty layer: silently emitting no
    MuJoCo defaults would leave every panel row unresolved and nothing would
    say why.
    """
    target = path or DEFAULTS_PATH
    if not os.path.isfile(target):
        raise DefaultsError(
            f"no probed MuJoCo defaults at {target}; refresh them with "
            "`uv run python tools/refresh_mj_defaults.py`")
    with open(target, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    for key in ("mujoco", "defaults", "skipped"):
        if key not in data:
            raise DefaultsError(f"{target} has no {key!r} section")
    for element, rows in data["skipped"].items():
        for attr, reason in rows.items():
            if reason not in SKIP_REASONS:
                raise DefaultsError(
                    f"{element}.{attr} is skipped for {reason!r}, which is not "
                    f"one of {list(SKIP_REASONS)}")
    return data


def coverage(data: dict) -> set[tuple[str, str]]:
    """Every (element, attribute) the probe reached, defaulted or skipped."""
    out: set[tuple[str, str]] = set()
    for section in ("defaults", "skipped"):
        for element, rows in data[section].items():
            out.update((element, attr) for attr in rows)
    return out


def counts(data: dict) -> dict[str, int]:
    """Defaulted and per-reason skipped totals, for the count gate to assert."""
    out = {"defaults": sum(len(r) for r in data["defaults"].values())}
    for reason in SKIP_REASONS:
        out[reason] = sum(1 for rows in data["skipped"].values()
                          for r in rows.values() if r == reason)
    return out
