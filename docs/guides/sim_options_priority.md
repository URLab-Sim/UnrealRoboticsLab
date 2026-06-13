# SimOptions Priority

MuJoCo stores one `mjOption` block per compiled model. When multiple articulations in the same scene set the same simulation-option field (timestep, gravity, solver, ...), URLab's resolver picks which value wins. This guide covers the rules and how to control the outcome without editing MJCF.

---

## The problem

MJCF lets every model declare an `<option>` block:

```xml
<mujoco model="armA">
  <option timestep="0.002" integrator="Newton"/>
  ...
</mujoco>
```

```xml
<mujoco model="armB">
  <option timestep="0.005" gravity="0 0 -1"/>
  ...
</mujoco>
```

MuJoCo's own spec merge (`mjs_attach`) silently drops these per-subtree option blocks during compile. Every field in `mjOption` is global: one timestep, one gravity vector, one solver choice. Without additional logic, a scene with both arms above would compile with none of their option settings applied.

URLab fills that gap with the resolver. It runs after every articulation attaches to the root spec and before `mj_compile`, writing per-articulation option values back into the compound spec based on priority.

---

## Resolution rules

The resolver walks each option field independently, using the field's `bOverride_<Field>` flag to decide who contributes:

1. **No articulations set `bOverride_<Field>`**: the field stays at MuJoCo's default (or whatever the Manager's `SimOptions` applies post-compile).
2. **One articulation sets `bOverride_<Field>`**: its value applies, silently.
3. **Two or more agree on the same value**: that value applies, silently.
4. **Two or more disagree**: highest `SimOptionsPriority` wins. Ties break on actor iteration order. A `WARNING` is logged to `LogURLab`. Ties also get a second `WARNING` about ambiguous fallback.
5. **The Manager's `SimOptions` (on `AAMjManager`)** is applied post-compile and acts as a full override: when the Manager has `bOverride_<Field>=true`, that value wins regardless of what the articulation resolver produced.

---

## Why per-field instead of whole-articulation

Two alternatives were considered: per-field (chosen) and "whole-block" (entire highest-priority articulation wins, losers contribute nothing).

Per-field matches how MJCF is authored. Authors write `<option field1="X" field2="Y"/>` with exactly the fields they care about; `bOverride_<Field>` mirrors that. When two articulations set *different* fields, both authors get what they asked for. Whole-block would silently discard non-conflicting settings from every articulation except the priority winner.

The trade-off is that per-field may produce a combination neither author explicitly tested (e.g. one author's `integrator` paired with another author's `tolerance`). That's rare in practice, but if it matters for your scene, push the authoritative values onto the Manager; Manager overrides run after the resolver.

---

## Worked example

Scene contains two articulations:

| Articulation | `bOverride_Timestep` | `Timestep` | `bOverride_Integrator` | `Integrator` | `bOverride_Gravity` | `Gravity` | `SimOptionsPriority` |
|---|---|---|---|---|---|---|---|
| ArmA | true | `0.002` | true | `Newton` | false | (default) | `5` |
| ArmB | true | `0.005` | false | (default) | true | low | `2` |

Resolution:

- **Timestep**: both set, values disagree, priority 5 > 2, ArmA wins. Spec `timestep = 0.002`. One WARNING.
- **Integrator**: only ArmA set. ArmA's `Newton` applies silently.
- **Gravity**: only ArmB set. ArmB's low gravity applies silently.

Compiled model: `timestep=0.002, integrator=Newton, gravity=<low>`. Both authors' intent is honored everywhere they didn't fight with another author. Exactly one warning fired, for the one real conflict.

---

## How to tailor the resolver

Three knobs, in increasing order of force:

### 1. Tweak `SimOptionsPriority`

Per-articulation integer in the Details panel under `MuJoCo|Options`. Higher wins. Default is `0`.

Use when two articulations disagree and you know which one's value is correct for the scene. Leaves every other field untouched.

### 2. Toggle `bOverride_<Field>` on the articulation

Open `SimOptions` on the articulation in the Details panel. Each field has a checkbox that gates whether the articulation participates in the resolver for that field.

Use to stop an articulation from contributing a value it no longer needs (silences warnings), or to explicitly start contributing one that MJCF didn't set.

### 3. Full override on the Manager

`AAMjManager` has its own `Options` struct. Set `bOverride_<Field>=true` there plus the value, and the Manager's value wins regardless of what the articulation resolver produced.

Use when you want one authoritative value for the whole scene, or when a particular combination of article-level settings is producing behaviour you don't want and you'd rather short-circuit the resolver for that field.

---

## How to silence a specific warning

The resolver only warns when articulations *actually disagree* on a field value. If you're seeing a warning you consider noise:

1. Check that the values really are different. The resolver uses `FMath::IsNearlyEqual` with `1e-12` precision for floats; `FVector::Equals` with `1e-6` for vectors. Near-equal is counted as agreement.
2. If one articulation no longer needs the override, untick its `bOverride_<Field>` in the Details panel.
3. If you want one authoritative value for the whole scene, move the override onto the Manager. Manager overrides never trigger the resolver's warnings.

---

## Reading the warnings

The log lines follow a consistent format so you can grep them:

```
LogURLab: Warning: [SimOptions] '<Field>' conflict across <N> articulations: <Name>(pri=<P>, val=<V>), ... . Applying <Winner>'s value <V>.
LogURLab: Warning: [SimOptions] '<Field>' ambiguous priority tie between <Names> (all pri=<P>). Fell back to actor iteration order; applied <Winner>'s value. Set AMjArticulation.SimOptionsPriority to disambiguate.
LogURLab: Warning: [SimOptions] <N> field(s) had cross-articulation conflicts. See https://urlab-sim.github.io/UnrealRoboticsLab/guides/sim_options_priority/ for resolution rules.
```

The first line fires once per disagreeing field. The second only appears when the winning priority ties with another contributor's priority. The third is a single tail line per compile cycle pointing at this guide.

---

## Fields covered by the resolver

All fields on `FMuJoCoOptions` that have a paired `bOverride_<Field>` flag participate. At the time of writing:

| Category | Field | Type | Notes |
|---|---|---|---|
| Timing | `Timestep` | float | Simulation time resolution. Lower values are more stable but slower. |
| Timing | `MemoryMB` | int | Arena size for the compiled model. |
| Environment | `Gravity` | FVector | UE units (cm/s^2). Converted at apply. |
| Environment | `Wind` | FVector | UE units (cm/s). Converted at apply. |
| Environment | `Magnetic` | FVector | MuJoCo units; Y is negated. |
| Environment | `Density` | float | Fluid density of the medium. |
| Environment | `Viscosity` | float | Fluid viscosity. |
| Solver | `Impratio` | float | Friction-to-normal impedance ratio. |
| Solver | `Tolerance` | float | Convergence tolerance. |
| Solver | `Iterations` | int | Solver iteration cap. |
| Solver | `LsIterations` | int | Line-search iteration cap. |
| Solver | `Integrator` | enum | Euler / RK4 / Implicit / ImplicitFast. |
| Solver | `Cone` | enum | Pyramidal / Elliptic friction cone. |
| Solver | `Solver` | enum | PGS / CG / Newton. |
| Solver | `NoslipIterations` | int | Noslip constraint solver iteration cap. |
| Solver | `NoslipTolerance` | float | Noslip constraint solver tolerance. |
| Solver | `CCD_Iterations` | int | Continuous collision detection iteration cap. |
| Solver | `CCD_Tolerance` | float | CCD tolerance. |

`bEnableMultiCCD`, `bEnableSleep`, and `SleepTolerance` do not have `bOverride_<Field>` flags and are treated as Manager-only scene toggles. They never participate in articulation-level resolution.

---

## When the dialog appears

The dialog pops once per compile cycle, editor only, when at least one field had a real disagreement. It summarises every conflict in a single popup and points at this guide. The automation test suite (`-Unattended`) suppresses the dialog via `FMessageDialog::Open`'s built-in unattended handling, so running the test suite will never block on a dialog.

Full per-contributor details live in the Output Log under `LogURLab` warnings (see [Reading the warnings](#reading-the-warnings) above).

---

## Related

- [Troubleshooting: SimOptions conflicts](../getting_started.md#dialog-simoptions-conflicts-detected) in Getting Started.
- MuJoCo's own [`<option>` XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#option) documents every field and its effect on the simulation.
