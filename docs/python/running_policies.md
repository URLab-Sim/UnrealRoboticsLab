# Running a Bundled Policy

A zero-to-running walkthrough: install the bridge, import the right MJCF,
add a PD controller to the articulation, launch the dashboard, run a
bundled pretrained policy.

If you already know the API and just want to launch something, the
short form is at the bottom under [Quick reference](#quick-reference).

For the broader Python surface, see
[Getting Started](getting_started.md) and the
[API Reference](api.md).

---

## 1. Install the bridge

Clone the [`urlab_bridge`](https://github.com/URLab-Sim/urlab_bridge)
companion repo and install the extras you need. Python 3.11+,
[uv](https://docs.astral.sh/uv/) recommended.

```bash
git clone https://github.com/URLab-Sim/urlab_bridge.git
cd urlab_bridge

uv sync                         # core client + transports
uv sync --extra ui              # dashboard (DearPyGui)
uv sync --extra robojudo        # bundled pretrained policies
uv pip install -e ./RoboJuDo    # policy framework (bundled submodule)
```

PHC-flagged policies (BeyondMimic, AMO, H2H, twist_tracker) also need:

```bash
cd RoboJuDo && git submodule update --init --recursive
```

For the full extras matrix and troubleshooting see
[Bridge install + dashboard](bridge.md#install).

---

## 2. Get the MJCF for your target policy

Each bundled policy expects a specific MJCF. The 29-DOF G1 policies all
work against the same XML; the 12-DOF G1 entries use a subset; the Go2
Walk-These-Ways policy uses the menagerie Go2.

| Policy key | Robot | MJCF source |
|---|---|---|
| `unitree_12dof` | G1 (12 DOF) | `urlab_bridge/assets/robots/g1/g1_29dof_rev_1_0.xml` (12 actuators driven; rest idle) |
| `unitree_wo_gait` | G1 (29 DOF) | `urlab_bridge/assets/robots/g1/g1_29dof_rev_1_0.xml` |
| `smooth` | G1 (29 DOF) | `urlab_bridge/assets/robots/g1/g1_29dof_rev_1_0.xml` |
| `beyondmimic_dance` | G1 (29 DOF) | `urlab_bridge/assets/robots/g1/g1_29dof_rev_1_0.xml` |
| `amo` | G1 (29 DOF) | `urlab_bridge/assets/robots/g1/g1_29dof_rev_1_0.xml` |
| `h2h` | G1 (21 DOF) | `urlab_bridge/assets/robots/g1/g1_29dof_rev_1_0.xml` |
| `twist_tracker` | G1 (12 DOF) | `urlab_bridge/assets/robots/g1/g1_29dof_rev_1_0.xml` |
| mjlab tracking | G1 (29 DOF) | `urlab_bridge/assets/robots/g1/g1_mjlab.xml` (mjlab actuator schema) |
| `go2_wtw` | Go2 (12 DOF) | `mujoco_menagerie/unitree_go2/go2.xml` (download separately) |

The G1 XMLs ship in `urlab_bridge/assets/robots/g1/` alongside their
mesh dirs (`meshes/` and `g1_mjlab_meshes/`). Drag the `.xml` straight
into the Unreal Content Browser. The importer runs the mesh
conversion pipeline (`Scripts/clean_meshes.py`) on first import and
produces the Articulation Blueprint.

For Go2, clone
[mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie)
and drag its `unitree_go2/go2.xml` into UE.

For the full import behaviour see [MJCF Import](../guides/mujoco_import.md).

---

## 3. Add a `UMjPDController` to the articulation

**Every articulation driven by a bundled policy needs a
`UMjPDController` component added to its Blueprint.** The bundled
policies emit position targets at policy rate; the PD controller
converts those targets into per-physics-step joint torques against
the live `qpos` / `qvel`. Without it the articulation either does not
move (motor actuators with no controller) or moves with raw MuJoCo PD
gains that do not match what the policy was trained against.

Steps in the editor:

1. Open the imported Articulation Blueprint (double-click in the
   Content Browser).
2. In the Components panel, click **+ Add Component**, search
   `MjPDController`, add it. The component auto-binds to every
   actuator on the articulation.
3. The defaults (`DefaultKp`, `DefaultKv`, `DefaultTorqueLimit`) are a
   reasonable starting point. Per-actuator gains can be overridden in
   the `Kp` / `Kv` / `TorqueLimits` arrays or pushed from Python at
   runtime via `client.runtime.set_gains(...)`.
4. Compile and save the Blueprint.

For gain tuning details and the full controller framework see
[Controllers and Gains](controllers_and_gains.md) and
[Controller Framework](../guides/controller_framework.md).

---

## 4. Launch the dashboard

Start UE, open or create a level, drop the imported articulation
Blueprint into it, and click **Play** (PIE). The bridge dispatcher
starts automatically on `tcp://localhost:5559`.

In a separate terminal:

```bash
cd urlab_bridge
uv run urlab-ui
```

The dashboard window opens. In the Connection bar at the top, the
default address `tcp://localhost:5559` is pre-filled. Click
**Connect**. The status pills should show:

- **session:** non-empty UUID
- **manager:** found
- **PIE state:** `ready`
- **articulations:** lists your imported robot's prefix

If anything is red, the most common causes are PIE not yet started,
the wrong host:port, or a controller that did not compile (the
**Debug** tab's compile-error pane surfaces compile messages).

For the full tab tour see [Bridge install + dashboard](bridge.md#dashboard).

---

## 5. Run a policy

### From the dashboard

1. Switch to the **Policy** tab.
2. Pick a policy from the dropdown (the bundled-policy registry is
   loaded at dashboard startup).
3. Set the **prefix** to the articulation prefix shown in the status
   pill (e.g. `g1`).
4. Click **Run**. The launcher checks the policy's
   `required_step_mode` against the current bridge mode, swaps mode
   if needed, and starts stepping. Joints / sensors / cameras update
   in their tabs as the policy drives the robot.

### Headless from the CLI

Same effect, no UI:

```bash
uv run urlab-policy --policy unitree_12dof --prefix g1
```

Pass `--help` for the full flag set (mode override, ZMQ endpoint,
device, etc.).

The launcher refuses to start if the bridge is in a mode the policy
declares incompatible (`required_step_mode`). The error message names
the expected mode so you can switch from the dashboard's status
pill or with `client.runtime.set_mode("direct")` from a REPL.

---

## Quick reference

For a returning user, the minimum sequence:

```bash
# Once
cd urlab_bridge && uv sync --extra ui --extra robojudo && uv pip install -e ./RoboJuDo

# Each session
# 1. UE: import urlab_bridge/assets/robots/g1/g1_29dof_rev_1_0.xml
#    Add UMjPDController to the Articulation Blueprint, compile.
# 2. UE: drop the BP into a level, click Play.
# 3. Bridge:
uv run urlab-ui                 # dashboard, click Connect, Policy tab
# or
uv run urlab-policy --policy unitree_12dof --prefix g1
```

See [Policy Registry](policy_registry.md) for the full list of
bundled keys, their robot / DOF / PHC requirements, and how to add
your own policy.
