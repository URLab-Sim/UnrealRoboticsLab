# URLab completion and handoff status — 2026-10-09

Implementation is paused at an unfinished authoring refactor. This checkpoint's latest materializer changes have not had a complete build or test run. Do not treat it as ready or as covered by older passing results. The plugin's established architecture remains the baseline described in [intended design](intended_design.md). Authoring requirements and decisions are in [authoring requirements](authoring_requirements.md) and the [design proposal](authoring_design_proposal.md).

## Established baseline

URLab has distinct simulation, render-server, viewer, and editor APIs. ZMQ carries commands, with local SHM bulk data or network data; gRPC exposes `dm_env` and URLab rendering extensions. XML plus assets, MJZ, and MJB are primary model inputs. Unreal authoring compiles to a native `mjModel` installed through the same world-owned physics service, MuJoCo adapter, and physics-independent scene/Cartesian-frame renderer used by external Python/server inputs. Authoring is not a separate runtime path. Python flex rendering was verified at an earlier baseline; it does not validate this checkpoint's C++ changes. The current blocker is editor mesh import/export conversion: unit, topology, and UV handling remain unvalidated.

The Linux core architecture and packaged workflows were verified at their recorded September 2026 checkpoints. Subsequent checkpoints are `35e2d15` (reject degenerate heightfields), `b5d9fae` (passing explicit, missing, and edited source-MeshDescription normal-export cases; no production importer fix), and `b916e07` (body-BVH semantic parity checks). The earlier authoring checkpoint `c440bca` and its packaged/corpus evidence exercised captured-source behavior; it is superseded by the accepted source-of-truth requirement and does not verify this refactor. The `URLab_Bridge` repository is clean at `902bf66`. Each checkpoint establishes only the code and tests it exercised.

## Current authoring state and blockers

The authoritative persistent authoring data must be editable Unreal assets, Blueprint components/references, and editable numeric fields. Imported XML, includes, model files, mesh/texture/resource bytes, and source paths must not be retained or used for original-source replay. Generated compile/cook XML and resource bytes are allowed when regenerated from current UE authored state. Includes expand into the authored graph; native model attachments reference separately editable child Blueprints. Compilation still hands the resulting model to the existing physics service and renderer.

This checkpoint partially removes original-source resources and author-specific runtime overrides, adds generated compiler payloads and child-model references, materializes heightfield/skin/flex resources, and adds QuickConvert visual/collision geoms. It remains unbuilt/untested; older package, corpus, and source-hidden results do not validate it. Editor mesh unit/topology/UV conversion is unvalidated. Nested child-Blueprint import/edit/compile needs end-to-end proof. Native resource coverage, including flex, dim3, and GMSH, plus source-free save/reopen/cook, remain open. New materializer and mesh import/export tests are present but unverified.

## Resume order and scope

1. Build this checkpoint and run focused mesh/materializer tests; settle editor mesh units, topology, and UV conversion.
2. Prove nested child Blueprint workflows, supported native resource coverage, and source-free export/compile/save/reopen/cook from editable UE state.
3. Run a fresh corpus comparison and packaged RGB/depth acceptance through the existing physics-service → MuJoCo-adapter → neutral-renderer path. The older packaged authoring run used NullRHI and did not prove pixels.

Task logic, runtime `name`/ID bindings, live authored-field mutation/recompile, save-back, ROS alignment, recording/replay, Gym/task wrappers, VR, DLSS, and ray tracing remain deferred. Local `audit_docs` artifacts are historical evidence outside this plugin repository.
