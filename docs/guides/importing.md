# Importing and authoring MuJoCo models

The Editor's MuJoCo XML factory imports an MJCF file as an Unreal Blueprint whose generated, typed component hierarchy is the persistent authoring document. The Content Browser import action and the editor-only `editor_import_xml` RPC use the same import operation. The RPC accepts an XML `path` and optional `force_reimport`; it is an Editor workflow, separate from the source-independent runtime API used by external controllers.

## Import an XML model

1. Import the `.xml` file through the Unreal Content Browser. The factory creates a Blueprint in the chosen destination and imports the supported referenced mesh and texture assets. The editor RPC uses `/Game/MuJoCoImports` as its destination.
2. Inspect the generated component hierarchy and diagnostics. Elements expose typed properties, schema-filtered references, and native Unreal transforms for supported spatial values. XML includes are expanded during import. Resource files captured by import are stored with the authored components so export does not have to read the original source tree.
3. Save the Blueprint, place it in a level, and add an `AAMjManager` scene-root actor. In an authored-simulation boot mode, the manager compiles the level's authored scene at play start, installs an owned model into the world simulation subsystem, and hands its presentation to the existing neutral renderer.

The RPC can automate step 1 from an external Editor client. It takes a filesystem path, not uploaded XML bytes; the RPC is unavailable in packaged games. `force_reimport=true` replaces the existing imported Blueprint, so use it only when that replacement is intended.

## Author a scene in Unreal

The scene can also be assembled from the generated typed model, section, asset, body, joint, geom, and actuator components. The manager owns the scene root; imported articulation Blueprints can be placed as scene participants. Asset entries have typed mesh and texture asset references, while geoms refer to the corresponding authored asset elements. Component properties and generated accessors provide Blueprint and C++ authoring access without editing XML directly.

Spatial import normalizes MJCF orientation spellings and `fromto` into Unreal transforms and supported dimensions. Those original spellings/endpoints are discarded, so export preserves the compiled meaning of the imported model rather than reconstructing its original syntax. Default-class spatial values remain inherited until a local authored override is made.

Visual mesh export reads the selected static mesh's LOD0 MeshDescription, preserving authored per-corner normals and UVs independently from collision geometry. If all corner normals are absent, the OBJ omits `vn` and MuJoCo applies its native missing-normal behavior; a partial normal stream is rejected. This does not promise parity with normals Unreal may generate for the built render LOD when mesh build settings recompute them. Exporting built-render topology/normals is a separate audit follow-up because it would also change the current source-MeshDescription geometry/build-scale contract. Arbitrary Unreal material graphs are not converted into MJCF materials; explicit MJCF material nodes remain authoritative. Some source formats have no Unreal preview converter even when the source bytes remain usable by MuJoCo.

## Runtime and packaging boundary

The authored level is compiled once at play start. `AAMjManager.CompileModel` reports whether the authored scene compiles; it does not replace the already installed runtime model. Generated authoring fields can be edited in Blueprint construction and level instances, but typed live-property bindings, runtime recompile/replacement from changed authored fields, and save-back are not committed features.

Asset bytes can be prepared and serialized with authored components before cooking. A scoped Linux Development cook/stage/archive passed for an authored map with a 2x2 hfield, and the staged game loaded that map with the linked MuJoCo 3.11.1 library. A negative cook with missing persisted `missing.obj` bytes exited 1. Existing `simulation_describe` and `simulation_observe` RPCs then confirmed the imported authored model at generation 1 with nq=7/nv=6, the expected Blueprint joint and advancing step/time. That NullRHI run verifies installation and stepping, not rendered pixels; broader resource and rendering validation remains open. The neutral renderer currently rejects a 1x1 heightfield (it requires at least 2 rows and columns). See [persistent cook/runtime evidence](../../../audit_docs/implementation_runs/authoring_packaged_handoff_20261002/README.md). Editor import UI, Details customization, and the import RPC remain Editor-only. XML, MJB, MJZ, and neutral-render sources continue to use the separate runtime loading path without authoring components.

See [MJCF authoring requirements and implementation status](../architecture/authoring_requirements.md) for supported semantics, known limits, and validation evidence.
