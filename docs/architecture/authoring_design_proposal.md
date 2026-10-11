# Generated Unreal MJCF authoring — concrete design

Implemented authoring architecture, revised 2026-10-02. [Requirements](authoring_requirements.md) define the workflow and remaining proof points. The generated-UE XML path, native MuJoCo compilation, import-factory corpus, sidecar-reference tests, scoped authored-map cook, and packaged installation/stepping checks have evidence, but the prior captured-source bundle is superseded by the accepted source-of-truth requirement: after import, UE-authored Blueprints/assets/fields must be sufficient without retained original bytes or paths. See the requirements section for the accepted resource materialization scope. The NullRHI package run did not check rendered pixels.

## One compiler and one persistent document

Generated Unreal component properties are the sole persistent authored document. There is no plain ProtoSpec AST, second synchronized document, or persistent native-spec mirror in the implemented path. The generated fields retain typed optional presence, child contexts and references. Ordinary attributes keep their source MJCF spelling in those fields; import consumes alternate spatial spellings and stores normalized native Unreal spatial values instead of retaining those raw spellings. The discarded spatial source forms are never reconstructed for export.

The retired compiler path (`MjSpecBuild.cpp`, `MjSpecWriteHooks.cpp` and generated `MjSpecWrite.gen.cpp`) constructed `mjSpec` element by element and duplicated XML reader rules. The current path emits one MJCF document per authored UE document and lets the bundled MuJoCo parser construct each `mjSpec`. Existing participant placement, conflict handling, `mjs_attach`, `mj_compile`, bindings and neutral renderer handoff remain the scene path.

```text
UE component document
  -> generated UE descriptors + shared XML writer + resolved dependency bytes
  -> MuJoCo mj_parseXMLString
  -> existing participant placement / conflict / mjs_attach
  -> mj_compile
  -> existing bindings and neutral renderer handoff
```

The native parse path retains transient name and asset substitutions by node serial, then reconstructs bindings from the parsed spec. It preserves scene assembly, `mjs_attach`, `mj_compile` and runtime handoff. The direct builder, handwritten write hooks, generated native write backend, generic profile SDK, plain AST and generic reader/writer were removed after focused import and compiled-model parity gates. Native MuJoCo remains responsible for physics validation and ordinary shortcut/alias interpretation.

`protospec_core` now contains the native-backed orientation resolver. `protospec_mjcf` contains the include expander and its diagnostics. tinyxml2 handles XML syntax; `protospec_harness` remains solely because editor parity tests compare native `mjModel` values. URLab's generated field descriptors and one UE gateway provide typed import/export. The corpus and focused inherited-Blueprint cases have passed or recorded the limits below; packaged runtime model installation is proven by the focused RPC probe; copied references retain their original target unless explicitly rebound.

## What the user builds

```text
Model
├─ Asset
│  ├─ Mesh Countertop       [Static Mesh asset picker]
│  └─ Texture Wood         [Texture asset picker]
├─ Worldbody
│  └─ Body Cabinet         [Unreal transform]
│     ├─ Joint Hinge       [typed properties and axis visualization]
│     └─ Geom Door         [Unreal transform/scale; Mesh → Countertop]
└─ Actuator
   └─ Position DoorDrive   [Joint → Hinge; Kp → numeric property]
```

Asset nodes define shared resources and draw nothing themselves. Geoms display their selected resource at their body/geom transform in the Blueprint viewport. The user can create the hierarchy or import XML through the editor action or editor import RPC. Both import entry points execute the same operation.

Spatial editing uses native Unreal transform controls and gizmos, not competing raw `pos`, `quat`, `euler`, `size` and `fromto` fields. Nonspatial physics parameters remain typed properties. Blueprint and user C++ code get typed access.

At runtime, the authored scene becomes `mjModel`, then the existing MuJoCo adapter supplies the neutral renderer. Editor previews do not become a second runtime render scene. Optional Blueprint bindings address the installed simulation; normal external Python access remains through the existing runtime API.

## One authoring representation

Generate Unreal component classes whose properties own authored data. Spatial state lives in native Unreal transforms, accessed by shared import/export conversion. There is no second persistent plain AST, property-bag document or native-spec mirror.

A common component base supplies identity and document ordering. Generated components participate in the Blueprint hierarchy; nonspatial section/resource nodes have identity transforms and no viewport geometry. Schema descriptors determine valid child contexts. Component attachment and document ordering must be tested under inherited Blueprints rather than assumed equivalent to XML order.

Ordinary fields have typed values and explicit presence. Generated Has/Get/Set/Clear functions expose optional values with normal Blueprint pins. Reset to an inherited Unreal value differs from Clear, which authors an absent MJCF value. Raw spatial variant attributes may be recognized as transient import inputs, but they are not competing persistent editable controls: imported `euler`, `axisangle`, `xyaxes`, `zaxis` and `fromto` become normalized UE spatial state. Default-class spatial fields likewise become normalized values and remain live through ordinary class inheritance and local overrides. Asset nodes have typed source-asset references.

## Actual code generation

One Python command consumes MuJoCo's own `doc/generate/mjcf_schema.py` representation of its pinned `src/xml/mjcf.schema`. It uses expanded attributes, element contexts, enums, arrays, reference categories and child rules. Do not write a second schema parser, scrape C++ with regex, or introduce an XSD-to-C++ framework. The generated XSD can check vocabulary; the source schema has the useful generation metadata.

Generation emits:

| Output | Purpose |
| --- | --- |
| Reflected component classes | One type per legal element/context, with typed scalar, enum, array, file and reference properties. |
| Blueprint functions | Has/Get/Set/Clear and typed reference operations. Direct optional-value Blueprint pins are not assumed to work. |
| Field/child descriptors | Source XML names, reflected property bindings, primitive types, presence, arity, child order/validity and target namespaces; transient spatial input spellings are identified. |
| Factory tables | Create an element by context/tag during import without handwritten per-tag switches. |
| Editor metadata | Categories, choices, reference filters and asset pickers; supplement only what upstream does not describe. |
| Manifest | Input revision/hashes and output list for deterministic regeneration and review. |

Descriptors bind directly to reflected fields. One XML reader/writer handles primitive value kinds. There is no handwritten ReadGeom, WriteMotor or CreatePositionActuator conversion.

A normal new numeric schema attribute generates a property, Blueprint functions and XML support without a per-element source edit. The generator reads expanded upstream attributes directly rather than folding ordinary aliases into a separate canonical AST field. MuJoCo interprets `material texture`, `cylinder diameter`, `light directional`, `fullinertia` and other raw MJCF spellings during native parse; the UE importer need not duplicate their compiler meaning. New schema syntax or unknown spatial semantics is reported rather than guessed. Generated output is never hand-edited. Naming uses a deterministic rule with collision detection.

Pin schema and runtime together. The generator reads bundled MuJoCo 3.11.1 at `2544c349977261541ad493ce03b32c711618b91d` (`mjVERSION_HEADER 3011001`). The isolated preparation check resolved upstream HEAD to `3b5ecadb7386d5d9233e0223404149fc5ca2a08d` (3.14.1) on 2026-10-02; generator/default-probe gates passed, while candidate XML and enum policy tests found native migration blockers. This candidate is not the production input or a runtime upgrade. Upstream's schema parser is an internal API, so a coordinated upgrade must regenerate and retest rather than emit newer fields against the older runtime; see [the preserved upgrade report](../../../audit_docs/implementation_runs/mujoco_upgrade_rehearsal_20260927/verification.md).

## Trace 1: XML mesh asset → geom → runtime

1. The importer reads a temporary XML DOM, resolves includes/dependencies and creates generated components from the factory table. It keeps ordinary authored attribute presence and tags, normalizes spatial variant spellings, and discards the DOM. XML syntax, primitive parsing and source diagnostics remain importer responsibilities; MuJoCo validates physics at native parse/compile.
2. Supported Unreal importers create persistent mesh/texture assets. Other supported resources require appropriate persistent data assets. Unsupported formats need an explicit decoder or diagnostic; the MJCF generator cannot supply a mesh decoder.
3. The generated mesh entry stores its UE asset reference. The geom's `ref<mesh>` resolves to that entry using the common reference mechanism. The picker selects the entry, not a filename string.
4. The editor preview draws the referenced mesh for the geom. Shared asset entries stay invisible. Transform edits move the geom/body; source-asset edits refresh consumers. These are pre-runtime editor visuals, with no requirement to compile physics on every drag.
5. Save the Blueprint and dependency packages. Reopening after deleting the original source directory must work. Import errors must identify partial outputs; native Undo alone is not presumed to roll back filesystem package writes.
6. Compilation/export converts the selected UE resources into MuJoCo-compatible dependency bytes. This is internal conversion, not a user export action. The UE descriptor writer emits resource names and references consistently. Cook stores the needed data rather than relying on editor-only source art or readable cooked CPU buffers.
7. Native MuJoCo parses and compiles the XML plus VFS. The existing runtime and neutral renderer take over. Compare preview and runtime for unintended differences.

Manual creation starts with a user-selected UE asset and follows the same path. Imported dependencies must become persistent UE assets, not soft paths to the original XML directory. Mesh export uses LOD0 MeshDescription topology, UVs and authored corner normals when present. If all corner normals are absent, OBJ omits `vn` and MuJoCo uses its native missing-normal behavior; partially missing normals fail closed. This does not guarantee parity with normals Unreal generates for the built render LOD under recompute-normal settings. Exporting built-render topology/normals remains an audit follow-up because it would also change the current source-MeshDescription geometry/build-scale contract. Arbitrary Unreal material graphs are not converted to MJCF; explicit MJCF material nodes remain authoritative.

Collision-data needs are evaluated per consuming geom using `contype`, `conaffinity` and explicit pairs. Both masks zero excludes ordinary mask-based contact but not explicit pairs. Masks do not choose CoACD or a decomposition algorithm. Preserve visible geometry independently; shared asset uses can require different compiler resources. Exact visual/collision representation in MuJoCo is a technical proof point, not an already-solved conversion policy.

## Trace 2: joint → position actuator

1. Schema entries generate joint and position-actuator types. The actuator's `joint: ref<joint>` generates picker filters and typed reference operations.
2. The generated string reference is the compiler-facing value. Editor data also tracks a selected SCS target with `FComponentKey`; one schema-driven path supplies options and reference traversal.
3. The fresh editor-only reference-identity test passed for an inherited target override, reference rename without parent mutation, placed-instance rename, saved/reloaded identity, and fail-closed target deletion. Internal target remapping for a duplicated subtree still needs direct proof.
4. Export resolves the editor binding to the authored MJCF name (and validates it against the generated string) before emitting `<position joint="Hinge" kp="..."/>`. Native MuJoCo handles gain/bias expansion. We do not populate `mjsActuator` through a handwritten converter.
5. No typed field-to-installed-model binding or live authored-field recompile is committed. The existing runtime remains a separate model-wide simulation service; authoring components are not simulation owners.

Resolve imported name references after nodes exist. Missing, ambiguous and wrong-type references produce diagnostics. Default-template contexts and ordinary elements must follow the generated schema context rules.

## Spatial import and editing

Use Unreal's centimeters/degrees and native transform controls for editing. At import, consume `fromto` and alternate orientation spellings into normalized spatial values; the original spellings are discarded. Export emits those normalized values and uses the pinned MuJoCo parser/compiler for the resulting XML. Initial import/export must compile to the same model within the stated tolerance; later edits are interpreted as edits to the normalized UE values, not as a renewed reading of discarded source spelling. Any required radians conversion for output must follow the effective compiler angle mode and the semantic category of each field.

A shared spatial pass handles positions, rotations, directions and dimensions. Primitive sizes map to scale/shape handles with correct radius/half-extent rules. Mesh intrinsic scale and geom instance scale must not be applied twice. Unsupported shear, negative scale or shape constraints need visible restrictions. Joint axes/endpoints use editor visualization.

The schema does not fully specify units or gizmo semantics. One audited annotation table and shared mathematical handlers are necessary handwritten domain code. This is an explicit exception to automatic generation, not a converter class per element. Angle conversion must distinguish angular from linear joint values and honor default contexts; do not multiply all numeric fields blindly.

Prefer MuJoCo's native spatial conversion results where they can be mapped reliably back to authored components, including defaults, frames and replicates. A narrowly tracked private conversion copy is acceptable only where native output cannot recover the needed authored relationship. Normalize default-class spatial fields as live default values; a concrete element inherits them until a local transform/size edit authors an override, and clearing that override resumes inheritance. A type change after import follows the normalized UE dimensions and the new type's size mapping. It does not reinterpret the old `fromto` endpoints. A default template shared by elements with different effective types stores one type-neutral, optional `MjSpatialHalfExtentsCm` value: radial X/Y and axial Z half-extents. Its override is atomic, and each concrete consumer maps the effective value through its own effective shape. Nonspatial defaults stay authored as before.

## Exactly what we write

These are concrete source responsibilities, not independent frameworks. Names are illustrative.

| Handwritten code | Contents | Provided by dependencies |
| --- | --- | --- |
| `generate_authoring.py` and templates | Read pinned schema IR; emit types/descriptors; coverage and naming checks. | MuJoCo schema parser; Unreal UHT. |
| Common authoring component and reference handling | Identity/order, optional presentation, schema-filtered pickers and reference resolution. | UE components, inherited templates, transactions and asset references. No custom inheritance engine or Blueprint graph. |
| Shared XML import/export | DOM/descriptor traversal, primitive encoding, reference resolution and diagnostics. | Standard XML parsing; all physics and shortcut interpretation delegated to MuJoCo. |
| Resource import/conversion | Invoke supported UE importers, save packages, produce compiler bytes and cook dependencies. | UE asset registry, persistence, cooking and available format importers. No new asset database. |
| Spatial/editor behavior | Import-only `fromto` midpoint/half-length normalization using effective default-class context; type-neutral UE centimetre half-extents and per-shape size mapping; MJCF/UE coordinate conversion, transform write-back and geom previews. | MuJoCo public orientation resolver and native compiler validation; UE transform widgets, Details and preview-world facilities. |

The exporter calls existing runtime installation contracts. It does not create another simulator or renderer. Source-specific mesh/texture conversion is distinct from the removed per-MJCF-element compiler mappings.

The earlier implementation captured original XML, dependencies, and file resources on the root generated component and replayed unchanged source bytes. That design is superseded: persistent state must be authored UE Blueprints, typed UE assets, and editable numeric fields. Generated compiler payloads may be cached only when derived from current UE state. Includes expand into the authored graph; native model attachments reference separately editable child Blueprints. `FMjFieldSpec.bFileResource` identifies `mesh.file`, `hfield.file`, `skin.file`, `texture.file` plus six cube-face attributes, `model.file`, and `flexcomp.file`; each must be materialized or rejected clearly, never retained as an opaque source payload. Ordinary raw MJCF fields continue to be parsed by MuJoCo's native XML compiler. Compilation and install must continue through the existing native MuJoCo/model service, adapter, and neutral renderer. The previous source-hidden and NullRHI results do not prove this revised source-of-truth contract or rendered pixels. See [requirements](authoring_requirements.md) and the [archived packaged handoff](../../../audit_docs/implementation_runs/authoring_packaged_handoff_20261002/README.md) for the distinction between prior evidence and remaining proof.

One bounded schema exception handles native alias elements. The pinned MuJoCo reader accepts and ignores unknown attributes on `frame` and `replicate`; generated descriptors flag those aliases so direct import warns and drops such attributes. Ordinary tags still reject unknown attributes. The generator checks the upstream alias set before applying this policy, so a new alias kind requires review rather than silently inheriting it.

No standalone AST SDK, generic profile system, persistent native-spec mirror, property-bag editor, custom Blueprint editor, MuJoCo compiler clone, general remote authoring protocol or universal runtime mutation engine. The editor import RPC is just a second entry to the importer.

The original cutover reduced production handwritten source by 11,777 nonblank lines; that historical count is recorded in `audit_docs/implementation_runs/authoring_simplification_20260927/loc_report.txt`. Relative to the subsequent committed baseline, the current resource/alias work adds 803 UE production, 48 native-helper and 15 generator nonblank lines (+866 total), plus 1,033 UE test and 24 generator-test lines. New production files account for 570 lines of bundle implementation, 23 of its API, 113/6 of the editor helper wrapper and 48 of the isolated native scanner CLI; the rest are changes to existing code. Generated C++ adds one nonblank line. These are source counts, not performance results or a final net comparison to the original pre-cutover checkout. Reproduce the current delta from repository root with `python3 ../audit_docs/implementation_runs/authoring_simplification_20260927/loc_report.py`; it compares tracked and new files with `git show HEAD:<path>` and separates production, tests, generated output and the retained model-diff harness.

## Packaged use and later runtime edits

`AAMjManager` compiles the placed authored scene at play start and installs the owned candidate and presentation into the world simulation subsystem; the source components do not own the installed model. `AMjArticulation` exposes Blueprint discovery of compiled element components. A scoped authoring-specific Linux Development cook/stage/archive passed for a map containing a 2x2 hfield, and the staged game loaded that map with MuJoCo 3.11.1. A negative cook with a referenced `missing.obj` lacking persisted compiler bytes exited 1, so this preparation error is observed to fail the cook. The existing `simulation_describe`/`simulation_observe` RPCs then confirmed generation 1, MuJoCo 3.11.1, the expected imported Blueprint joint, nq=7/nv=6, stable model SHA-256, and advancing step sequence/time. This proves packaged model installation and stepping; the NullRHI run does not validate rendered pixels. The neutral heightfield renderer currently rejects dimensions below 2x2. Persistent logs and artifact details are in the [2026-10-02 packaged handoff](../../../audit_docs/implementation_runs/authoring_packaged_handoff_20261002/README.md).

The Blueprint-callable `AAMjManager.CompileModel` compiles for status and diagnostics; it does not replace an already installed runtime model. Typed live field bindings, recompile/replacement after runtime edits, and save-back remain optional design followups. Schema metadata does not provide safe compiled-field mutation rules. Do not implement a mapping for every runtime field in the initial replacement. Candidate recompilation can be investigated first; retaining enough source for it does not require a packaged editor.

Persistence of play edits, state migration across recompile and arbitrary material support remain unresolved. Nothing here silently saves play changes to an asset.

## Verification and remaining gates

The native XML parse/compile path, direct SCS and actor import, generated raw-attribute descriptors, editor reference traversal, and native model comparison corpus have been exercised in the integrated build. The plain AST/SDK/IO code and direct `mjSpec` write backend are deleted. Regenerate with `cd protospec && uv run python -m protospec_gen.emit_ue --out .. --check`; run `uv run pytest -q` for schema/generator coverage; stage narrow native helpers with `protospec/build.sh` (or `.ps1`) before the Unreal editor build. Differential compile compares `mjModel` against native MuJoCo on the fixture corpus, while import tests cover SCS/actor graphs, aliases, defaults, references, assets and includes.

Focused tests cover isolated native dependency discovery, native-ignored alias attributes, edited mesh/texture bundle export after hiding the original source directory, manager-to-runtime handoff, selected visual-mesh export without collision data, and derived-Blueprint edits/export. The completed full import-factory corpus attempted all 906 native-valid inputs: 889 full-model passes with preview assets, one native-equal KTX model without Unreal preview conversion, three strict BVH mismatches, and 13 inert include fragments. The per-index evidence and limits are in `audit_docs/implementation_runs/full_corpus_20260927/factory_portable_report.md`. The Content Browser factory and editor `editor_import_xml` RPC converge on the same `ImportXmlSync` import operation. The `URLab.Authoring` filter passed 4/4 tests, `URLab.Export` passed 5/5 after fixing the selected-MeshAsset precedence conflict, and mesh preparation passed 1/1; see the [packaged authoring handoff](../../../audit_docs/implementation_runs/authoring_packaged_handoff_20261002/README.md). The final packaged RPC probe reports generation 1, nq=7/nv=6, expected joint identity and advancing sequence/time after the latest mesh changes. Packaged visual-output verification remains open because the runtime probe used NullRHI. The fresh sidecar-reference test passed for inherited and placed target renames, persistence, and deleted-target rejection; copied references intentionally retain their original target unless a user explicitly rebinds them. The log is in the [packaged authoring handoff](../../../audit_docs/implementation_runs/authoring_packaged_handoff_20261002/reference_graph_test.log). Live runtime binding and save-back are optional design work, not a committed gate. A compiled-model parity result establishes initial physics meaning, not preservation of discarded `fromto` endpoints or every later editor workflow.
