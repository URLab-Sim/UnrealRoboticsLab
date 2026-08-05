# Spike 0b: `UPROPERTY() TOptional<T>` under real editor conditions

Gates the plan's storage decision: generated `UMj*` components hold MJCF attributes as
`UPROPERTY(EditAnywhere) TOptional<T>`, where unset means "absent from the document" and set
means "authored". Fallback if this fails is generated `bOverride_` pairs.

Engine: UE 5.7 (`C:\Program Files\Epic Games\UE_5.7`). Branch: `spike/toptional-lifecycle`.

## Spike code

- `Source/URLabEditor/Private/Tests/MjSpikeOptionalComponent.h/.cpp` - throwaway `UActorComponent`
  carrying `TOptional<double>`, `TOptional<FVector>`, `TOptional<FQuat>`,
  `TOptional<TArray<double>>`, `TOptional<FString>`, `TOptional<EMjSpikeOptEnum>`, a
  `BlueprintReadWrite TOptional<int32>` hazard probe, and a plain `FQuat` control.
- `Source/URLabEditor/Private/Tests/MjSpikeOptionalTests.cpp` - 10 automation tests under
  `URLab.Spike.TOptional.*`.
- `Source/URLabEditor/URLabEditor.Build.cs` - adds `BlueprintGraph` (for `UEdGraphSchema_K2`).

Run: `Automation RunTests URLab.Spike` in `UnrealEditor-Cmd`, or the whole suite via
`Scripts/build_and_test.ps1`. Delete all three spike files at cutover.

## Verdict

**`TOptional` storage is safe to build the emitter on.** Every mechanism the plan depends on
passes, including the one flagged as least exercised (template-vs-instance delta). The
`bOverride_` fallback is not needed, wholesale or per-case.

Two rules the emitter must obey, both confirmed here rather than assumed:

1. Never emit `Replicated` on an optional. It is a hard UHT error.
2. Never emit `BlueprintReadWrite` / `BlueprintReadOnly` on an optional. It compiles clean and
   silently produces a broken Blueprint pin. Expose `UFUNCTION` accessors instead if Blueprint
   access is ever needed.

One accepted degradation: multi-selecting objects whose optionals differ in set/unset state
replaces the value editor with a three-item combo. Details below.

## Results by mechanism

| # | Mechanism | Test | Result |
|---|-----------|------|--------|
| - | Reflection, defaults, `Identical` set vs unset | `Defaults` | Pass |
| - | Property text format and import round-trip | `TextFormat` | Pass |
| - | Delta emission oracle (change / clear / add) | `DeltaEmission` | Pass |
| 1 | Blueprint SCS template save + disk reload | `ScsTemplateSaveLoad` | Pass |
| 2 | Level instance save + disk reload | `LevelInstanceSaveLoad` | Pass |
| 3 | Template-vs-instance delta through disk | `TemplateInstanceDelta` | Pass |
| 4 | Undo / redo across set, change, clear | `UndoRedo` | Pass |
| 5 | Duplication and component copy/paste | `DuplicateAndCopyPaste` | Pass |
| 6 | Details panel semantics + multi-select | `DetailsHandles` | Pass (with a documented degradation) |
| - | UHT hazard: `BlueprintReadWrite` | `BlueprintPinHazard` | Pass (hazard confirmed) |
| - | UHT hazard: `Replicated` | build probe, see below | Hard error confirmed |

Full suite after the spike: 335 `Result={Success}`, zero failures.

## Assertions used

### Baseline (`Defaults`)

- Every optional on a fresh component reports `IsSet() == false`.
- Every one reflects as `FOptionalProperty` with a non-null `GetValueProperty()`. This holds for
  all six inner types including `TArray<double>`, so UHT's `CanBeOptionalValue` gate never
  rejected any of them.
- `FProperty::Identical_InContainer` distinguishes all four cases: unset vs unset equal,
  unset vs set-to-default-value **not** equal, set(x) vs set(x) equal, set(x) vs set(y) not equal.
  The second is the one everything else rests on: absent and zero are distinguishable.

### Text format (`TextFormat`)

Exact exported text, `PPF_Copy`:

```
OptDouble  (1.500000)
OptVector  ((X=1.000000,Y=2.000000,Z=3.000000))
OptQuat    ((X=0.500000,Y=0.500000,Z=0.500000,W=0.500000))
OptArray   ((0.250000,-4.000000,9.750000))
OptEnum    (Beta)
OptString  <not exported>
```

Refinement of the handed-down "`()` unset, `(Value)` set" note, and it matters for any text-based
assertion: an unset optional exports as `()` **only when the baseline it is compared against has
it set**. Exported against a null baseline, `FProperty::ExportText_Direct` finds the value
identical to the default, returns `false`, and leaves the string untouched. So:

- `ExportText_InContainer(..., Delta=nullptr, ...)` on an unset optional returns `false` and the
  buffer stays empty.
- `ExportText_InContainer(..., Delta=<baseline with it set>, ...)` returns `true` and yields `()`.
- Importing `()` over an inherited set value clears it, rather than being a no-op.

Round-trip: each property exported from an authored source and imported into a target holding the
opposite state ends up `Identical` to the source, so import actively clears as well as sets.

### Delta emission (`DeltaEmission`)

The oracle is `FProperty::ExportText_InContainer` with the archetype as `Delta`: it returns true
exactly when a delta-serializing writer would emit the property. Against an archetype with five
optionals set:

- inherited `OptDouble`, `OptQuat`, `OptArray` emit nothing;
- inherited-unset `OptBlueprintInt` emits nothing;
- changed `OptVector` emits `((X=9.000000,Y=9.000000,Z=9.000000))`;
- **explicitly cleared `OptEnum` emits `()`** - the case that would silently revert to the
  template value if optionals were not delta-aware;
- newly set `OptString` emits `(added)`.

### Blueprint SCS template (`ScsTemplateSaveLoad`)

Creates an `AActor` blueprint with one SCS node of the spike component, sets five optionals and
leaves two unset, compiles, `SavePackage`, then forces a genuine disk reload (rename the package
aside, `ResetLoaders`, drop `RF_Standalone|RF_Public`, `CollectGarbage`, `LoadPackage`).

Asserted on the reloaded SCS template: all five set optionals carry their exact values, and both
unset optionals reload **unset**, not default-constructed.

### Level instance (`LevelInstanceSaveLoad`)

Editor world in its own package, two actors: one with an authored instance component, one with an
all-unset instance component. Saved as a `.umap`, reloaded from disk. Same value assertions on the
authored one; all seven optionals asserted still unset on the other.

### Template vs instance (`TemplateInstanceDelta`)

The path the plan is most exposed to. Blueprint template has `OptDouble`, `OptVector`, `OptEnum`
set. A placed instance then:

- inherits `OptDouble` (asserted set to the template's 1.5 at spawn),
- overrides `OptVector` to (9,9,9),
- **explicitly clears `OptEnum`** while the template has it set,
- leaves `OptString` never set.

Before saving, the delta against the archetype is asserted to be exactly `{OptVector, OptEnum}`.
The map is saved, unloaded, and the **template is then edited to `OptDouble = 2.5` and
recompiled**, so an inherited value cannot have been baked into the instance record. After
reloading the map from disk:

- `OptDouble == 2.5` (inherited, tracks the edited template),
- `OptVector == (9,9,9)` (override survives),
- `OptEnum` still unset despite the template having it set (explicit clear survives),
- `OptString` still unset.

### Undo / redo (`UndoRedo`)

Three separate `FScopedTransaction`s driven through `GEditor->UndoTransaction()` /
`RedoTransaction()`:

- set (`OptDouble`, `OptVector`, `OptArray` at once): undo returns all three to unset, redo
  restores exact values including array contents;
- value change: undo restores the prior value, redo reapplies the new one;
- clear (`OptDouble` and `OptArray`): undo restores set state and values, redo re-clears.

The transaction buffer therefore round-trips the set/unset flag, not just the payload.

### Duplication and copy/paste (`DuplicateAndCopyPaste`)

- `DuplicateObject` of the component: all values and unset states preserved.
- `StaticDuplicateObject` of the owning actor: same, checked on the duplicate's component.
- `FComponentEditorUtils::CopyComponents` / `PasteComponents` with an explicit buffer (the real
  T3D clipboard path, no clipboard or selection needed), copying one authored and one all-unset
  component together. Two components paste back, the authored one verified value by value and the
  unset one verified fully unset.

Observed clipboard payload:

```
Begin Object Class=/Script/URLabEditor.MjSpikeOptionalComponent Name="SpikeInstComp" ...
   OptDouble=(1.500000)
   OptVector=((X=1.000000,Y=2.000000,Z=3.000000))
   OptQuat=((X=0.500000,Y=0.500000,Z=0.500000,W=0.500000))
   OptArray=((0.250000,-4.000000,9.750000))
   OptEnum=(Beta)
   CreationMethod=Instance
End Object

Begin Object Class=/Script/URLabEditor.MjSpikeOptionalComponent Name="SpikeUnsetComp" ...
   CreationMethod=Instance
End Object
```

Unset optionals are absent rather than written as `()`, because they match the CDO. Asserted both
ways: the payload contains `OptDouble=(` and does not contain `OptString=`.

### Details panel (`DetailsHandles`)

Driven through `IPropertyRowGenerator` plus `IPropertyHandleOptional`, which is the same handle
API the details widgets call, so this is automated rather than eyeballed. What the engine actually
does, all asserted:

- **Unset optional keeps its own row.** The row's property is the `FOptionalProperty` and
  `IPropertyHandle::AsOptional()` is valid. `GetOptionalValue` returns `Success` with a null inner
  property. `SetOptionalValue(nullptr, nullptr)` succeeds and default-initialises the value. This
  row carries `EPropertyButton::OptionalSet`; `SPropertyEditorOptional` supplies the widget and
  the "This is an optional property..." tooltip.
- **Set optional collapses to its inner value row.** `FDetailPropertyRow` sets
  `bForceShowOnlyChildren` when an optional is set and the selection is not mixed
  (`Editor/PropertyEditor/Private/DetailPropertyRow.cpp`), so the row's own handle is the inner
  value, **not** the optional. Asserted: no `FOptionalProperty` row exists for a set optional; the
  inner row reads the value; the optional handle is reached via `GetParentHandle()->AsOptional()`
  and `ClearOptionalValue()` through it unsets the property. That parent call is what the row's
  `X` button (`EPropertyButton::OptionalClear`, added by
  `PropertyEditorHelpers::MakeRequiredPropertyButtons` for any child-of-option node) does. The
  name shown on that row is taken from the parent optional, so the user still sees the property
  name, with Set/None expressed as "Set button" vs "value plus X button".
- **Inner `FVector`** exposes the usual X/Y/Z child handles. Setting X through the handle writes
  through to the optional's value.
- **Inner `TArray<double>`** is the normal array UI: the row handle's `AsArray()` is valid, reports
  3 elements, and `AddItem()` grows the optional's array to 4.
- **Inner `FQuat`** produced a row with zero child rows. This is not a `TOptional` artifact: a
  plain non-optional `FQuat PlainQuat` control on the same component produces zero child rows too,
  and the test asserts the two child counts are equal.
- **Multi-select with mixed set/unset:** the optional row is retained (the mixed case is exempted
  from `bForceShowOnlyChildren`), `GetOptionalValue` returns `MultipleValues`, and
  `SetOptionalValue` applied to the mixed selection reaches both objects.

Manual pass, visual only. Everything semantic above is asserted, so what a human still has to
confirm is that the widgets render as the code says: the Set button on a None row, the X button
next to a set value, X/Y/Z spinboxes for a set `FVector`, the standard array header for a set
`TArray`. From `SPropertyEditorOptional::Construct`, a mixed selection replaces the value editor
with a combo box offering exactly `Multiple States` / `Set all to Value` / `Set all to None`; there
is no per-object editing while the selection is mixed, and no partial edit of the inner value. For
the emitter's use case (bulk-editing an attribute across several selected bodies where some
authored it and some did not) this is a real but acceptable degradation: select the uniform subset,
or set all then edit.

## UHT hazards

### `BlueprintReadWrite TOptional<int32>`: compiles, broken pin

Confirmed both ways. The build succeeds with no UHT error and no warning under
`-WarningsAsErrors`. At runtime the property carries `CPF_BlueprintVisible`, but

```
ConvertPropertyToPinType(OptBlueprintInt) = false, PinCategory = bad_type
```

so any Blueprint graph node built from it is a broken pin. The reason there is no diagnostic is
`UhtOptionalProperty.UpdateCaps` (`Engine/Source/Programs/Shared/EpicGames.UHT/Types/Properties/UhtOptionalProperty.cs`):
it clears `IsMemberSupportedByBlueprint`, then re-grants it whenever the inner type is
Blueprint-supported. `int32` is, so UHT considers the property fine while the K2 schema does not.

### `Replicated TOptional<int32>`: hard UHT error

Probed by temporarily adding `UPROPERTY(Replicated) TOptional<int32>` to the spike component,
building, then removing it. Build failed at header-tool time:

```
Plugins\UnrealRoboticsLab\Source\URLabEditor\Private\Tests\MjSpikeOptionalComponent.h(45): Error:
Replicated Optionals with MemoryImageAllocators are not yet supported
```

The check is unconditional on `EPropertyFlags.Net` regardless of inner type
(`UhtOptionalProperty.cs:220`); the message's mention of allocators is misleading. The probe was
reverted and the build re-verified green, so the tree is not left broken.

Neighbouring check worth knowing, same file: `'Struct' recursion via optionals is unsupported for
properties` fires if a `USTRUCT` contains a `TOptional` of itself. Relevant only if the emitter
ever nests a struct inside its own optional.

## Notes for the emitter

- Delta correctness is what makes this design work, and it is real: an instance that clears an
  attribute the template sets serialises `()` and reloads cleared. "Absent" is a first-class state
  end to end, not an alias for the default value.
- Test packages are written under `/Game/URLabSpikeTemp/` and removed afterwards. A loaded
  package's linker keeps the file open, so the tests call `ResetLoaders` and drop the package
  before deleting; otherwise the assets are left behind in `Content/`.
