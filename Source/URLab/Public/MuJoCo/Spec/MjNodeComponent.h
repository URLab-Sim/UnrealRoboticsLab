// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "State/MjStateProducer.h"

#include "MjNodeComponent.generated.h"

struct FMjArticulationState;
struct mjModel_;
struct mjData_;
typedef mjModel_ mjModel;
typedef mjData_ mjData;

class UMjNodeComponent;

/**
 * One raw child of an element, as the tree adapters last resolved it: the
 * storage slot the schema puts it in, or a mark that the child is not a spec
 * element at all.
 *
 * Kept by pointer AND serial. The pointer is what makes the batch cheap to
 * check against the live child list; the serial is what makes the check exact,
 * because a collected component's address can be handed to a new one and a
 * different element type would sit in a different slot.
 */
struct FMjChildSlot
{
	UMjNodeComponent* Node = nullptr;
	uint64 Serial = 0;
	int32 Slot = -1;
	bool bIsElement = false;
};

/**
 * A way an element's picture and what will be simulated have come apart.
 *
 * One value per rule, because a rule is cleared when it stops being broken and
 * a message string is not a key. The rows they produce all land in
 * `PreviewProblems`, which is what the user reads.
 */
enum class EMjPreviewProblem : uint8
{
	/**
	 * The effective shape has no picture and no asset to draw: a `mesh` or
	 * `hfield` geom that names none. MuJoCo will not compile it either.
	 */
	UndrawableShape,

	/** The scale handle cannot author this element's size, so a drag was refused. */
	ScaleNotEditable,

	/** A drag would have authored a size of zero or less, so it was refused. */
	NonPositiveSize,
};

/**
 * The base of every generated MJCF element component.
 *
 * A MuJoCo spec is not stored beside the component tree; the component tree
 * IS the spec. This class carries everything an element needs that is not
 * one of its schema attributes: authored name, identity, spec order,
 * provenance, the compiled-model id it bound to, and the transform write-back
 * that keeps the editor gizmo and the authored pose the same fact.
 *
 * Nothing here scales with the schema. The 145 element classes and their 1,533
 * attributes are emitted into MuJoCo/Gen; this is the fixed cost underneath them.
 */
UCLASS(Abstract, ClassGroup = (MuJoCo))
class URLAB_API UMjNodeComponent : public USceneComponent, public IMjStateProducer
{
	GENERATED_BODY()

public:
	UMjNodeComponent();

	// --- Authored name ----------------------------------------------------- //

	/**
	 * The element's MJCF name, presence-wrapped.
	 *
	 * A plain FString would collapse "unset" against "authored empty", and the
	 * nameless-element branch of the SDK's referrer-safe Rename has no UE
	 * equivalent without that distinction. Generated Visit hands this slot to the
	 * reader and writer at whichever field id the element's schema declares its
	 * identity attribute (`name` for most, `dclass` for <default>).
	 */
	UPROPERTY(EditAnywhere, Category = "MuJoCo")
	TOptional<FString> MjName;

	// --- Identity ---------------------------------------------------------- //

	/**
	 * Stable per-element identity, minted from a process-monotonic counter.
	 *
	 * It survives value edits and save/load, is re-minted on duplication so a
	 * copy is a new element, and is what the compile-time auto-namer keys the
	 * reserved names of unnamed elements on. Mint order is also spec order for
	 * a tree the reader built, which is why it is the deterministic tie-break the
	 * tree adapters fall back on when sibling indices collide.
	 */
	UPROPERTY(VisibleAnywhere, AdvancedDisplay, Category = "MuJoCo|Provenance")
	uint64 Serial = 0;

	/** Mint `Serial` if this element does not have one yet. Idempotent. */
	void EnsureSerial();

	/** Unconditionally mint a fresh `Serial`. */
	void MintSerial();

	// --- Spec order ---------------------------------------------------- //

	/**
	 * The element's position among the siblings sharing its storage slot.
	 *
	 * Declaration order is semantic in MJCF (joint order under a body determines
	 * the qpos layout), and it cannot rest on `GetAttachChildren()`:
	 * `USceneComponent::AttachChildren` is Transient and rebuilt from registration
	 * order at load, with no persistence contract for level instances. Only
	 * `USCS_Node::ChildNodes` is a serialized ordered array, and a level instance
	 * has none. So spec order is persisted here, stamped by the factories at
	 * creation and renumbered by the adapters on insert and remove.
	 *
	 * `INDEX_NONE` means unstamped, and it is the default because the default is
	 * what a component added by hand in the components panel arrives with. Zero
	 * is a position -- the first one -- so a hand-added joint used to claim the
	 * front of its body's joint list and silently rewrite the robot's qpos
	 * layout. An unstamped element instead orders after every stamped sibling in
	 * its slot, so adding one appends, and the walk that reads spec order stamps
	 * it where it landed.
	 */
	UPROPERTY(VisibleAnywhere, AdvancedDisplay, Category = "MuJoCo|Provenance")
	int32 SiblingIndex = INDEX_NONE;

	/**
	 * This element's children with their storage slots already resolved, so the
	 * schema tables are asked once per child rather than once per question.
	 *
	 * Reading a spec asks a parent for its children constantly -- the reader asks
	 * again for every element it inserts, and a whole-tree walk asks once per
	 * node -- and answering means resolving each child's element type and its
	 * slot under this parent through the dispatch tables. That made reading a
	 * model cost the square of the number of children a parent has.
	 *
	 * Not serialized and not a UPROPERTY: it is derived from the tree and from
	 * the schema, both of which are still there, so it is rebuilt rather than
	 * saved. It is also never trusted on its own -- every use checks it against
	 * the live child list first, which is what makes a structural edit made
	 * behind the adapters' back correct itself instead of going unnoticed.
	 */
	mutable TArray<FMjChildSlot> ChildSlotCache;

	// --- Provenance -------------------------------------------------------- //

	/** The MJCF file this element was read from; empty when authored in-editor. */
	UPROPERTY(VisibleAnywhere, AdvancedDisplay, Category = "MuJoCo|Provenance")
	FString SourceFile;

	/** The 1-based line of `SourceFile` this element was read from; 0 when unknown. */
	UPROPERTY(VisibleAnywhere, AdvancedDisplay, Category = "MuJoCo|Provenance")
	int32 SourceLine = 0;

	// --- Reference diagnostics --------------------------------------------- //

	/**
	 * This element's reference attributes that name nothing in the spec.
	 *
	 * A reference is a NAME, resolved at compile time, so a typo or a renamed
	 * target is silent until MuJoCo refuses the model -- and it refuses it from
	 * the compiler, with no line and no component to blame. Recorded here so the
	 * element carrying the bad name is the one that says so, at the moment it
	 * becomes bad rather than at the moment somebody presses play.
	 *
	 * Transient: it describes the spec as it is right now, and the spec is what
	 * it is derived from.
	 */
	UPROPERTY(VisibleAnywhere, AdvancedDisplay, Transient, Category = "MuJoCo|Diagnostics")
	TArray<FString> DanglingReferences;

	/**
	 * Why the schema cannot keep this element where it has been put.
	 *
	 * MJCF says which elements may contain which, and a component parented
	 * against that is not an error the user gets told about at the time: the
	 * element is simply absent from the compiled model, and the model still
	 * compiles. Recorded on the element itself, the same surface and for the same
	 * reason as `DanglingReferences` -- a transient toast is gone before the user
	 * has finished adding the next one.
	 *
	 * Re-derived on every registration, so moving the element somewhere legal
	 * clears it.
	 */
	UPROPERTY(VisibleAnywhere, AdvancedDisplay, Transient, Category = "MuJoCo|Diagnostics")
	TArray<FString> PlacementProblems;

	/**
	 * Where this element's picture and what will be simulated disagree.
	 *
	 * The viewport is the only report most of these have: a geom that resolves to
	 * a shape with nothing to draw simply is not there, and a scale handle that
	 * cannot author a size looks identical to one that did. Neither is an error
	 * the model reaches the compiler with a line number for, and a toast is gone
	 * before the user has finished the gesture that caused it.
	 *
	 * Transient, and keyed by `EMjPreviewProblem` rather than by text, so a rule
	 * that stops being broken takes its own row away and leaves the others.
	 */
	UPROPERTY(VisibleAnywhere, AdvancedDisplay, Transient, Category = "MuJoCo|Diagnostics")
	TArray<FString> PreviewProblems;

	/**
	 * Record `Message` as this element's row for `Problem`, replacing any earlier
	 * one, and say it once in the message log and the run log.
	 *
	 * Once per element and per rule: the row is the persistent surface and is
	 * rewritten freely, while the logs are a transition and would otherwise
	 * repeat on every registration for the rest of the session. Keyed on `Serial`
	 * for the same reason `CheckPlacementLegality` is -- a component is not the
	 * same object across a Blueprint reconstruct, and a new element is a new
	 * mistake.
	 */
	void NotePreviewProblem(EMjPreviewProblem Problem, const FString& Message);

	/** Drop this element's row for `Problem`, and let it be said again if it returns. */
	void ClearPreviewProblem(EMjPreviewProblem Problem);

	// --- Runtime binding --------------------------------------------------- //

	/** Record an id resolved elsewhere (the engine's one pass over the binding). */
	void BindTo(int32 Id);

	/** Forget the compiled id. Called when the compiled model is discarded. */
	void Unbind();

	/** The compiled-model id, or unset when this element is not bound. */
	const TOptional<int32>& GetBoundId() const { return BoundId; }

	// --- State production -------------------------------------------------- //

	/**
	 * Declare this element's per-step state into the articulation IR.
	 *
	 * Runs on the physics worker inside the engine's fenced scope: the model and
	 * data arrive as call parameters and must not be retained.
	 */
	virtual void DescribeState(const mjModel* m, mjData* d, FMjArticulationState& Out) const override {}

	// --- Preview transform ------------------------------------------------- //

	/**
	 * Drive the component's relative transform from the element's effective pose
	 * and, where the element has one, its effective size.
	 *
	 * Effective, not authored: a geom whose `pos` comes from its default class
	 * has to preview where MuJoCo will put it. Write-back stays authored-only, so
	 * previewing an inherited value never authors it.
	 *
	 * Spec to preview only: `SetRelativeLocationAndRotation` fires no editor
	 * hooks, so this can never re-enter the write-back below. It also establishes
	 * `LastPreviewTransform`, which is what makes the write-back's presence guard
	 * a guard rather than a coin toss.
	 */
	void SyncPreviewFromSpec();

	/**
	 * The same, with one whole-spec index held open around it.
	 *
	 * A single sync asks the default-class chain several questions, and each of
	 * them indexes the spec unless something already has. This is the entry point
	 * for the moments where the sync is the whole of the work -- load, register,
	 * undo -- and it is a no-op wrapper inside a pass that already holds a scope.
	 */
	void SyncPreviewUnderOneScope();

	/**
	 * The relative transform the spec says this element sits at.
	 *
	 * What `SyncPreviewFromSpec` would apply, without applying it, so the
	 * write-back can recover a cold baseline without moving the component it is
	 * about to read. False when the element has no pose attributes at all.
	 */
	bool ComputePreviewTransform(FTransform& Out);

	/** True when this element's schema declares `pos` and an orientation. */
	bool HasPoseAttributes() const;

	// --- What an element is in the spec -------------------------------- //

	/**
	 * True when this element is a `<default>` class, or sits inside one.
	 *
	 * A class partial is an inheritance template, not scene content: MuJoCo never
	 * places it, never compiles it and never draws it. Its attributes are whatever
	 * the class chose to declare, which need not describe a standalone element --
	 * `<default class="body"><geom type="capsule" material="body"/></default>` has
	 * a shape and no size -- so anything that previews it is previewing a value
	 * that was never meant to stand on its own.
	 *
	 * The question is answered by position, not by a flag: a partial IS a child of
	 * the `<default>` carrying the class, and `<default>` nests.
	 */
	bool IsClassPartial() const;

	/**
	 * True when other elements resolve their own presentation through this one.
	 *
	 * Two elements of a spec are shared in this sense, and the schema says
	 * which by where it puts them: a `<default>` class is an inheritance layer
	 * every element naming it reads through, and an `<asset>` resource is a named
	 * thing elements point at. Everything else is content, and content decides
	 * only its own picture.
	 *
	 * Being under either counts, because a change anywhere beneath a shared node
	 * changes what resolving through it yields -- `<material><layer/></material>`
	 * and `<default><geom/></default>` alike.
	 */
	bool IsSharedPresentationInput() const;

	// --- Presentation ------------------------------------------------------ //

	/**
	 * Re-derive everything about this element's picture from the spec.
	 *
	 * The base is the pose and size preview; `UMjGeom` adds the meshes and their
	 * colour. Overriding this is what a future element with a picture has to do,
	 * and is all it has to do: the propagation below finds it without being told.
	 */
	virtual void RefreshPresentation();

	/**
	 * Re-derive the picture of every element of the spec this one belongs to.
	 *
	 * Editing a shared node changes what everything inheriting from or pointing at
	 * it looks like, and presentation is a push: a geom's colour is baked into a
	 * dynamic material instance when its visualiser is built, and nothing re-reads
	 * it afterwards. So the edit has to say so.
	 *
	 * Deliberately a spec-wide re-derivation rather than a walk of the edited
	 * node's dependents. A dependent list is a reverse index over references and
	 * class chains that has to be built, kept current across every rename and
	 * reparent, and extended by hand the day a new element type resolves a
	 * material -- three ways to be silently wrong. Asking every element to re-read
	 * the spec it already knows how to read cannot go stale, and a new
	 * referencing element type is covered the moment it overrides
	 * `RefreshPresentation`. The cost is bounded by making the trigger the narrow
	 * thing instead: only an edit to a shared node runs this, and a shared node is
	 * edited by hand, one keystroke at a time.
	 */
	void RefreshSpecPresentation();

	// --- Scale, and what it is allowed to mean ----------------------------- //
	//
	// Which of the three answers an element gets is a fact about the schema, not
	// a list kept here: see `MjScalePolicy.h`. A `<geom>` and a `<site>` both
	// carry a `size` a GeomType decides, so both are edited by the scale handle
	// under their own type's lock; an element with no size refuses the drag by
	// snapping back to one, because a scale it cannot express is a picture that
	// disagrees with the simulation.

	/**
	 * The relative scale this element's effective size implies, if any.
	 *
	 * False for an element with no size, and for one whose size cannot be
	 * resolved -- too short for its type, or non-positive -- which leaves the
	 * component where it is rather than collapsing it to nothing.
	 */
	virtual bool TryPreviewScaleFromSpec(FVector& OutScale) const;

	/** True when a scale drag is an edit of this element's size. */
	virtual bool HasScaleMapping() const;

	/**
	 * Snap the component's scale onto what this element can represent.
	 *
	 * A sphere has one radius, so a non-uniform drag has no meaning; the rule is
	 * to constrain visibly rather than to pick a component or to refuse. An
	 * element with no size at all has no such rule to apply and snaps back to
	 * one. Runs before the write-back compares, so the user sees the snap on the
	 * spot and the spec only ever receives a representable scale.
	 */
	virtual void ConstrainPreviewScale();

	/**
	 * Author this element's size from a relative scale, under its own arity.
	 *
	 * False when nothing was authored, which is not a failure but a refusal with
	 * a reason: the element's size is not a scale, or the scale asks for a size
	 * of zero or less, which MuJoCo's own `checksize` will not accept. The caller
	 * puts the component back where the spec still says it is, so a refusal is
	 * something the user sees rather than a silent divergence.
	 */
	virtual bool WriteBackScale(const FVector& Scale);

	// --- Reference dropdowns ----------------------------------------------- //

	/**
	 * Every name the reference field at `FieldId` could legally point at.
	 *
	 * The generated `Get<Field>Options` accessors call this, one per reference
	 * attribute, and the details panel's `GetOptions` meta calls those. It is a
	 * member rather than a free function because that is the only spelling
	 * available inside a generated class body: the table it forwards to lives in
	 * the dispatch unit, which includes every element header and so cannot be
	 * included by one. Deliberately not inline, for the same reason.
	 */
	TArray<FString> RefNameOptions(int32 FieldId) const;

	// --- UObject / USceneComponent ----------------------------------------- //

	virtual void PostInitProperties() override;
	virtual void PostLoad() override;
	virtual void OnComponentCreated() override;
	virtual void PostDuplicate(bool bDuplicateForPIE) override;

	/**
	 * Blueprint recompile reinstancing, drag-into-level, and PIE spawn.
	 *
	 * SCS templates never register, which is why `PostLoad` carries the same
	 * sync: a template receives gizmo moves and so needs the baseline too.
	 */
	virtual void OnRegister() override;

#if WITH_EDITOR
	/**
	 * Judge this element's parentage against the schema's child slots.
	 *
	 * Fills `PlacementProblems`, and says so once in the message log and the run
	 * log for each element that becomes illegally placed. On `OnRegister`, which
	 * is the one moment BOTH ways of adding a component reach: the components
	 * panel of a placed actor and the Blueprint's construction script, whose
	 * templates reach it through the preview actor built from them.
	 */
	void CheckPlacementLegality();
#endif

#if WITH_EDITOR
	/**
	 * Gizmo drags, both viewports, template and instance.
	 *
	 * Acts on every invocation regardless of `bFinished`, because an SCS template
	 * only ever receives false.
	 */
	virtual void PostEditComponentMove(bool bFinished) override;

	/**
	 * The only moment the name an element is about to lose still exists.
	 *
	 * A rename has to reach the elements that refer to this one, and they refer
	 * to it BY the old name -- so without capturing it here there is nothing left
	 * to search the spec for once the edit has landed.
	 */
	virtual void PreEditChange(FProperty* PropertyAboutToChange) override;

	/** Details-panel edits of the Relative* members and of spatial attributes. */
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;

	/** Undo restores properties without hooks, leaving the baseline stale. */
	virtual void PostEditUndo() override;
#endif

protected:
	/**
	 * Write the component's relative transform back into the element's `pos`,
	 * orientation and size, but only the parts that actually differ from what
	 * `SyncPreviewFromSpec` last applied.
	 *
	 * The guard is load-bearing, not an optimisation. `PostEditComponentMove`
	 * recurses into every attached child unconditionally, so dragging a body
	 * delivers the hook to descendants whose own relative transform did not move.
	 * An unconditional write would author pose presence on every one of them.
	 * `pos` is defaultable, so unset and explicit zero differ: the compiled model
	 * would be unchanged at that moment, but the element would have silently lost
	 * its inheritance link to its default class.
	 *
	 * A cold cache is not the absence of a baseline: the cache is a cache OF the
	 * spec, and the spec is still there to compare against. So the unset
	 * case fills the baseline from the spec and goes on to compare, rather
	 * than syncing and returning -- which discarded the move that had just
	 * happened and put the component back, losing the drag.
	 *
	 * Each attribute is written only when its own component moved. An element
	 * with `pos` but no `quat` (light, joint, replicate) ignores rotation deltas
	 * outright: mapping a rotation onto `light.dir` or `joint.axis` is a
	 * deliberate follow-on, not something a drag should do by accident.
	 */
	void WriteBackTransformIfChanged();

	/** The compiled-model id, or unset. Runtime only: never serialized. */
	TOptional<int32> BoundId;

	/**
	 * Which rule each row of `PreviewProblems` came from, in the same order.
	 *
	 * Kept beside the rows rather than in them so that clearing one rule does not
	 * mean matching on the text it wrote. Not a UPROPERTY, for the same reason the
	 * rows are Transient: both are derived from the spec as it is right now.
	 */
	TArray<EMjPreviewProblem> PreviewProblemKeys;

	/**
	 * The relative transform `SyncPreviewFromSpec` last applied, and the
	 * reference the write-back's change detector compares against. Transient by
	 * construction: a reload re-derives it from the spec, on `PostLoad` for
	 * a saved level or an SCS template and on `OnRegister` for everything else.
	 */
	TOptional<FTransform> LastPreviewTransform;

#if WITH_EDITOR
	/** What `MjName` held when the details panel announced it was about to change. */
	TOptional<FString> NameBeforeEdit;
#endif
};

namespace urlab::spec
{
/**
 * Record, on every element of the spec rooted at `Root`, the reference
 * attributes it carries that name nothing the spec declares.
 *
 * Whole-spec rather than per-element because that is the question: a reference
 * dangles relative to the set of names the model declares, and no element knows
 * that set on its own. `Adapter` is the tree adapter for the graph the elements
 * live in.
 */
template <class Adapter>
void MjNoteDanglingReferences(UMjNodeComponent& Root);
}  // namespace urlab::spec
