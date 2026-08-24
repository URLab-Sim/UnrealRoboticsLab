// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The element's authored `pos` and `quat` are reached through the schema's own
// reflection tables (field id by MJCF attribute name) and the profile's field
// accessors, so no per-element code exists here and none is generated for it.
// What varies is the storage, and that is the Shape policy's business.

#include "MuJoCo/Spec/MjNodePose.h"

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjSpecProfile.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Utils/URLabAxisConv.h"

#if URLAB_MJ_GEN
#include <vector>
THIRD_PARTY_INCLUDES_START
#include "protospec/core.h"
#include "protospec/detail.h"
#include "protospec/model_core.h"
#include "protospec/profile.h"
THIRD_PARTY_INCLUDES_END
#include "MuJoCo/Gen/MjDispatch.gen.h"
#endif

#if URLAB_MJ_GEN

namespace
{
namespace psm = ps::mjcf;
namespace pssdk = ps::sdk;

/** How close two transforms must be before a gizmo drag counts as a no-op. */
constexpr double MjPreviewEpsilon = UE_KINDA_SMALL_NUMBER;

struct FMjPoseFieldIds
{
	int Pos = -1;
	int Quat = -1;

	bool IsValid() const { return Pos >= 0 || Quat >= 0; }
};

FMjPoseFieldIds PoseFieldIdsOf(psm::ElementType Type)
{
	FMjPoseFieldIds Out;
	Out.Pos = ps::sdk::internal::FieldIdByName(Type, "pos");
	Out.Quat = ps::sdk::internal::FieldIdByName(Type, "quat");
	return Out;
}

/** The element's authored MuJoCo-space pose, in MJCF component order. */
bool ReadMjPose(UMjNodeComponent& Node, double OutPos[3], double OutQuat[4], bool& bPosSet, bool& bQuatSet)
{
	using P = urlab::spec::FMjInstanceProfile;
	bPosSet = false;
	bQuatSet = false;
	bool bAny = false;
	urlab::spec::gen::DispatchByType(Node, [&](auto& Element) {
		using E = std::decay_t<decltype(Element)>;
		const FMjPoseFieldIds Ids = PoseFieldIdsOf(urlab::spec::gen::TMjElementType<E>::Value);
		if (!Ids.IsValid())
		{
			return;
		}
		bAny = true;
		if (Ids.Pos >= 0)
		{
			bPosSet = urlab::spec::ReadFixed<P, 3>(Element, Ids.Pos, OutPos);
		}
		if (Ids.Quat >= 0)
		{
			bQuatSet = urlab::spec::ReadFixed<P, 4>(Element, Ids.Quat, OutQuat);
		}
	});
	return bAny;
}

/**
 * Fill whichever of `pos` and `quat` the element did not author from its
 * default-class chain.
 *
 * Reached only when the authored read left something unset, because resolving a
 * chain means indexing the whole spec and the answer is the same either way
 * when the element authored the value itself.
 */
void FillMjPoseFromClasses(UMjNodeComponent& Node, double OutPos[3], double OutQuat[4], bool& bPosSet, bool& bQuatSet)
{
	urlab::spec::WithEffectiveDoc(Node, [&](auto& Effective) {
		using P = typename std::decay_t<decltype(Effective)>::ProfileType;
		urlab::spec::gen::DispatchByType(Node, [&](auto& Element) {
			using E = std::decay_t<decltype(Element)>;
			const FMjPoseFieldIds Ids = PoseFieldIdsOf(urlab::spec::gen::TMjElementType<E>::Value);
			if (!bPosSet && Ids.Pos >= 0)
			{
				Effective.ForEachLayer(Element, [&](const auto& Layer) {
					return (bPosSet = urlab::spec::ReadFixed<P, 3>(Layer, Ids.Pos, OutPos));
				});
			}
			if (!bQuatSet && Ids.Quat >= 0)
			{
				Effective.ForEachLayer(Element, [&](const auto& Layer) {
					return (bQuatSet = urlab::spec::ReadFixed<P, 4>(Layer, Ids.Quat, OutQuat));
				});
			}
		});
	});
}

/** Author `pos`, `quat`, or both. An id below zero means the element has none. */
void WriteMjPose(UMjNodeComponent& Node, const double* InPos, const double* InQuat)
{
	using P = urlab::spec::FMjInstanceProfile;
	urlab::spec::gen::DispatchByType(Node, [&](auto& Element) {
		using E = std::decay_t<decltype(Element)>;
		const FMjPoseFieldIds Ids = PoseFieldIdsOf(urlab::spec::gen::TMjElementType<E>::Value);
		if (InPos != nullptr && Ids.Pos >= 0)
		{
			const double Values[3] = {InPos[0], InPos[1], InPos[2]};
			pssdk::internal::SetFixedField<P, 3>(Element, Ids.Pos, Values);
		}
		if (InQuat != nullptr && Ids.Quat >= 0)
		{
			const double Values[4] = {InQuat[0], InQuat[1], InQuat[2], InQuat[3]};
			pssdk::internal::SetFixedField<P, 4>(Element, Ids.Quat, Values);
		}
	});
}

/** True when this element's schema declares an orientation attribute. */
bool HasQuatAttribute(UMjNodeComponent& Node)
{
	bool bHas = false;
	urlab::spec::gen::DispatchByType(Node, [&](auto& Element) {
		using E = std::decay_t<decltype(Element)>;
		bHas = PoseFieldIdsOf(urlab::spec::gen::TMjElementType<E>::Value).Quat >= 0;
	});
	return bHas;
}
} // namespace

namespace urlab::spec
{
bool MjNodeHasPoseAttributes(const UMjNodeComponent& Node)
{
	bool bHas = false;
	urlab::spec::gen::DispatchByType(const_cast<UMjNodeComponent&>(Node), [&](auto& Element) {
		using P = urlab::spec::FMjInstanceProfile;
		using E = std::decay_t<decltype(Element)>;
		bHas = PoseFieldIdsOf(urlab::spec::gen::TMjElementType<E>::Value).IsValid();
	});
	return bHas;
}

bool MjNodeComputePreviewTransform(UMjNodeComponent& Node, FTransform& Out)
{
	// A class default object is not an element and has no spec to read.
	if (Node.HasAnyFlags(RF_ClassDefaultObject))
	{
		return false;
	}

	double MjPos[3] = {0.0, 0.0, 0.0};
	double MjQuat[4] = {1.0, 0.0, 0.0, 0.0};
	bool bPosSet = false;
	bool bQuatSet = false;
	if (!ReadMjPose(Node, MjPos, MjQuat, bPosSet, bQuatSet))
	{
		return false;
	}
	if (!bPosSet || !bQuatSet)
	{
		FillMjPoseFromClasses(Node, MjPos, MjQuat, bPosSet, bQuatSet);
	}

	// A pose no layer authored previews at the schema default (origin, identity),
	// which is what MuJoCo compiles it to. Reading it back out of the preview is
	// what the write-back's change detector then suppresses.
	//
	// And a scale no layer authored previews at the shape's own default, which
	// for every element that has no resolvable size is one. NOT the component's
	// current scale: this transform is the write-back's baseline whenever the
	// cache is cold, and seeding it from the very component the user is dragging
	// folds the drag into the baseline -- "nothing moved", no snap, and a sphere
	// that previews as an ellipsoid while compiling as MuJoCo's default sphere.
	FVector Scale = FVector::OneVector;
	Node.TryPreviewScaleFromSpec(Scale);
	Out = FTransform(URLabAxisConv::MjQuatToUe(MjQuat), URLabAxisConv::MjPositionToUe(MjPos), Scale);
	return true;
}

void MjNodeSyncPreviewFromSpec(UMjNodeComponent& Node)
{
	FTransform Preview;
	if (!MjNodeComputePreviewTransform(Node, Preview))
	{
		return;
	}

	FVector Scale = Node.GetRelativeScale3D();
	const bool bHasScale = Node.TryPreviewScaleFromSpec(Scale);

	Node.LastPreviewTransform = Preview;
	Node.SetRelativeLocationAndRotation(Preview.GetLocation(), Preview.GetRotation());
	if (bHasScale)
	{
		Node.SetRelativeScale3D(Preview.GetScale3D());
	}
}

void MjNodeWriteBackTransformIfChanged(UMjNodeComponent& Node)
{
	if (Node.HasAnyFlags(RF_ClassDefaultObject))
	{
		return;
	}

	// The scale answer does not depend on the element having a pose, and tying
	// the two together is how a whole model came to be drawn at a scale MuJoCo
	// never sees. A `<mujoco>` root authors neither `pos` nor `quat`, so it left
	// here at once -- and it is the component a placed actor's own transform
	// lands on. Scale the actor and every geom under it is drawn stretched while
	// the simulation goes on using the `size` the gesture never touched, which is
	// the picture and the model saying different things.
	//
	// Every element the schema gives a `size` also gives a transform, so a
	// pose-less element is an unsized one and the scale it can hold is one. That
	// makes the guard exact rather than a heuristic, and it is a guard rather
	// than an unconditional call because resolving a shape indexes the spec.
	if (!MjNodeHasPoseAttributes(Node))
	{
		if (!Node.GetRelativeScale3D().Equals(FVector::OneVector, MjPreviewEpsilon))
		{
			urlab::spec::FMjEffectiveScope Effective(Node);
			Node.ConstrainPreviewScale();
		}
		return;
	}

	// No cached baseline is not the same as no baseline. The cache is a cache OF
	// the spec, so when it is cold the spec itself answers the question,
	// and the comparison below is the same comparison it always was.
	//
	// Syncing and returning here instead -- which is what this did -- discards
	// the move that just happened and puts the component back where the spec
	// says it is. That is the transform widget moving and the geom staying put,
	// and it is this guard failing in the direction that loses the user's input
	// rather than the direction that protects the spec. The protection is
	// unaffected: a descendant that received the hook without moving still
	// matches the spec and still authors nothing.
	if (!Node.LastPreviewTransform.IsSet())
	{
		FTransform FromSpec;
		if (!MjNodeComputePreviewTransform(Node, FromSpec))
		{
			return;
		}
		Node.LastPreviewTransform = FromSpec;
	}

	// The scale is settled BEFORE anything is compared, so the comparison and
	// the spec both see a scale the element can actually hold. Running the lock
	// after the early-out below instead would mean it never runs at all in the
	// one case it exists for: a freshly added geom whose baseline is cold and
	// whose size is unset would fold the dragged scale into its own baseline,
	// "nothing moved", and the non-uniform scale would stand.
	//
	// Guarded on the scale differing, and that guard is the cost, not the
	// correctness: moving an actor delivers this hook to every descendant, not
	// one of whose relative transforms changed, and resolving a shape indexes
	// the entire spec once per element that asks. A descendant that did not move
	// matches its baseline on all three components, and its baseline scale came
	// from the spec through the shape's own mapping, so the snap would be a
	// no-op there by construction.
	const FVector CurrentScale = Node.GetRelativeScale3D();
	TUniquePtr<urlab::spec::FMjEffectiveScope> Effective;
	if (!Node.LastPreviewTransform->GetScale3D().Equals(CurrentScale, MjPreviewEpsilon))
	{
		Effective = MakeUnique<urlab::spec::FMjEffectiveScope>(Node);
		Node.ConstrainPreviewScale();
	}

	if (Node.LastPreviewTransform->Equals(
			FTransform(Node.GetRelativeRotation().Quaternion(), Node.GetRelativeLocation(), Node.GetRelativeScale3D()),
			MjPreviewEpsilon))
	{
		return;
	}

	// Past the early-out, so the descendants of a moving actor never pay for it:
	// everything below resolves the class chain more than once, and each instance
	// below opens its own, an instance being a separate spec with its own root.
	if (!Effective.IsValid())
	{
		Effective = MakeUnique<urlab::spec::FMjEffectiveScope>(Node);
	}

	// The pose this element was at before the drag. It is the baseline the change
	// detector uses below, and it is also how the propagation at the end tells an
	// instance that was following this element from one that had moved itself.
	const FTransform Baseline = Node.LastPreviewTransform.GetValue();
	const FTransform Current(Node.GetRelativeRotation().Quaternion(), Node.GetRelativeLocation(), Node.GetRelativeScale3D());
	const bool bPosMoved = !Node.LastPreviewTransform->GetLocation().Equals(Current.GetLocation(), MjPreviewEpsilon);
	const bool bRotMoved = HasQuatAttribute(Node) && !Node.LastPreviewTransform->GetRotation().Equals(Current.GetRotation(), MjPreviewEpsilon);
	const bool bScaleMoved =
		Node.HasScaleMapping() && !Node.LastPreviewTransform->GetScale3D().Equals(Current.GetScale3D(), MjPreviewEpsilon);

	if (!bPosMoved && !bRotMoved && !bScaleMoved)
	{
		return;
	}

	double MjPos[3] = {0.0, 0.0, 0.0};
	double MjQuat[4] = {1.0, 0.0, 0.0, 0.0};
	URLabAxisConv::UePositionToMj(Current.GetLocation(), MjPos);
	URLabAxisConv::UeQuatToMj(Current.GetRotation(), MjQuat);

	// Join the open undo transaction before authoring anything.
	//
	// The gizmo opens one and records the component's own transform, but the
	// MJCF attributes below are separate properties and are not in that record
	// unless the object asks to be. Without this an undo restores the transform,
	// leaves `pos` at its new value, and the next sync puts that value straight
	// back on the component -- so the undo appears to do nothing at all.
	//
	// A no-op outside a transaction, which is what the paths that author without
	// a user edit behind them want.
	Node.Modify();

	// Each attribute is authored only where its own component moved. Writing the
	// unmoved half as well would author a value the element was inheriting.
	WriteMjPose(Node, bPosMoved ? MjPos : nullptr, bRotMoved ? MjQuat : nullptr);
	const bool bScaleAuthored = bScaleMoved && Node.WriteBackScale(Current.GetScale3D());

	// A refused scale is a refusal in the viewport too. The spec still says what
	// it said, so a component left holding the scale that was refused is a
	// picture of a model that does not exist -- and for the collapse this refusal
	// exists for, that picture is a geom that has vanished. It goes back to the
	// pose it was dragged to and the scale the spec still holds.
	FTransform Settled = Current;
	if (bScaleMoved && !bScaleAuthored)
	{
		Settled.SetScale3D(Baseline.GetScale3D());
		Node.SetRelativeScale3D(Settled.GetScale3D());
	}
	Node.LastPreviewTransform = Settled;

	// The Blueprint editor's viewport does not drag the component the user can
	// see. It applies the delta to the construction-script TEMPLATE and leaves
	// the preview actor's own components holding the pose they had, then carries
	// `RelativeLocation` across for the ones that had not overridden it.
	//
	// Nothing carries the authored attribute, and the gap does not stay
	// cosmetic. Re-running the construction scripts -- which that same drag does,
	// on every delta -- reads every attribute an instance holds differently from
	// its template as an instance OVERRIDE, caches it, and puts it back on the
	// rebuilt component. So the moment the template's `pos` moves, the instance's
	// unchanged `pos` becomes an override of it, and the preview is pinned to the
	// pre-drag pose for the rest of the session while the widget goes on moving.
	// That is the transform that cannot be edited.
	//
	// Attribute by attribute, and only onto an instance that still holds this
	// element's own pre-drag value: one that had been moved on its own really has
	// overridden the template and must keep what it authored.
	urlab::spec::MjNodeForEachInstanceOfTemplate(FSpecRef::OverOwner(&Node), Node, [&](UMjNodeComponent& Instance) {
		const bool bTakePos =
			bPosMoved && Instance.GetRelativeLocation().Equals(Baseline.GetLocation(), MjPreviewEpsilon);
		const bool bTakeRot = bRotMoved && Instance.GetRelativeRotation().Quaternion().Equals(Baseline.GetRotation(), MjPreviewEpsilon);
		const bool bTakeScale =
			bScaleAuthored && Instance.GetRelativeScale3D().Equals(Baseline.GetScale3D(), MjPreviewEpsilon);
		if (!bTakePos && !bTakeRot && !bTakeScale)
		{
			return;
		}
		// The instance is a different object from the template being dragged, so
		// it carries its own undo record or none.
		Instance.Modify();
		WriteMjPose(Instance, bTakePos ? MjPos : nullptr, bTakeRot ? MjQuat : nullptr);
		if (bTakeScale)
		{
			Instance.WriteBackScale(Current.GetScale3D());
		}
		Instance.SyncPreviewFromSpec();
	});
}
} // namespace urlab::spec

#endif // URLAB_MJ_GEN
