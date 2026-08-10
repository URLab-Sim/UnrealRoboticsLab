// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MjFromtoFold.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "MuJoCo/Gen/MjEnums.gen.h"
#include "MuJoCo/Gen/MjReflect.gen.h"

THIRD_PARTY_INCLUDES_START
#include "resolve.h"
THIRD_PARTY_INCLUDES_END

namespace urlab::spec
{
namespace
{

// mjEPS (user_util.h:31), verbatim, so the "points too close" boundary is the
// engine's rather than an approximation of it.
constexpr double MjEps = 1e-14;

/** The shapes `fromto` is defined for (mjCGeom::Compile, user_objects.cc:3979). */
bool AdmitsFromto(EMjGeomType Type)
{
	return Type == EMjGeomType::capsule || Type == EMjGeomType::cylinder ||
		Type == EMjGeomType::ellipsoid || Type == EMjGeomType::box;
}

/** The attribute ids the fold needs, or -1 where the element has none. */
struct FFromtoFieldIds
{
	int Fromto = -1;
	int Pos = -1;
	int Quat = -1;
	int Size = -1;
	int Type = -1;

	bool IsComplete() const { return Fromto >= 0 && Pos >= 0 && Quat >= 0 && Size >= 0 && Type >= 0; }
};

FFromtoFieldIds FieldIdsOf(psm::ElementType Type)
{
	FFromtoFieldIds Out;
	Out.Fromto = pssdk::internal::FieldIdByName(Type, "fromto");
	Out.Pos = pssdk::internal::FieldIdByName(Type, "pos");
	Out.Quat = pssdk::internal::FieldIdByName(Type, "quat");
	Out.Size = pssdk::internal::FieldIdByName(Type, "size");
	Out.Type = pssdk::internal::FieldIdByName(Type, "type");
	return Out;
}

/**
 * What `mjuu_normvec` (user_util.cc:141) returns: the length, or zero below
 * mjEPS.
 *
 * Only the return value, because the fold wants the half-length and hands the
 * direction to ps::core untouched. Spelled as the engine spells it -- an
 * accumulating sum then one sqrt -- since the half-length is written into the
 * spec and compared against the engine's own bit for bit.
 */
double VectorNorm(const double* Vec, int32 Num)
{
	double SumSquares = 0.0;
	for (int32 Index = 0; Index < Num; ++Index)
	{
		SumSquares += Vec[Index] * Vec[Index];
	}
	return SumSquares < MjEps ? 0.0 : FMath::Sqrt(SumSquares);
}

/**
 * The quaternion MuJoCo derives from a capsule's axis.
 *
 * ProtoSpec's `ps::core` already carries `mjuu_normvec` and `mjuu_z2quat` lifted
 * verbatim from the engine, drift-gated against the originals, and its ZAxis
 * case is that exact pair in that exact order -- which is what
 * `mjCGeom::Compile` runs on a `fromto`. Calling it rather than keeping a third
 * copy is not tidiness: the differential gate compares compiled models BYTE for
 * byte, and a second implementation of the same formula only has to reassociate
 * one expression to land a ULP away. This module's own copy did exactly that.
 *
 * `Vec` is the raw difference of the two ends, not a unit vector: the resolver
 * normalizes internally, including the engine's "leave it alone if it is already
 * unit to within mjEPS" rule, which is where the divergence was.
 */
void AxisToQuat(const double Vec[3], double OutQuat[4])
{
	const std::array<double, 4> Quat =
		ps::core::ResolveOrientation(ps::core::OrientKind::ZAxis, Vec, ps::core::OrientContext{});
	OutQuat[0] = Quat[0];
	OutQuat[1] = Quat[1];
	OutQuat[2] = Quat[2];
	OutQuat[3] = Quat[3];
}

/**
 * What folding one element did, and to whose `fromto` it applies.
 *
 * The layers are carried out of the fold because retiring a class's `fromto` is
 * a decision about every element that reads it, not about any one of them.
 */
struct FFoldOutcome
{
	/** The `<default>` partials in this element's chain that author a `fromto`. */
	TArray<UMjNodeComponent*> ClassLayers;
	/** True when pos/quat/size were authored and the element's own `fromto` cleared. */
	bool bFolded = false;
};

/**
 * Offer `Function` every layer of `Element` that authors a six-value `fromto`.
 *
 * Not just the winning one. MuJoCo merges the class chain into the element
 * before its own fold runs, so an element that folds has to account for the
 * `fromto` of every layer behind it too -- one left live beside the `pos` this
 * pass is about to author is the "both pos and fromto" the engine rejects.
 */
template <class P, class E, class Effective, class Fn>
void ForEachFromtoLayer(const E& Element, const Effective& Layers, int FieldId, Fn&& Function)
{
	Layers.ForEachLayer(Element, [&](const auto& Layer) {
		TArray<double> Candidate;
		if (ReadSeq<P>(Layer, FieldId, Candidate) && Candidate.Num() == 6)
		{
			Function(Layer, Candidate);
		}
		// Never stops: the question is which layers author one, not which wins.
		return false;
	});
}

/**
 * Fold one element's effective `fromto`, if it has one and the fold is exact.
 *
 * `Effective` resolves `type`, `size` and `fromto` itself the way the compiler's
 * class merge will, which is the whole reason this runs after the spec is
 * built rather than during the read. An element that authors no `fromto` but
 * inherits one from its class is as much a fold site as one that spells it out,
 * and previewing it from its own storage alone would draw a capsule of no
 * length.
 */
template <class P, class E, class Effective>
FFoldOutcome FoldElement(E& Element, const Effective& Layers)
{
	FFoldOutcome Outcome;
	const FFromtoFieldIds Ids = FieldIdsOf(gen::TMjElementType<E>::Value);
	if (!Ids.IsComplete())
	{
		return Outcome;
	}

	TArray<double> Fromto;
	bool bOwnFromto = false;
	ForEachFromtoLayer<P>(Element, Layers, Ids.Fromto, [&](const E& Layer, TArray<double>& Candidate) {
		// The nearest layer wins, which is the one the merge would keep.
		if (Fromto.Num() == 0)
		{
			Fromto = MoveTemp(Candidate);
		}
		if (&Layer == &Element)
		{
			bOwnFromto = true;
		}
		else
		{
			Outcome.ClassLayers.Add(const_cast<E*>(&Layer));
		}
	});
	if (Fromto.Num() != 6)
	{
		return Outcome;
	}

	// The type decides which size slots the half-length lands in, so an
	// unresolvable or inadmissible type is a fold this pass must not attempt.
	int32 TypeIndex = -1;
	Layers.ForEachLayer(Element, [&](const auto& Layer) { return ReadEnum<P>(Layer, Ids.Type, TypeIndex); });
	if (TypeIndex < 0 || !AdmitsFromto(static_cast<EMjGeomType>(TypeIndex)))
	{
		return Outcome;
	}

	// MuJoCo rejects a non-zero pos alongside fromto rather than resolving it
	// (user_objects.cc:3987). Leaving the attribute authored keeps that error
	// the engine's to report, with its own wording and its own source location.
	double Pos[3] = {0.0, 0.0, 0.0};
	bool bPosAuthored = false;
	Layers.ForEachLayer(Element, [&](const auto& Layer) { return (bPosAuthored = ReadFixed<P, 3>(Layer, Ids.Pos, Pos)); });
	if (bPosAuthored && (Pos[0] != 0.0 || Pos[1] != 0.0 || Pos[2] != 0.0))
	{
		return Outcome;
	}

	// mjCGeom::Compile's own order: the half-length is the norm over two, and the
	// direction stays raw for the resolver to normalize.
	const double Vec[3] = {Fromto[0] - Fromto[3], Fromto[1] - Fromto[4], Fromto[2] - Fromto[5]};
	const double Half = VectorNorm(Vec, 3) / 2.0;
	if (Half < MjEps)
	{
		return Outcome;
	}

	// The size the merge would see: the element, then its class chain, then the
	// schema defaults (a site's 0.005 triple; a geom has none and so reads 0).
	TArray<double> Size;
	bool bSizeAuthored = false;
	Layers.ForEachLayer(Element, [&](const auto& Layer) { return (bSizeAuthored = ReadSeq<P>(Layer, Ids.Size, Size)); });
	if (!bSizeAuthored)
	{
		ReadSeq<P>(P::template Defaults<E>(), Ids.Size, Size);
	}

	// mjsGeom::size is a three-slot array whose unauthored slots are zero, so
	// the authored prefix is padded rather than treated as short.
	double Slots[3] = {0.0, 0.0, 0.0};
	for (int32 Index = 0; Index < Size.Num() && Index < 3; ++Index)
	{
		Slots[Index] = Size[Index];
	}

	const EMjGeomType GeomType = static_cast<EMjGeomType>(TypeIndex);
	int32 SlotCount = 2;
	if (GeomType == EMjGeomType::ellipsoid || GeomType == EMjGeomType::box)
	{
		Slots[2] = Half;
		Slots[1] = Slots[0];
		SlotCount = 3;
	}
	else
	{
		Slots[1] = Half;
	}

	const double NewPos[3] = {(Fromto[0] + Fromto[3]) / 2.0, (Fromto[1] + Fromto[4]) / 2.0,
		(Fromto[2] + Fromto[5]) / 2.0};
	double NewQuat[4] = {1.0, 0.0, 0.0, 0.0};
	AxisToQuat(Vec, NewQuat);

	pssdk::internal::SetFixedField<P, 3>(Element, Ids.Pos, NewPos);
	pssdk::internal::SetFixedField<P, 4>(Element, Ids.Quat, NewQuat);
	pssdk::internal::SetSeqField<P>(Element, Ids.Size, Slots, static_cast<std::size_t>(SlotCount));
	if (bOwnFromto)
	{
		gen::Thunks(gen::TMjElementType<E>::Value).Clear(&Element, Ids.Fromto);
	}
	Outcome.bFolded = true;
	return Outcome;
}

/** Retire one `<default>` partial's `fromto`, now that every reader has folded. */
void ClearFromto(UMjNodeComponent& Node)
{
	gen::DispatchByType(Node, [&](auto& Element) {
		using E = std::decay_t<decltype(Element)>;
		const psm::ElementType Type = gen::TMjElementType<E>::Value;
		const int FieldId = pssdk::internal::FieldIdByName(Type, "fromto");
		if (FieldId >= 0)
		{
			gen::Thunks(Type).Clear(&Element, FieldId);
		}
	});
}

/** True when `Node` is a `<default>` class rather than a model element. */
bool IsDefaultClass(const UMjNodeComponent& Node)
{
	psm::ElementType Type;
	return gen::ElementTypeOfNode(Node, Type) && Type == psm::ElementType::Default;
}

/**
 * Fold every element under `Node`, collecting what the class layers are owed.
 *
 * `bInDefaults` marks the subtree of a `<default>`. A class partial is an
 * inheritance template, not an element: folding it would author a pose onto the
 * class itself, and every geom inheriting it would then carry that one pose. Its
 * `fromto` is retired by the caller instead, once every element that reads it
 * has folded.
 */
template <class P, class Adapter, class Effective>
void FoldSubtree(UMjNodeComponent& Node, const Effective& Layers, bool bInDefaults,
	TSet<UMjNodeComponent*>& Retire, TSet<UMjNodeComponent*>& Keep)
{
	const bool bDefaultClass = IsDefaultClass(Node);
	if (!bInDefaults && !bDefaultClass)
	{
		gen::DispatchByType(Node, [&](auto& Element) {
			const FFoldOutcome Outcome = FoldElement<P>(Element, Layers);
			for (UMjNodeComponent* Layer : Outcome.ClassLayers)
			{
				// A layer whose reader could not fold keeps its `fromto`, so the
				// engine raises its own diagnostic over that reader rather than
				// this pass quietly dropping geometry.
				(Outcome.bFolded ? Retire : Keep).Add(Layer);
			}
		});
	}

	for (const FMjOrderedChild& Child : Adapter::OrderedChildren(Node))
	{
		FoldSubtree<P, Adapter>(*Child.Node, Layers, bInDefaults || bDefaultClass, Retire, Keep);
	}
}

} // namespace

template <class Adapter>
void FoldFromtoTree(UMjNodeComponent& Root)
{
	// One context for the whole tree: it indexes the spec's classes and
	// parents, which is per-spec work, and the fold only ever changes field
	// values -- never the tree the context is built over.
	WithEffectiveDoc(Root, [&](auto& Layers) {
		using P = typename std::decay_t<decltype(Layers)>::ProfileType;

		TSet<UMjNodeComponent*> Retire;
		TSet<UMjNodeComponent*> Keep;
		FoldSubtree<P, Adapter>(Root, Layers, false, Retire, Keep);

		// Only after every reader has folded: clearing a class's `fromto` mid-walk
		// would hide it from the elements still to come.
		for (UMjNodeComponent* Layer : Retire)
		{
			if (!Keep.Contains(Layer))
			{
				ClearFromto(*Layer);
			}
		}
	});
}

template void FoldFromtoTree<FMjInstanceAdapter>(UMjNodeComponent&);
#if WITH_EDITOR
template void FoldFromtoTree<FMjScsAdapter>(UMjNodeComponent&);
#endif

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
