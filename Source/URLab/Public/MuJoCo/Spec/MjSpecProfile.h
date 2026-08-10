// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// The hand half of the Unreal emission profile.
//
// ProtoSpec's SDK, reader and writer run on a profile of six policies. Two of
// them -- Str and Shape -- follow from the storage decisions the generator makes
// and ship with it (ps::ue::FMjGeneratedProfile). The other four follow from
// Unreal instead, and are here:
//
//   Doc     what the spec root is and what its top-level sections are
//   Tree    how children are stored, ordered, inserted and removed
//   Ref     how a reference slot is proxied and rewritten
//   Ident   identity, provenance, construction, duplication
//
// Two profiles ship, not one, because Unreal has two object graphs that both
// hold a component tree: a live actor's attachment hierarchy, and a Blueprint's
// USCS_Node template graph. They differ in Tree and share everything else, which
// is exactly the shape the policy split was chosen for.
//
// Nothing here scales with the schema. It scales with the number of Unreal
// mechanisms, which is a fixed list that never grows again.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjRefView.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjTreeAdapters.h"

THIRD_PARTY_INCLUDES_START
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "protospec/core.h"
#include "protospec/detail.h"
#include "protospec/model_core.h"
#include "protospec/profile.h"
THIRD_PARTY_INCLUDES_END

namespace urlab::spec
{

// --- Ident ------------------------------------------------------------------ //
//
// Identity, provenance and construction all live on UMjNodeComponent, so this is
// one implementation for all 145 element classes rather than 145.

/**
 * The class registered for `Type`, or `Fallback` when nothing is registered.
 *
 * Declared in MjNodeFactories.h, which the node factories use to build hand
 * subclasses in place of generated ones. That header cannot be included here:
 * it builds FScsNodeFactory and FInstanceNodeFactory over FMjScsProfile and
 * FMjInstanceProfile, which this file only finishes defining below. Forward
 * declaring the one entry point Ident needs avoids the cycle.
 */
URLAB_API UClass* MjElementClass(psm::ElementType Type, UClass* Fallback);

struct FMjIdentPolicy
{
	using serial_t = std::uint64_t;

	template <class E>
	static serial_t Serial(const E& Element)
	{
		return Element.Serial;
	}

	template <class E>
	static void SetSerial(E& Element, serial_t S)
	{
		Element.Serial = S;
	}

	template <class E>
	static ps::SourceLoc Loc(const E& Element)
	{
		return ps::SourceLoc{gen::FMjStrPolicy::ToUtf8(FStringView(Element.SourceFile)),
			static_cast<int>(Element.SourceLine)};
	}

	template <class E>
	static void SetLoc(E& Element, ps::SourceLoc L)
	{
		Element.SourceFile = gen::FMjStrPolicy::FromUtf8(L.file);
		Element.SourceLine = static_cast<int32>(L.line);
	}

	/**
	 * Construct a detached element.
	 *
	 * The transient package is the outer only until the tree adapter links it,
	 * which the profile contract guarantees happens in the same call: Adopt takes
	 * an owner, never a detached node that outlives the statement. The node
	 * factories construct in the right place directly and never reach here.
	 */
	template <class E>
	static E* New()
	{
		UClass* ResolvedClass = MjElementClass(gen::TMjElementType<E>::Value, E::StaticClass());
		E* Element = NewObject<E>(GetTransientPackage(), ResolvedClass, NAME_None, RF_Transactional);
		Element->EnsureSerial();
		return Element;
	}

	/** Deep copy with a fresh identity, minted by UMjNodeComponent::PostDuplicate. */
	template <class E>
	static E* Clone(const E& Element)
	{
		return DuplicateObject<E>(&Element, Element.GetOuter());
	}
};

// --- Ref -------------------------------------------------------------------- //
//
// References reach every algorithm as move-only slots handed out BY VALUE, never
// as a pointer into storage. The scan recognises one by the RefView wrapper
// generated Visit puts around it, and builds its operations over the storage the
// view points at -- the view itself dies with the Visit frame.

struct FMjRefPolicy
{
	using slot = pssdk::RefSlot<FStringView>;

	/** The operation table for one slot storage shape. */
	template <class SlotType>
	static const typename slot::Ops* OpsFor()
	{
		using Storage = std::remove_const_t<SlotType>;
		static const typename slot::Ops Table{
			[](void* Object) -> bool {
				Storage& S = *static_cast<Storage*>(Object);
				if constexpr (TIsOptional<Storage>::value)
				{
					return S.IsSet() && !S.GetValue().IsEmpty();
				}
				else
				{
					return !S.IsEmpty();
				}
			},
			[](void* Object) -> FStringView {
				Storage& S = *static_cast<Storage*>(Object);
				if constexpr (TIsOptional<Storage>::value)
				{
					return S.IsSet() ? FStringView(S.GetValue()) : FStringView();
				}
				else
				{
					return FStringView(S);
				}
			},
			[](void* Object, FStringView Name) {
				Storage& S = *static_cast<Storage*>(Object);
				if constexpr (TIsOptional<Storage>::value)
				{
					if (Name.IsEmpty())
					{
						S.Reset();
					}
					else
					{
						S = FString(Name);
					}
				}
				else
				{
					S = FString(Name);
				}
			},
			[](void* Object) {
				Storage& S = *static_cast<Storage*>(Object);
				if constexpr (TIsOptional<Storage>::value)
				{
					S.Reset();
				}
				else
				{
					S.Empty();
				}
			},
		};
		return &Table;
	}

	template <class OnRef>
	struct FScan
	{
		OnRef* On;
		psm::ElementType Type;

		template <class U>
		void field(int Id, const char* Name, U& Value)
		{
			using View = std::decay_t<U>;
			if constexpr (urlab::ref_view_detail::TIsRefView<View>::value)
			{
				using SlotType = typename urlab::ref_view_detail::TIsRefView<View>::Slot;
				static_assert(!std::is_const_v<SlotType>,
					"the reference scan hands out live slots and needs a mutable element");
				if (Value.IsSet())
				{
					(*On)(Id, Name, slot{Value.SlotPtr(), OpsFor<SlotType>()},
						pssdk::detail::RefTargetsAt(Type, Id));
				}
			}
		}
		template <class C>
		void child(int, const char*, C&)
		{
		}
		template <class C>
		void union_child(int, const char*, C&)
		{
		}
	};

	struct FGrabRef
	{
		int Target;
		void* Object = nullptr;
		const typename slot::Ops* Ops = nullptr;

		template <class U>
		void field(int Id, const char*, U& Value)
		{
			if (Id != Target)
			{
				return;
			}
			using View = std::decay_t<U>;
			if constexpr (urlab::ref_view_detail::TIsRefView<View>::value)
			{
				using SlotType = typename urlab::ref_view_detail::TIsRefView<View>::Slot;
				if constexpr (!std::is_const_v<SlotType>)
				{
					Object = Value.SlotPtr();
					Ops = OpsFor<SlotType>();
				}
			}
		}
		template <class C>
		void child(int, const char*, C&)
		{
		}
		template <class C>
		void union_child(int, const char*, C&)
		{
		}
	};

	/** The plain string field a dynamic (target_from) reference names. */
	struct FGrabDyn
	{
		int Target;
		void* Object = nullptr;
		const typename slot::Ops* Ops = nullptr;

		template <class U>
		void field(int Id, const char*, U& Value)
		{
			if (Id != Target)
			{
				return;
			}
			using Slot = std::decay_t<U>;
			if constexpr ((std::is_same_v<Slot, TOptional<FString>> || std::is_same_v<Slot, FString>) &&
						  !std::is_const_v<U>)
			{
				Object = &Value;
				Ops = OpsFor<Slot>();
			}
		}
		template <class C>
		void child(int, const char*, C&)
		{
		}
		template <class C>
		void union_child(int, const char*, C&)
		{
		}
	};

	struct FConstName
	{
		int Target;
		FStringView Out;

		template <class U>
		void field(int Id, const char*, const U& Value)
		{
			if (Id != Target)
			{
				return;
			}
			if constexpr (urlab::ref_view_detail::TIsRefView<std::decay_t<U>>::value)
			{
				Out = Value.Get();
			}
		}
		template <class C>
		void child(int, const char*, const C&)
		{
		}
		template <class C>
		void union_child(int, const char*, const C&)
		{
		}
	};

	template <class Pred>
	struct FClearer
	{
		Pred* Predicate;
		psm::ElementType Type;

		template <class U>
		void field(int Id, const char*, U& Value)
		{
			using View = std::decay_t<U>;
			if constexpr (urlab::ref_view_detail::TIsRefView<View>::value)
			{
				using SlotType = typename urlab::ref_view_detail::TIsRefView<View>::Slot;
				if constexpr (!std::is_const_v<SlotType>)
				{
					if (Value.IsSet() && (*Predicate)(Value.Get(), pssdk::detail::RefTargetsAt(Type, Id)))
					{
						Value.Clear();
					}
				}
			}
		}
		template <class C>
		void child(int, const char*, C&)
		{
		}
		template <class C>
		void union_child(int, const char*, C&)
		{
		}
	};

	template <class E, class OnRef>
	static void ScanTyped(E& Element, OnRef&& On)
	{
		FScan<std::remove_reference_t<OnRef>> Visitor{&On, gen::TMjElementType<std::decay_t<E>>::Value};
		gen::Visit(Element, Visitor);
	}

	template <class E>
	static slot SlotAt(E& Element, int FieldId)
	{
		if (FieldId < 0)
		{
			return slot{};
		}
		FGrabRef Grab{FieldId};
		gen::Visit(Element, Grab);
		return Grab.Ops != nullptr ? slot{Grab.Object, Grab.Ops} : slot{};
	}

	template <class E>
	static FStringView NameAt(const E& Element, int FieldId)
	{
		if (FieldId < 0)
		{
			return FStringView();
		}
		FConstName Grab{FieldId, FStringView()};
		gen::Visit(Element, Grab);
		return Grab.Out;
	}

	template <class E>
	static slot DynSlot(E& Element, int FieldId)
	{
		if (FieldId < 0)
		{
			return slot{};
		}
		FGrabDyn Grab{FieldId};
		gen::Visit(Element, Grab);
		return Grab.Ops != nullptr ? slot{Grab.Object, Grab.Ops} : slot{};
	}

	template <class E, class Pred>
	static void ClearTypedIf(E& Element, Pred&& Predicate)
	{
		FClearer<std::remove_reference_t<Pred>> Visitor{&Predicate, gen::TMjElementType<std::decay_t<E>>::Value};
		gen::Visit(Element, Visitor);
	}

private:
	template <class T>
	struct TIsOptional : std::false_type
	{
	};
	template <class T>
	struct TIsOptional<TOptional<T>> : std::true_type
	{
	};
};

// --- Doc -------------------------------------------------------------------- //

/**
 * Visit adapter that grabs one authored `TOptional<FString>` field by id.
 *
 * At namespace scope rather than inside the one function that uses it, because
 * a local class may not declare member templates and `field` has to be one.
 */
struct FMjGrabOptionalString
{
	int Target;
	const FString* Out = nullptr;

	template <class U>
	void field(int Id, const char*, const U& Value)
	{
		if (Id != Target)
		{
			return;
		}
		if constexpr (std::is_same_v<std::decay_t<U>, TOptional<FString>>)
		{
			if (Value.IsSet())
			{
				Out = &Value.GetValue();
			}
		}
	}
	template <class C>
	void child(int, const char*, const C&)
	{
	}
	template <class C>
	void union_child(int, const char*, const C&)
	{
	}
};

/**
 * What the spec root is and what its top-level sections are.
 *
 * Parameterised on the tree adapter because reaching a root's sections is the
 * only thing about a root that differs between the two object graphs.
 */
template <class TreeAdapter>
struct TMjDocPolicy
{
	using doc_type = typename gen::TMjElementOf<psm::ElementType::Model>::Type;
	using root_type = doc_type;
	using node_ptr = void*;

	/**
	 * The root is the spec, not content: it carries no name, cannot be
	 * selected, renamed, deleted or referenced, and every content walk skips it.
	 */
	template <class E>
	static constexpr bool is_root = std::is_same_v<std::remove_const_t<E>, doc_type>;

	/**
	 * Every top-level section except the <default> tree.
	 *
	 * Class-defining elements live only under <default> and are authoring
	 * templates rather than model content, so operations that act on real
	 * elements use this and reach the class tree explicitly when they want it.
	 */
	template <class D, class Fn>
	static void ForEachLiveSection(D& Doc, Fn&& Function)
	{
		using DefaultType = typename gen::TMjElementOf<psm::ElementType::Default>::Type;
		TreeAdapter::ForEachChild(Doc, [&](auto& Section) {
			if constexpr (!std::is_same_v<std::decay_t<decltype(Section)>, DefaultType>)
			{
				Function(Section);
			}
		});
	}

	/** The `model` attribute of <mujoco>, or unset. */
	static std::optional<FStringView> Name(const doc_type& Model)
	{
		FMjGrabOptionalString Grab{pssdk::internal::FieldIdByName(psm::ElementType::Model, "model")};
		gen::Visit(Model, Grab);
		if (Grab.Out == nullptr)
		{
			return std::nullopt;
		}
		return FStringView(*Grab.Out);
	}
};

/** The generated half plus the four Unreal ones, minus the graph. */
template <class TreeAdapter>
struct TMjProfile : gen::FMjGeneratedProfile
{
	using Doc = TMjDocPolicy<TreeAdapter>;
	using Tree = TreeAdapter;
	using Ref = FMjRefPolicy;
	using Ident = FMjIdentPolicy;
};

/** The spec as a spawned actor's component hierarchy. */
struct FMjInstanceProfile : TMjProfile<FMjInstanceAdapter>
{
};

#if WITH_EDITOR
/** The spec as a Blueprint's construction-script template graph. */
struct FMjScsProfile : TMjProfile<FMjScsAdapter>
{
};
#endif

/**
 * The read helper the pose preview needs: an authored fixed-arity field as N
 * scalars in MJCF component order. False when the field is unauthored or is not
 * fixed-arity of that width.
 */
template <class P, std::size_t N, class E>
bool ReadFixed(const E& Element, int FieldId, double* Out)
{
	bool bRead = false;
	pssdk::internal::ReadField<P>(Element, FieldId, [&](const auto& Slot) {
		using S = typename P::Shape;
		using I = typename S::template inner_t<std::decay_t<decltype(Slot)>>;
		if constexpr (S::template kind_v<I> == pssdk::Shape::Fixed)
		{
			using F = typename S::template fixed<I>;
			if constexpr (F::size == N)
			{
				if (S::IsSet(Slot))
				{
					typename F::scalar Buffer[N]{};
					F::Load(S::Read(Slot), Buffer);
					for (std::size_t Index = 0; Index < N; ++Index)
					{
						Out[Index] = static_cast<double>(Buffer[Index]);
					}
					bRead = true;
				}
				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	});
	return bRead;
}

/**
 * As ReadFixed, for a range- or unbounded-arity numeric field. False when the
 * field is unauthored or is not a numeric sequence.
 */
template <class P, class E>
bool ReadSeq(const E& Element, int FieldId, TArray<double>& Out)
{
	bool bRead = false;
	pssdk::internal::ReadField<P>(Element, FieldId, [&](const auto& Slot) {
		using S = typename P::Shape;
		using I = typename S::template inner_t<std::decay_t<decltype(Slot)>>;
		constexpr pssdk::Shape Kind = S::template kind_v<I>;
		if constexpr (Kind == pssdk::Shape::Range || Kind == pssdk::Shape::Unbounded)
		{
			using Q = typename S::template seq<I>;
			using Sc = typename Q::scalar;
			if constexpr (std::is_arithmetic_v<Sc>)
			{
				if (S::IsSet(Slot))
				{
					const I& Value = S::Read(Slot);
					const std::size_t Count = Q::Size(Value);
					TArray<Sc> Buffer;
					Buffer.SetNumZeroed(static_cast<int32>(Count));
					if (Count > 0)
					{
						Q::Load(Value, Buffer.GetData());
					}
					Out.Reset();
					for (Sc Scalar : Buffer)
					{
						Out.Add(static_cast<double>(Scalar));
					}
					bRead = true;
				}
				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	});
	return bRead;
}

/**
 * As ReadFixed, for an enum field: the enumerator's schema declaration index.
 * False when the field is unauthored or is not an enum.
 */
template <class P, class E>
bool ReadEnum(const E& Element, int FieldId, int32& Out)
{
	bool bRead = false;
	pssdk::internal::ReadField<P>(Element, FieldId, [&](const auto& Slot) {
		using S = typename P::Shape;
		using I = typename S::template inner_t<std::decay_t<decltype(Slot)>>;
		if constexpr (S::template kind_v<I> == pssdk::Shape::Enum)
		{
			if (S::IsSet(Slot))
			{
				Out = static_cast<int32>(S::EnumIndex(S::Read(Slot)));
				bRead = true;
			}
			return true;
		}
		else
		{
			return false;
		}
	});
	return bRead;
}

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
