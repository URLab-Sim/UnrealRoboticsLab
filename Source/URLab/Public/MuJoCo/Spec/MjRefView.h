// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"
#include "Containers/StringView.h"

#include <type_traits>

namespace urlab
{

/**
 * A typed, aliasing proxy over one stored MJCF reference.
 *
 * Reference storage is a name string, because that is what MJCF is -- and that
 * makes it indistinguishable from a plain `string` attribute, which is a problem
 * rather than a detail: everything that scans references (referrer-safe rename,
 * the dangling-reference report, attach-time prefixing) would silently miss the
 * ones it could not recognise. Generated Visit therefore hands every reference
 * over wrapped in one of these, and the wrapper is what the reference scan
 * matches on at compile time.
 *
 * `Target` is a phantom parameter, as in the plain profile's `Ref<T>`, so the
 * target type survives into the SDK's typed reference API.
 *
 * `SlotType` is spelled explicitly at every generated call site, because a
 * required reference stores a plain FString while an optional one stores
 * TOptional<FString>, and a const traversal binds either as const. The mutating
 * operations are constrained out on a const slot rather than failing inside.
 *
 * The view is a temporary that dies with the Visit frame. A Ref policy handing
 * out a RefSlot must build its operations over `SlotPtr()`, never over the view.
 */
template <class Target, class SlotType = TOptional<FString>>
struct RefView
{
	using TargetType = Target;
	using SlotStorage = std::remove_const_t<SlotType>;

	/** Aggregate-initialized by generated Visit: `RefView<T, S>{ Element.Member }`. */
	SlotType& Slot;

	/** Authored AND non-empty. An authored empty name is not a reference. */
	bool IsSet() const
	{
		if constexpr (bOptional)
		{
			return Slot.IsSet() && !Slot.GetValue().IsEmpty();
		}
		else
		{
			return !Slot.IsEmpty();
		}
	}

	/** The referenced name, or empty when unset. */
	FStringView Get() const
	{
		if constexpr (bOptional)
		{
			return Slot.IsSet() ? FStringView(Slot.GetValue()) : FStringView();
		}
		else
		{
			return FStringView(Slot);
		}
	}

	/** Point at `Name`. An empty name clears the slot. */
	template <class S = SlotType, std::enable_if_t<!std::is_const_v<S>, int> = 0>
	void Set(FStringView Name) const
	{
		if (Name.IsEmpty())
		{
			Clear();
			return;
		}
		Slot = FString(Name);
	}

	/**
	 * Unauthor the reference.
	 *
	 * A required reference has no unauthored state, so clearing empties the name
	 * and the validator reports the missing reference it has become.
	 */
	template <class S = SlotType, std::enable_if_t<!std::is_const_v<S>, int> = 0>
	void Clear() const
	{
		if constexpr (bOptional)
		{
			Slot.Reset();
		}
		else
		{
			Slot.Empty();
		}
	}

	/** The aliased storage, which outlives this view. */
	SlotType* SlotPtr() const { return &Slot; }

private:
	template <class T>
	struct TIsOptionalSlot : std::false_type
	{
	};
	template <class T>
	struct TIsOptionalSlot<TOptional<T>> : std::true_type
	{
	};

	static constexpr bool bOptional = TIsOptionalSlot<SlotStorage>::value;
};

namespace ref_view_detail
{
template <class T>
struct TIsRefView : std::false_type
{
};
template <class Target, class SlotType>
struct TIsRefView<RefView<Target, SlotType>> : std::true_type
{
	using Slot = SlotType;
};
}  // namespace ref_view_detail

}  // namespace urlab
