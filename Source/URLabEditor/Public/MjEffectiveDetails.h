// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// What an unset attribute is actually worth.
//
// A spec attribute is a TOptional because "unset" is a real state the writer
// depends on: an element emits what it authored and nothing else. In the panel
// that honesty reads as a blank -- a geom that inherits its size and its colour
// from `<default class="visual">` shows two Set buttons and no numbers, and the
// user has to open the class to find out what the compiler will use.
//
// This puts the inherited value back on the row, greyed, with the class it came
// from, plus one button that authors it onto the element. Reading is effective;
// writing stays authored-only, which is the asymmetry the whole preview layer
// rests on: showing an inherited value must never quietly author it.
//
// One customization, registered against UMjNodeComponent. The details panel
// runs every registered customization it finds up the class chain, so this
// coexists with the geom's own rather than replacing it.

#include "CoreMinimal.h"

#include "IDetailCustomization.h"

class FOptionalProperty;
class UMjNodeComponent;

/** Which layer supplied an attribute the element itself leaves unset. */
enum class EMjValueSource : uint8
{
	/** None did, not even the schema: there is nothing to put on the row. */
	None,

	/** A `<default>` class in the document, which the user can go and edit. */
	Class,

	/** MuJoCo's own value for the attribute. No document mentions it. */
	Schema,
};

/** One attribute's effective value, and where it came from. */
struct FMjEffectiveValue
{
	/** The value as the property system formats it. */
	FString Text;

	/** The `<default>` class, when `Source` is `Class`. "main" for the root one. */
	FString ClassName;

	EMjValueSource Source = EMjValueSource::None;

	bool IsSet() const { return Source != EMjValueSource::None; }
};

/** The inherited-value rows, for every MuJoCo element. */
class FMjEffectiveDetails : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();

	// IDetailCustomization
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;

	/**
	 * What the compiler will use for `Optional` when `Node` authors nothing.
	 *
	 * The nearest layer above the element that supplies the attribute, resolved
	 * through the layering `ps::sdk::EffectiveField` applies: the `<default>`
	 * class chain from nearest to furthest, and then MuJoCo's own value for the
	 * attribute. False only when NO layer supplies it -- an attribute the schema
	 * itself leaves open, like a geom's `size`, where the honest row is a blank.
	 *
	 * The row builder's own question, exposed because it is the answer worth
	 * asserting: a display string is checkable where a Slate widget is not.
	 * Join an `FMjEffectiveScope` around a batch of calls -- see the
	 * customization -- or each one indexes the spec for itself.
	 */
	static bool ResolveInherited(UMjNodeComponent& Node, const FOptionalProperty& Optional, FMjEffectiveValue& Out);

	/** How the row spells `Value` beside the widget, source and all. */
	static FText DescribeValue(const FMjEffectiveValue& Value);

	static void RegisterAll();
	static void UnregisterAll();
};
