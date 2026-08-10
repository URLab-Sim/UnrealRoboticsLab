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
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "MjComponentDetailCustomizations.h"

#include "DetailLayoutBuilder.h"
#include "DetailCategoryBuilder.h"
#include "DetailWidgetRow.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"

#include "MuJoCo/Elements/MjGeom.h"

// ============================================================================
// Geom — adds CoACD decomposition buttons (the only non-hiding logic).
// DefaultClass hiding now lives on the UPROPERTY itself.
// ============================================================================

TSharedRef<IDetailCustomization> FMjGeomDetailCustomization::MakeInstance()
{
	return MakeShareable(new FMjGeomDetailCustomization);
}

void FMjGeomDetailCustomization::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{

	// Deliberately NOT calling the inherited-value pass here. The editor runs
	// every layout registered along the class chain, so the one registered
	// against `UMjNodeComponent` already ran for this geom before this did.
	// Calling it again customised the same rows twice, and the second pass reset
	// each row before rebuilding it, which is what discarded the first pass's
	// widgets.

	TArray<TWeakObjectPtr<UObject>> Objects;
	DetailBuilder.GetObjectsBeingCustomized(Objects);

	if (Objects.Num() != 1)
		return;
	TWeakObjectPtr<UMjGeom> WeakGeom = Cast<UMjGeom>(Objects[0].Get());
	if (!WeakGeom.IsValid())
		return;

	// Decomposition buttons (only for mesh geoms).
	if (WeakGeom->GetType() != EMjGeomType::mesh)
		return;

	// A sibling of `MuJoCo|Geom`, never a child of it. Unreal reads `A|B|C` as a
	// nested path, so a category under the generated attributes' own category
	// creates that parent implicitly and early, while the attributes' real
	// category is created late by the first row that lands in it. The panel then
	// draws two Geom sections: the early one holding uncustomised rows and the
	// late one holding the inherited-value rows nobody could find.
	IDetailCategoryBuilder& DecompCategory = DetailBuilder.EditCategory("MuJoCo|Decomposition");

	DecompCategory.AddCustomRow(FText::FromString("Decompose"))
		.NameContent()
			[SNew(STextBlock).Text(FText::FromString("CoACD Decomposition"))]
		.ValueContent()
		.MaxDesiredWidth(300.f)
			[SNew(SHorizontalBox)
				+ SHorizontalBox::Slot()
					.AutoWidth()
					.Padding(2.f)
						[SNew(SButton)
								.Text(FText::FromString("Decompose Mesh"))
								.OnClicked_Lambda([WeakGeom]() -> FReply {
									if (WeakGeom.IsValid())
										WeakGeom->DecomposeMesh();
									return FReply::Handled();
								})]
				+ SHorizontalBox::Slot()
					.AutoWidth()
					.Padding(2.f)
						[SNew(SButton)
								.Text(FText::FromString("Remove Decomposition"))
								.OnClicked_Lambda([WeakGeom]() -> FReply {
									if (WeakGeom.IsValid())
										WeakGeom->RemoveDecomposition();
									return FReply::Handled();
								})]];
}
