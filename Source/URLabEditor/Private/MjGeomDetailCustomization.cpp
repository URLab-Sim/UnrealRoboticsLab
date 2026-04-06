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
#include "MjGeomDetailCustomization.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "DetailLayoutBuilder.h"
#include "DetailCategoryBuilder.h"
#include "DetailWidgetRow.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"

TSharedRef<IDetailCustomization> FMjGeomDetailCustomization::MakeInstance()
{
    return MakeShareable(new FMjGeomDetailCustomization);
}

void FMjGeomDetailCustomization::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
    TArray<TWeakObjectPtr<UObject>> Objects;
    DetailBuilder.GetObjectsBeingCustomized(Objects);

    if (Objects.Num() != 1) return;
    TWeakObjectPtr<UMjGeom> WeakGeom = Cast<UMjGeom>(Objects[0].Get());
    if (!WeakGeom.IsValid()) return;

    if (WeakGeom->Type != EMjGeomType::Mesh) return;

    IDetailCategoryBuilder& Category = DetailBuilder.EditCategory("MuJoCo|Geom|Decomposition",
        FText::FromString("Mesh Decomposition"), ECategoryPriority::Important);

    Category.AddCustomRow(FText::FromString("Decompose"))
        .NameContent()
        [
            SNew(STextBlock).Text(FText::FromString("CoACD Decomposition"))
        ]
        .ValueContent()
        .MaxDesiredWidth(300.f)
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot()
            .AutoWidth()
            .Padding(2.f)
            [
                SNew(SButton)
                .Text(FText::FromString("Decompose Mesh"))
                .OnClicked_Lambda([WeakGeom]() -> FReply
                {
                    if (WeakGeom.IsValid()) WeakGeom->DecomposeMesh();
                    return FReply::Handled();
                })
            ]
            + SHorizontalBox::Slot()
            .AutoWidth()
            .Padding(2.f)
            [
                SNew(SButton)
                .Text(FText::FromString("Remove Decomposition"))
                .OnClicked_Lambda([WeakGeom]() -> FReply
                {
                    if (WeakGeom.IsValid()) WeakGeom->RemoveDecomposition();
                    return FReply::Handled();
                })
            ]
        ];
}
