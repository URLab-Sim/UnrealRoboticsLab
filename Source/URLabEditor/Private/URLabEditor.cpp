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
#include "URLabEditor.h"
#include "URLabEditorLogging.h"

DEFINE_LOG_CATEGORY(LogURLabEditor);
#include "PropertyEditorModule.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "MjGeomDetailCustomization.h"

#include "LevelEditor.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "MuJoCo/Components/QuickConvert/MjQuickConvertComponent.h"
#include "ScopedTransaction.h"

void FURLabEditorModule::StartupModule()
{
    FPropertyEditorModule& PropertyModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
    PropertyModule.RegisterCustomClassLayout(
        UMjGeom::StaticClass()->GetFName(),
        FOnGetDetailCustomizationInstance::CreateStatic(&FMjGeomDetailCustomization::MakeInstance));
    PropertyModule.NotifyCustomizationModuleChanged();

    // Register viewport actor context menu extender
    FLevelEditorModule& LevelEditorModule = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
    FLevelEditorModule::FLevelViewportMenuExtender_SelectedActors MenuExtender =
        FLevelEditorModule::FLevelViewportMenuExtender_SelectedActors::CreateStatic(
            &FURLabEditorModule::OnExtendActorContextMenu);
    auto& Extenders = LevelEditorModule.GetAllLevelViewportContextMenuExtenders();
    Extenders.Add(MenuExtender);
    ViewportContextMenuExtenderHandle = Extenders.Last().GetHandle();
}

void FURLabEditorModule::ShutdownModule()
{
    if (FModuleManager::Get().IsModuleLoaded("PropertyEditor"))
    {
        FPropertyEditorModule& PropertyModule = FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");
        PropertyModule.UnregisterCustomClassLayout(UMjGeom::StaticClass()->GetFName());
    }

    if (FModuleManager::Get().IsModuleLoaded("LevelEditor"))
    {
        FLevelEditorModule& LevelEditorModule = FModuleManager::GetModuleChecked<FLevelEditorModule>("LevelEditor");
        LevelEditorModule.GetAllLevelViewportContextMenuExtenders().RemoveAll(
            [this](const FLevelEditorModule::FLevelViewportMenuExtender_SelectedActors& Delegate) {
                return Delegate.GetHandle() == ViewportContextMenuExtenderHandle;
            });
    }
}

TSharedRef<FExtender> FURLabEditorModule::OnExtendActorContextMenu(
    const TSharedRef<FUICommandList> CommandList,
    const TArray<AActor*> SelectedActors)
{
    TSharedRef<FExtender> Extender = MakeShared<FExtender>();

    if (SelectedActors.Num() > 0)
    {
        Extender->AddMenuExtension(
            "ActorControl",
            EExtensionHook::After,
            CommandList,
            FMenuExtensionDelegate::CreateLambda([SelectedActors](FMenuBuilder& MenuBuilder)
            {
                MenuBuilder.AddSubMenu(
                    FText::FromString("MuJoCo Quick Convert"),
                    FText::FromString("Add MjQuickConvertComponent with preset configuration"),
                    FNewMenuDelegate::CreateStatic(&FURLabEditorModule::BuildQuickConvertSubMenu, SelectedActors));
            }));
    }

    return Extender;
}

void FURLabEditorModule::BuildQuickConvertSubMenu(FMenuBuilder& MenuBuilder, TArray<AActor*> SelectedActors)
{
    // Capture actors as weak pointers so they survive across frames
    TArray<TWeakObjectPtr<AActor>> WeakActors;
    for (AActor* Actor : SelectedActors)
    {
        WeakActors.Add(Actor);
    }

    MenuBuilder.AddMenuEntry(
        FText::FromString("Simple Static"),
        FText::FromString("Simple collision, fixed in place (no free joint)"),
        FSlateIcon(),
        FUIAction(FExecuteAction::CreateStatic(&FURLabEditorModule::ApplyQuickConvert, WeakActors, true, false)));

    MenuBuilder.AddMenuEntry(
        FText::FromString("Simple Dynamic"),
        FText::FromString("Simple collision, free to move under physics"),
        FSlateIcon(),
        FUIAction(FExecuteAction::CreateStatic(&FURLabEditorModule::ApplyQuickConvert, WeakActors, false, false)));

    MenuBuilder.AddMenuEntry(
        FText::FromString("Complex Static"),
        FText::FromString("CoACD decomposition, fixed in place (no free joint)"),
        FSlateIcon(),
        FUIAction(FExecuteAction::CreateStatic(&FURLabEditorModule::ApplyQuickConvert, WeakActors, true, true)));

    MenuBuilder.AddMenuEntry(
        FText::FromString("Complex Dynamic"),
        FText::FromString("CoACD decomposition, free to move under physics"),
        FSlateIcon(),
        FUIAction(FExecuteAction::CreateStatic(&FURLabEditorModule::ApplyQuickConvert, WeakActors, false, true)));
}

void FURLabEditorModule::ApplyQuickConvert(TArray<TWeakObjectPtr<AActor>> Actors, bool bStatic, bool bComplex)
{
    FScopedTransaction Transaction(FText::FromString("MuJoCo Quick Convert"));

    int32 Applied = 0;
    for (const TWeakObjectPtr<AActor>& WeakActor : Actors)
    {
        AActor* Actor = WeakActor.Get();
        if (!Actor) continue;

        // Skip if already has a QuickConvert component
        if (Actor->FindComponentByClass<UMjQuickConvertComponent>())
        {
            UE_LOG(LogURLabEditor, Warning, TEXT("Actor '%s' already has MjQuickConvertComponent, skipping."), *Actor->GetName());
            continue;
        }

        Actor->Modify();

        // Set mobility to Movable (required for MuJoCo transform sync)
        if (USceneComponent* Root = Actor->GetRootComponent())
        {
            Root->SetMobility(EComponentMobility::Movable);
        }

        // Create and configure the component
        UMjQuickConvertComponent* Comp = NewObject<UMjQuickConvertComponent>(Actor, NAME_None, RF_Transactional);
        Comp->Static = bStatic;
        Comp->ComplexMeshRequired = bComplex;

        Actor->AddInstanceComponent(Comp);
        Comp->RegisterComponent();

        Actor->GetPackage()->MarkPackageDirty();
        Applied++;
    }

    UE_LOG(LogURLabEditor, Log, TEXT("MuJoCo Quick Convert applied to %d actor(s) [Static=%d, Complex=%d]"),
        Applied, bStatic, bComplex);
}

IMPLEMENT_MODULE(FURLabEditorModule, URLabEditor)
