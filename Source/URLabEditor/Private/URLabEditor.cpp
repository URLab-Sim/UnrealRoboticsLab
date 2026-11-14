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
#include "MjEditorStyle.h"
#include "MjBridgeServerSubsystem.h"
#include "MjEditorOpHandlers.h"
#include "Bridge/BridgeServerProvider.h"
#include "SMjBridgeServerToggle.h"
#include "Editor.h"
#include "ToolMenus.h"
#include "ToolMenuContext.h"
#include "ToolMenuEntry.h"
#include "ToolMenuSection.h"
#include "MessageLogModule.h"
#include "PropertyEditorModule.h"
#include "MjFrameTypeCustomizations.h"
#include "MuJoCoSpec/MJSpecDetails.h"
#include "MuJoCoSpec/MJSpecVisualizers.h"
#include "UnrealEdGlobals.h"
#include "Editor/UnrealEdEngine.h"
#include "Misc/CoreDelegates.h"
#include "LevelEditor.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "MuJoCo/Convert/MjQuickConvertComponent.h"
#include "ScopedTransaction.h"

DEFINE_LOG_CATEGORY(LogURLabEditor);

void FURLabEditorModule::StartupModule()
{
	FMjEditorStyle::Initialize();

	{
		FMessageLogModule& MessageLogModule = FModuleManager::LoadModuleChecked<FMessageLogModule>(TEXT("MessageLog"));
		FMessageLogInitializationOptions InitOptions;
		InitOptions.bShowPages = true;
		MessageLogModule.RegisterLogListing(TEXT("URLab"), NSLOCTEXT("URLab", "URLabLogLabel", "URLab"), InitOptions);
	}

	URLabBridgeProvider::RegisterResolver([]() -> UURLabBridgeServer* {
		if (!GEditor)
			return nullptr;
		UURLabBridgeServerSubsystem* Sub =
			GEditor->GetEditorSubsystem<UURLabBridgeServerSubsystem>();
		if (!Sub)
			return nullptr;
		Sub->StartServer();
		return Sub->GetBridgeServer();
	});

	URLabEditorOpHandlers::RegisterAll();
	FMjFrameTypeCustomization::RegisterAll();
	MJSpecDetails::RegisterAll();
	if (GUnrealEd)
	{
		MJSpecVisualizers::RegisterAll();
	}
	else
	{
		FCoreDelegates::OnPostEngineInit.AddStatic(&MJSpecVisualizers::RegisterAll);
	}

	FPropertyEditorModule& PropertyModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
	PropertyModule.NotifyCustomizationModuleChanged();

	FLevelEditorModule& LevelEditorModule = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
	FLevelEditorModule::FLevelViewportMenuExtender_SelectedActors MenuExtender =
		FLevelEditorModule::FLevelViewportMenuExtender_SelectedActors::CreateStatic(
			&FURLabEditorModule::OnExtendActorContextMenu);
	auto& Extenders = LevelEditorModule.GetAllLevelViewportContextMenuExtenders();
	Extenders.Add(MenuExtender);
	ViewportContextMenuExtenderHandle = Extenders.Last().GetHandle();

	UToolMenus::RegisterStartupCallback(FSimpleMulticastDelegate::FDelegate::CreateLambda([]() {
		UToolMenu* ToolBar = UToolMenus::Get()->ExtendMenu("LevelEditor.LevelEditorToolBar.PlayToolBar");
		if (!ToolBar)
			return;
		FToolMenuSection& Section = ToolBar->FindOrAddSection("URLab");
		Section.AddEntry(FToolMenuEntry::InitWidget(
			"URLabBridgeServerToggle",
			SNew(SURLabBridgeServerToggle),
			FText::GetEmpty(),
			/*bNoIndent=*/true,
			/*bSearchable=*/false));
	}));
}

void FURLabEditorModule::ShutdownModule()
{
	FMjEditorStyle::Shutdown();

	if (FModuleManager::Get().IsModuleLoaded("MessageLog"))
	{
		FMessageLogModule& MessageLogModule = FModuleManager::GetModuleChecked<FMessageLogModule>(TEXT("MessageLog"));
		MessageLogModule.UnregisterLogListing(TEXT("URLab"));
	}

	URLabBridgeProvider::RegisterResolver(nullptr);
	URLabEditorOpHandlers::UnregisterAll();

	MJSpecVisualizers::UnregisterAll();

	if (FModuleManager::Get().IsModuleLoaded("PropertyEditor"))
	{
		FMjFrameTypeCustomization::UnregisterAll();
		MJSpecDetails::UnregisterAll();
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
			FMenuExtensionDelegate::CreateLambda([SelectedActors](FMenuBuilder& MenuBuilder) {
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
	TArray<TWeakObjectPtr<AActor>> WeakActors;
	for (AActor* Actor : SelectedActors)
	{
		WeakActors.Add(Actor);
	}

	MenuBuilder.AddMenuEntry(
		FText::FromString("Static (Simple Collision)"),
		FText::FromString("Add MjQuickConvertComponent configured as static with simple collision"),
		FSlateIcon(),
		FUIAction(FExecuteAction::CreateStatic(&FURLabEditorModule::ApplyQuickConvert, WeakActors, true, false)));

	MenuBuilder.AddMenuEntry(
		FText::FromString("Static (Complex / CoACD)"),
		FText::FromString("Add MjQuickConvertComponent configured as static with CoACD decomposition"),
		FSlateIcon(),
		FUIAction(FExecuteAction::CreateStatic(&FURLabEditorModule::ApplyQuickConvert, WeakActors, true, true)));

	MenuBuilder.AddMenuEntry(
		FText::FromString("Dynamic (Simple Collision)"),
		FText::FromString("Add MjQuickConvertComponent configured as dynamic with simple collision"),
		FSlateIcon(),
		FUIAction(FExecuteAction::CreateStatic(&FURLabEditorModule::ApplyQuickConvert, WeakActors, false, false)));

	MenuBuilder.AddMenuEntry(
		FText::FromString("Dynamic (Complex / CoACD)"),
		FText::FromString("Add MjQuickConvertComponent configured as dynamic with CoACD decomposition"),
		FSlateIcon(),
		FUIAction(FExecuteAction::CreateStatic(&FURLabEditorModule::ApplyQuickConvert, WeakActors, false, true)));
}

void FURLabEditorModule::ApplyQuickConvert(TArray<TWeakObjectPtr<AActor>> Actors, bool bStatic, bool bComplex)
{
	const FScopedTransaction Transaction(FText::FromString("Add MuJoCo Quick Convert"));
	for (const TWeakObjectPtr<AActor>& WeakActor : Actors)
	{
		AActor* Actor = WeakActor.Get();
		if (!Actor) continue;
		Actor->Modify();
		UMjQuickConvertComponent* Comp = NewObject<UMjQuickConvertComponent>(Actor);
		Comp->Static = bStatic;
		Comp->ComplexMeshRequired = bComplex;
		Comp->RegisterComponent();
		Actor->AddInstanceComponent(Comp);
	}
}

IMPLEMENT_MODULE(FURLabEditorModule, URLabEditor)
