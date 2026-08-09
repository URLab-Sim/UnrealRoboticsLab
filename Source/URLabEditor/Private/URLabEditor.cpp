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
#include "SMjStepModeIndicator.h"
#include "SMjBridgeServerToggle.h"
#include "Editor.h"
#include "ToolMenus.h"
#include "ToolMenuContext.h"
#include "ToolMenuEntry.h"
#include "ToolMenuSection.h"
#include "ToolMenuMisc.h"

DEFINE_LOG_CATEGORY(LogURLabEditor);
#include "PropertyEditorModule.h"

#include "MjEffectiveDetails.h"
#include "MjElementVisualizers.h"
#include "MjFrameTypeCustomizations.h"
#include "MuJoCo/Spec/MjElementIdentity.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Elements/MjSensorRuntime.h"
#include "MjComponentDetailCustomizations.h"
#include "MuJoCo/Gen/Elements/Constraints/MjPair.gen.h"
#include "MuJoCo/Gen/Elements/Constraints/MjExclude.gen.h"
#include "MuJoCo/Gen/Elements/Constraints/MjEquality.gen.h"

#include "LevelEditor.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "MuJoCo/Convert/MjQuickConvertComponent.h"
#include "ScopedTransaction.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/SCS_Node.h"
#include "MuJoCo/Elements/MjSensorRuntime.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Elements/MjJointRuntime.h"
#include "MuJoCo/Gen/Elements/Geometry/MjSite.gen.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Gen/Elements/Defaults/MjDefault.gen.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "SMjArticulationOutliner.h"
#include "Kismet2/BlueprintEditorUtils.h"
#include "Framework/Docking/WorkspaceItem.h"

void FURLabEditorModule::StartupModule()
{
	FMjEditorStyle::Initialize();

	// Install the bridge-server resolver so AAMjManager (URLab module) can
	// discover the editor-time server without depending on URLabEditor.
	// The resolver lazy-starts the subsystem's server so a PIE BeginPlay
	// that fires before the user toggles AutoStart still gets a server.
	URLabBridgeProvider::RegisterResolver([]() -> UURLabBridgeServer* {
		if (!GEditor)
			return nullptr;
		UURLabBridgeServerSubsystem* Sub =
			GEditor->GetEditorSubsystem<UURLabBridgeServerSubsystem>();
		if (!Sub)
			return nullptr;
		Sub->StartServer(); // idempotent; honours INI auto-start
		return Sub->GetBridgeServer();
	});

	// Editor-only op handlers (import_xml, create_level, spawn_*, ...).
	// The dispatcher in URLab forwards editor-only ops through
	// URLabOpRegistry, which these handlers populate.
	URLabEditorOpHandlers::RegisterAll();

	// The MuJoCo frame types render as a single inline row with a unit label,
	// rather than as an expandable struct of three doubles.
	FMjFrameTypeCustomization::RegisterAll();

	// Joints, sites, lights and cameras have no mesh to preview with, so the
	// editor drew nothing for them at all. One visualizer against the element
	// base draws all of them from the components and their effective values.
	FMjElementVisualizer::RegisterAll();

	// An unset attribute is not a blank: it is whatever the element's default
	// class says. The panel shows that value, greyed, next to the class name.
	FMjEffectiveDetails::RegisterAll();

	// A component dropped somewhere the schema does not admit it used to fail
	// at compile, minutes later. It says so at the drop now.
	FMjAddTimeLegality::RegisterAll();

	FPropertyEditorModule& PropertyModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
	// Only the geom needs a custom layout, for the CoACD decomposition buttons.
	// Every other element's panel is what the generated UPROPERTY metadata makes
	// it, which is the point of generating them.
	{
		TArray<UClass*> GeomClasses;
		GetDerivedClasses(UMjGeom::StaticClass(), GeomClasses, true);
		GeomClasses.Add(UMjGeom::StaticClass());
		for (UClass* Class : GeomClasses)
		{
			PropertyModule.RegisterCustomClassLayout(
				Class->GetFName(),
				FOnGetDetailCustomizationInstance::CreateStatic(&FMjGeomDetailCustomization::MakeInstance));
		}
	}
	PropertyModule.NotifyCustomizationModuleChanged();

	// Register viewport actor context menu extender
	FLevelEditorModule& LevelEditorModule = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
	FLevelEditorModule::FLevelViewportMenuExtender_SelectedActors MenuExtender =
		FLevelEditorModule::FLevelViewportMenuExtender_SelectedActors::CreateStatic(
			&FURLabEditorModule::OnExtendActorContextMenu);
	auto& Extenders = LevelEditorModule.GetAllLevelViewportContextMenuExtenders();
	Extenders.Add(MenuExtender);
	ViewportContextMenuExtenderHandle = Extenders.Last().GetHandle();

	// Register auto-parenting hook for MuJoCo components
	OnObjectModifiedHandle = FCoreUObjectDelegates::OnObjectModified.AddRaw(this, &FURLabEditorModule::OnObjectModified);

	// Register the StepMode status indicator into the level editor toolbar.
	// The indicator is a small Slate widget that polls AAMjManager::Instance
	// every 0.5s and shows a coloured pill: green=Live, amber=Direct,
	// blue=Puppet, grey=Auto/none. No asset deps; pure code.
	UToolMenus::RegisterStartupCallback(FSimpleMulticastDelegate::FDelegate::CreateLambda([]() {
		UToolMenu* ToolBar = UToolMenus::Get()->ExtendMenu("LevelEditor.LevelEditorToolBar.PlayToolBar");
		if (!ToolBar)
			return;
		FToolMenuSection& Section = ToolBar->FindOrAddSection("URLab");
		Section.AddEntry(FToolMenuEntry::InitWidget(
			"URLabStepModeIndicator",
			SNew(SMjStepModeIndicator),
			FText::GetEmpty(),
			/*bNoIndent=*/true,
			/*bSearchable=*/false));
		Section.AddEntry(FToolMenuEntry::InitWidget(
			"URLabBridgeServerToggle",
			SNew(SURLabBridgeServerToggle),
			FText::GetEmpty(),
			/*bNoIndent=*/true,
			/*bSearchable=*/false));
	}));

	// Register MuJoCo Outliner tab (guard against double registration on hot-reload)
	FGlobalTabmanager::Get()->UnregisterNomadTabSpawner(TEXT("MjArticulationOutliner"));
	FGlobalTabmanager::Get()->RegisterNomadTabSpawner(
								TEXT("MjArticulationOutliner"),
								FOnSpawnTab::CreateLambda([](const FSpawnTabArgs& Args) -> TSharedRef<SDockTab> {
									TSharedRef<SMjArticulationOutliner> Outliner = SNew(SMjArticulationOutliner);

									return SNew(SDockTab)
										.TabRole(ETabRole::NomadTab)
										.Label(FText::FromString(TEXT("MuJoCo Outliner")))
											[Outliner];
								}))
		.SetDisplayName(FText::FromString(TEXT("MuJoCo Outliner")))
		.SetTooltipText(FText::FromString(TEXT("Filtered view of MuJoCo articulation components")));
}

void FURLabEditorModule::ShutdownModule()
{
	FMjEditorStyle::Shutdown();

	FMjElementVisualizer::UnregisterAll();
	FMjAddTimeLegality::UnregisterAll();

	URLabBridgeProvider::RegisterResolver(nullptr);
	URLabEditorOpHandlers::UnregisterAll();

	FCoreUObjectDelegates::OnObjectModified.Remove(OnObjectModifiedHandle);

	FGlobalTabmanager::Get()->UnregisterNomadTabSpawner(TEXT("MjArticulationOutliner"));

	if (FModuleManager::Get().IsModuleLoaded("PropertyEditor"))
	{
		FPropertyEditorModule& PropertyModule = FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");
		// Unregister all subclass customizations
		auto UnregisterWithSubclasses = [&](UClass* BaseClass) {
			TArray<UClass*> Classes;
			GetDerivedClasses(BaseClass, Classes, true);
			Classes.Add(BaseClass);
			for (UClass* Class : Classes)
			{
				PropertyModule.UnregisterCustomClassLayout(Class->GetFName());
			}
		};
		UnregisterWithSubclasses(UMjGeom::StaticClass());
		FMjEffectiveDetails::UnregisterAll();
		FMjFrameTypeCustomization::UnregisterAll();
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
		if (!Actor)
			continue;

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

void FURLabEditorModule::OnObjectModified(UObject* Object)
{
	if (bIsAutoParenting)
		return;

	USimpleConstructionScript* SCS = Cast<USimpleConstructionScript>(Object);
	if (!SCS)
		return;

	UBlueprint* BP = SCS->GetBlueprint();
	if (!BP || !BP->GeneratedClass || !BP->GeneratedClass->IsChildOf(AMjArticulation::StaticClass()))
		return;

	// Defer to next tick so the SCS tree is fully updated before we scan
	TWeakObjectPtr<USimpleConstructionScript> WeakSCS = SCS;
	TWeakObjectPtr<UBlueprint> WeakBP = BP;
	GEditor->GetTimerManager()->SetTimerForNextTick([this, WeakSCS, WeakBP]() {
		if (bIsAutoParenting)
			return;
		USimpleConstructionScript* SCSPtr = WeakSCS.Get();
		UBlueprint* BPPtr = WeakBP.Get();
		if (!SCSPtr || !BPPtr)
			return;

		TArray<USCS_Node*> AllNodes = SCSPtr->GetAllNodes();
		bIsAutoParenting = true;
		bool bMoved = false;

		for (USCS_Node* Node : AllNodes)
		{
			bMoved |= AutoParentSCSNode(Node, SCSPtr);
		}
		bIsAutoParenting = false;

		if (bMoved)
		{
			FBlueprintEditorUtils::MarkBlueprintAsStructurallyModified(BPPtr);
		}
	});
}

bool FURLabEditorModule::AutoParentSCSNode(USCS_Node* Node, USimpleConstructionScript* SCS)
{
	if (!Node || !Node->ComponentTemplate)
		return false;

	// Determine the correct parent folder name for this component type
	// Which organisational folder a freshly added component belongs under is a
	// question about the element it is, not about its C++ class: an actuator is
	// thirteen classes with no shared base, and a sensor is forty-eight.
	FString TargetParentName;
#if URLAB_MJ_GEN
	using urlab::spec::psm::ElementType;
	const UMjNodeComponent* Element = Cast<UMjNodeComponent>(Node->ComponentTemplate);
	ElementType Type;
	if (Element == nullptr || !urlab::spec::MjElementTypeOfNode(*Element, Type))
		return false;

	if (UMjSensorRuntime::IsSensor(Element))
		TargetParentName = TEXT("SensorsRoot");
	else if (UMjActuatorRuntime::IsActuator(Element))
		TargetParentName = TEXT("ActuatorsRoot");
	else if (Type == ElementType::Default)
		TargetParentName = TEXT("DefaultsRoot");
	else if (Type == ElementType::Spatial || Type == ElementType::Fixed)
		TargetParentName = TEXT("TendonsRoot");
	else if (Type == ElementType::Pair || Type == ElementType::Exclude)
		TargetParentName = TEXT("ContactsRoot");
	else if (Type == ElementType::Connect || Type == ElementType::Weld
		|| Type == ElementType::EqualityJoint || Type == ElementType::EqualityTendon
		|| Type == ElementType::EqualityFlex || Type == ElementType::Flexvert
		|| Type == ElementType::Flexstrain)
		TargetParentName = TEXT("EqualitiesRoot");
	else
		return false;
#else
	return false;
#endif

	// Check if already correctly parented
	USCS_Node* CurrentParent = SCS->FindParentNode(Node);
	if (CurrentParent && CurrentParent->GetVariableName().ToString() == TargetParentName)
		return false;

	// Skip components that are under DefaultsRoot — they are default templates
	// and must not be moved to organizational folders
	{
		USCS_Node* Ancestor = CurrentParent;
		while (Ancestor)
		{
			if (Ancestor->GetVariableName().ToString() == TEXT("DefaultsRoot"))
				return false;
			Ancestor = SCS->FindParentNode(Ancestor);
		}
	}

	// Find the target parent node
	USCS_Node* TargetParent = nullptr;
	for (USCS_Node* SearchNode : SCS->GetAllNodes())
	{
		if (SearchNode->GetVariableName().ToString() == TargetParentName)
		{
			TargetParent = SearchNode;
			break;
		}
	}

	if (!TargetParent)
		return false;

	// Reparent
	if (CurrentParent)
	{
		CurrentParent->RemoveChildNode(Node);
	}
	else
	{
		SCS->RemoveNode(Node, /*bNotify=*/false);
	}
	TargetParent->AddChildNode(Node);

	FString CompName = Node->GetVariableName().ToString();
	UE_LOG(LogURLabEditor, Log, TEXT("[Auto-Parent] Moved '%s' to '%s'"), *CompName, *TargetParentName);
	return true;
}

IMPLEMENT_MODULE(FURLabEditorModule, URLabEditor)
