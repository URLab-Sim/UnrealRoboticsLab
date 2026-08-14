// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Convex decomposition, offered where the geom is.
//
// Decomposition is an action, not an attribute: it does not appear in the
// compiled model and nothing in MJCF describes it. So it lives on the
// Blueprint editor's component tree rather than in the details panel beside
// the geom's real attributes.
//
// The operation is split across the two modules because it has to be. Running
// CoACD and building the hull geoms is URLab's (`DecomposeMeshInto`); turning
// the OBJs it wrote into Unreal assets and `<mesh>` elements is this module's,
// because the import machinery is editor-only. A hull arrives here naming a
// mesh that does not exist yet, and finishing that is the whole job below.

#include "MjDecompositionMenu.h"

#include "GameFramework/Actor.h"
#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/StaticMesh.h"
#include "Kismet2/BlueprintEditorUtils.h"
#include "SubobjectEditorMenuContext.h"
#include "ToolMenus.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

#include "MujocoMeshImporter.h"

#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Elements/MjMesh.h"
#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Spec/MjNodeFactories.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "Utils/URLabLogging.h"

#if URLAB_MJ_GEN
#include "MuJoCo/Gen/Elements/Assets/MjAsset.gen.h"
#endif

#define LOCTEXT_NAMESPACE "MjDecompositionMenu"

namespace
{

/** The engine's own name for the Blueprint component tree's context menu. */
const FName kSubobjectContextMenu("Kismet.SubobjectEditorContextMenu");

/**
 * The mesh geoms in a selection, effective type resolved.
 *
 * Effective, not authored: a menagerie model writes `type="mesh"` on a
 * `<default>` class and leaves it off the geom, so asking the geom's own
 * storage answers `sphere` for most of the meshes worth decomposing.
 */
TArray<UMjGeom*> MeshGeomsIn(const TArray<UObject*>& Selection)
{
	TArray<UMjGeom*> Out;
	for (UObject* Object : Selection)
	{
		UMjGeom* const Geom = Cast<UMjGeom>(Object);
		if (Geom != nullptr && Geom->EffectiveType() == EMjGeomType::mesh)
		{
			Out.Add(Geom);
		}
	}
	return Out;
}

/** The Blueprint a component template belongs to, or null for an instance. */
UBlueprint* OwningBlueprint(const UMjGeom& Geom)
{
	for (UObject* Outer = Geom.GetOuter(); Outer != nullptr; Outer = Outer->GetOuter())
	{
		if (UBlueprint* const Found = Cast<UBlueprint>(Outer))
		{
			return Found;
		}
		if (UBlueprintGeneratedClass* const Generated = Cast<UBlueprintGeneratedClass>(Outer))
		{
			return Cast<UBlueprint>(Generated->ClassGeneratedBy);
		}
	}
	return nullptr;
}

#if URLAB_MJ_GEN

/**
 * The document's `<asset>` section, created if it has none.
 *
 * A model that declared no assets has no section to add to, which is the
 * ordinary case for a geom whose mesh came in through a class. Creating one is
 * what the reader would have done had the document mentioned an asset at all.
 */
template <class Factory>
UMjAsset* AssetSectionOf(const FSpecRef& Spec)
{
	UMjNodeComponent* const Root = Spec.GetRoot();
	if (Root == nullptr)
	{
		return nullptr;
	}
	for (const urlab::spec::FMjOrderedChild& Child : urlab::spec::MjOrderedChildrenOf(Spec, *Root))
	{
		if (UMjAsset* const Section = Cast<UMjAsset>(Child.Node))
		{
			return Section;
		}
	}
	return &Factory::template Create<UMjAsset>(*Root);
}

/**
 * Give `Hull` a `<mesh>` that resolves, and an Unreal asset to draw with.
 *
 * Both halves matter and answer different questions. `File` is what the
 * compiler reads; the imported UStaticMesh is what the editor draws. A hull
 * missing either is respectively uncompilable or invisible.
 */
template <class Factory>
bool FinishHull(const FSpecRef& Spec, const UMjGeom::FDecomposedHull& Hull,
	const FString& DestinationPath)
{
	UMjGeom* const Geom = Hull.Geom.Get();
	if (Geom == nullptr || Hull.MeshName.IsEmpty() || Hull.ObjPath.IsEmpty())
	{
		return false;
	}

	UMjAsset* const Section = AssetSectionOf<Factory>(Spec);
	if (Section == nullptr)
	{
		UE_LOG(LogURLabEditor, Warning,
			TEXT("[Decompose] '%s' has no document to add a <mesh> to."), *Hull.MeshName);
		return false;
	}

	UStaticMesh* const Asset = urlab::editor::ImportMeshAsset(Hull.ObjPath, DestinationPath, Hull.MeshName);

	UMjMeshBase& Element = Factory::template Create<UMjMeshBase>(*Section);
	Element.MjName = Hull.MeshName;
	// Absolute, because a generated hull has no source directory for the asset
	// pass to resolve a relative path against.
	Element.File = Hull.ObjPath;
	if (UMjMesh* const Imported = Cast<UMjMesh>(&Element))
	{
		Imported->MeshAsset = Asset;
		Imported->FileAsset = FSoftObjectPath(Asset);
	}

	// The reference the compiler follows. `MeshName` is a display name and
	// reaches nothing; without this the geom compiles naming no mesh at all.
	Geom->Mesh = Hull.MeshName;
	return true;
}

#endif // URLAB_MJ_GEN

/** Decompose every selected mesh geom and finish each hull it produced. */
void Decompose(TArray<TWeakObjectPtr<UMjGeom>> Geoms)
{
#if URLAB_MJ_GEN
	const FScopedTransaction Transaction(LOCTEXT("Decompose", "Decompose Mesh"));

	for (const TWeakObjectPtr<UMjGeom>& Weak : Geoms)
	{
		UMjGeom* const Geom = Weak.Get();
		if (Geom == nullptr)
		{
			continue;
		}

		const FSpecRef Spec = FSpecRef::OverOwner(Geom);
		UBlueprint* const BP = OwningBlueprint(*Geom);
		AActor* const Owner = Geom->GetOwner();
		if (BP == nullptr && Owner == nullptr)
		{
			continue;
		}

		TArray<UMjGeom::FDecomposedHull> Hulls;
		if (!Geom->DecomposeMeshInto(Hulls))
		{
			continue;
		}

		const FString DestinationPath = MjDecomposedMeshPath(Spec);
		int32 Finished = 0;
		if (BP != nullptr)
		{
			urlab::spec::FMjScsScope Scope(*BP);
			for (const UMjGeom::FDecomposedHull& Hull : Hulls)
			{
				Finished += FinishHull<urlab::spec::FScsNodeFactory>(Spec, Hull, DestinationPath) ? 1 : 0;
			}
		}
		else
		{
			urlab::spec::FMjInstanceScope Scope(*Owner);
			for (const UMjGeom::FDecomposedHull& Hull : Hulls)
			{
				Finished += FinishHull<urlab::spec::FInstanceNodeFactory>(Spec, Hull, DestinationPath) ? 1 : 0;
			}
		}

		UE_LOG(LogURLabEditor, Log, TEXT("[Decompose] '%s': %d/%d hulls finished into %s"),
			*Geom->GetName(), Finished, Hulls.Num(), *DestinationPath);

		if (BP != nullptr)
		{
			FBlueprintEditorUtils::MarkBlueprintAsStructurallyModified(BP);
		}
	}
#endif
}

void RemoveDecomposition(TArray<TWeakObjectPtr<UMjGeom>> Geoms)
{
	const FScopedTransaction Transaction(LOCTEXT("RemoveDecomposition", "Remove Decomposition"));
	for (const TWeakObjectPtr<UMjGeom>& Weak : Geoms)
	{
		if (UMjGeom* const Geom = Weak.Get())
		{
			Geom->RemoveDecomposition();
		}
	}
}

/** A labelled numeric row, bound straight to the geoms it will run over. */
TSharedRef<SWidget> NumericRow(const FText& Label, const FText& Tooltip, float Min, float Max,
	TFunction<float()> Get, TFunction<void(float)> Set)
{
	return SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center).Padding(2.f, 0.f)
			[SNew(STextBlock).Text(Label).ToolTipText(Tooltip)]
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(2.f, 0.f)
			[SNew(SSpinBox<float>)
					.MinValue(Min)
					.MaxValue(Max)
					.MinDesiredWidth(70.f)
					.ToolTipText(Tooltip)
					.Value_Lambda([Get]() { return Get(); })
					.OnValueChanged_Lambda([Set](float NewValue) { Set(NewValue); })];
}

/** The parameters and the two actions, for a selection already known to be meshes. */
void BuildSection(UToolMenu* Menu, TArray<UMjGeom*> Geoms)
{
	TArray<TWeakObjectPtr<UMjGeom>> Weak;
	for (UMjGeom* Geom : Geoms)
	{
		Weak.Add(Geom);
	}
	// The first selected geom shows the current values; a change writes to all
	// of them, so a multi-selection decomposes with one set of settings rather
	// than silently using each geom's own.
	TWeakObjectPtr<UMjGeom> First = Weak[0];

	FToolMenuSection& Section = Menu->AddSection("URLabDecomposition",
		LOCTEXT("SectionLabel", "Convex Decomposition"));

	Section.AddEntry(FToolMenuEntry::InitWidget("CoACDThreshold",
		NumericRow(LOCTEXT("Threshold", "Threshold"),
			LOCTEXT("ThresholdTip", "CoACD concavity threshold. Lower is more faithful and more hulls."),
			0.01f, 1.0f,
			[First]() { return First.IsValid() ? First->CoACDThreshold : 0.05f; },
			[Weak](float V) {
				for (const TWeakObjectPtr<UMjGeom>& G : Weak)
				{
					if (G.IsValid())
					{
						G->Modify();
						G->CoACDThreshold = V;
					}
				}
			}),
		FText::GetEmpty()));

	Section.AddEntry(FToolMenuEntry::InitWidget("CoACDExtrudeMargin",
		NumericRow(LOCTEXT("ExtrudeMargin", "Extrude Margin"),
			LOCTEXT("ExtrudeMarginTip",
				"How far an extruded hull is pushed out. Only used when Extrude is on."),
			0.0f, 1.0f,
			[First]() { return First.IsValid() ? First->CoACDExtrudeMargin : 0.01f; },
			[Weak](float V) {
				for (const TWeakObjectPtr<UMjGeom>& G : Weak)
				{
					if (G.IsValid())
					{
						G->Modify();
						G->CoACDExtrudeMargin = V;
					}
				}
			}),
		FText::GetEmpty()));

	Section.AddMenuEntry("CoACDExtrude",
		LOCTEXT("Extrude", "Extrude Hulls"),
		LOCTEXT("ExtrudeTip",
			"Thicken each hull along its base. Helps thin plates and shells, whose hulls are nearly "
			"flat and make the solver jitter; wrong for anything that has to fit."),
		FSlateIcon(),
		FUIAction(
			FExecuteAction::CreateLambda([Weak]() {
				const bool bNew = Weak[0].IsValid() ? !Weak[0]->bCoACDExtrude : true;
				for (const TWeakObjectPtr<UMjGeom>& G : Weak)
				{
					if (G.IsValid())
					{
						G->Modify();
						G->bCoACDExtrude = bNew;
					}
				}
			}),
			FCanExecuteAction(),
			FIsActionChecked::CreateLambda(
				[First]() { return First.IsValid() && First->bCoACDExtrude; })),
		EUserInterfaceActionType::ToggleButton);

	Section.AddMenuEntry("Decompose",
		LOCTEXT("DecomposeLabel", "Decompose Mesh"),
		LOCTEXT("DecomposeTip",
			"Run CoACD and add the hulls as collision geoms, with a <mesh> for each."),
		FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([Weak]() { Decompose(Weak); })));

	Section.AddMenuEntry("RemoveDecomposition",
		LOCTEXT("RemoveLabel", "Remove Decomposition"),
		LOCTEXT("RemoveTip", "Delete the hull geoms and put this geom's own collision back."),
		FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([Weak]() { RemoveDecomposition(Weak); })));
}

} // namespace

void FMjDecompositionMenu::Register()
{
	// Deferred: extending a menu before the menu system has started leaves it
	// rebuilding its widgets every tick, and that cost lands on the game thread.
	UToolMenus::RegisterStartupCallback(
		FSimpleMulticastDelegate::FDelegate::CreateStatic(&FMjDecompositionMenu::Extend));
}

void FMjDecompositionMenu::Extend()
{
	UToolMenus* const ToolMenus = UToolMenus::Get();
	if (ToolMenus == nullptr)
	{
		return;
	}

	// The engine registers this menu lazily, the first time someone right-clicks
	// the tree. Extending it is safe before that: an extension is recorded
	// against the name and applied when the menu is built.
	UToolMenu* const Menu = ToolMenus->ExtendMenu(kSubobjectContextMenu);
	if (Menu == nullptr)
	{
		return;
	}

	Menu->AddDynamicSection("URLabDecompositionDynamic", FNewToolMenuDelegate::CreateLambda(
		[](UToolMenu* InMenu) {
			USubobjectEditorMenuContext* const Context = InMenu->FindContext<USubobjectEditorMenuContext>();
			if (Context == nullptr)
			{
				return;
			}
			// Nothing is added for a selection with no mesh geom in it, which is
			// most selections: the entry appears where it applies and nowhere else.
			const TArray<UMjGeom*> Geoms = MeshGeomsIn(Context->GetSelectedObjects());
			if (Geoms.IsEmpty())
			{
				return;
			}
			BuildSection(InMenu, Geoms);
		}));
}

#undef LOCTEXT_NAMESPACE
