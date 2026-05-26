// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MjLevelOps.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "AssetToolsModule.h"
#include "AutomatedAssetImportData.h"
#include "Editor.h"
#include "EngineUtils.h"
#include "Engine/Blueprint.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "Engine/World.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/WorldSettings.h"
#include "IAssetTools.h"
#include "LevelEditorSubsystem.h"
#include "Misc/Paths.h"
#include "ObjectTools.h"
#include "UObject/UObjectGlobals.h"
#include "URLabEditorLogging.h"

#include "Engine/DirectionalLight.h"
#include "Engine/PointLight.h"
#include "Engine/SpotLight.h"
#include "Components/LightComponent.h"

#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/QuickConvert/MjQuickConvertComponent.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Sensors/MjSensor.h"
#include "MuJoCo/Components/Sensors/MjCamera.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "MujocoImportFactory.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/Light.h"

namespace URLabLevelOps
{
    bool ImportXmlSync(
        const FString& AbsXmlPath,
        bool bForceReimport,
        FString& OutBlueprintClassPath,
        FString& OutBlueprintShortName,
        bool& bOutImportedNow,
        FString& OutError)
    {
        OutBlueprintClassPath.Empty();
        OutBlueprintShortName.Empty();
        bOutImportedNow = false;
        OutError.Empty();

        if (!FPaths::FileExists(AbsXmlPath))
        {
            OutError = FString::Printf(TEXT("xml file not found: %s"), *AbsXmlPath);
            return false;
        }

        const FString StemBase = FPaths::GetBaseFilename(AbsXmlPath);
        const FString DestPath = TEXT("/Game/MuJoCoImports");
        const FString ObjectPath = FString::Printf(TEXT("%s/%s.%s"), *DestPath, *StemBase, *StemBase);
        const FString GeneratedClassPath = FString::Printf(
            TEXT("%s/%s.%s_C"), *DestPath, *StemBase, *StemBase);

        // Reuse path: asset already exists and caller didn't force a reimport.
        FAssetRegistryModule& AssetRegistry =
            FModuleManager::LoadModuleChecked<FAssetRegistryModule>(TEXT("AssetRegistry"));
        FAssetData Existing = AssetRegistry.Get().GetAssetByObjectPath(FSoftObjectPath(ObjectPath));
        if (!bForceReimport && Existing.IsValid())
        {
            OutBlueprintClassPath = GeneratedClassPath;
            OutBlueprintShortName = StemBase;
            bOutImportedNow = false;
            return true;
        }

        // Force-reimport path: ImportAssetsAutomated(bReplaceExisting=true)
        // does NOT delete the in-memory UBlueprint before our factory's
        // FKismetEditorUtilities::CreateBlueprint runs. CreateBlueprint
        // asserts on FindObject<UBlueprint>(...) == 0 -> editor crash.
        // Destroy the existing asset ourselves first.
        if (bForceReimport && Existing.IsValid())
        {
            bool bWasFound = false;
            FString DestroyErr;
            if (!DestroyAssetSync(ObjectPath, bWasFound, DestroyErr))
            {
                OutError = FString::Printf(
                    TEXT("force_reimport failed to clear existing BP %s: %s"),
                    *ObjectPath, *DestroyErr);
                return false;
            }
        }

        // Drive the existing factory programmatically.
        FAssetToolsModule& AssetToolsModule =
            FModuleManager::LoadModuleChecked<FAssetToolsModule>(TEXT("AssetTools"));
        UAutomatedAssetImportData* ImportData = NewObject<UAutomatedAssetImportData>();
        ImportData->Filenames.Add(AbsXmlPath);
        ImportData->DestinationPath = DestPath;
        ImportData->bReplaceExisting = bForceReimport;
        // FactoryName must match the registered factory's class short name.
        ImportData->FactoryName = TEXT("MujocoImportFactory");

        TArray<UObject*> Imported = AssetToolsModule.Get().ImportAssetsAutomated(ImportData);
        if (Imported.Num() == 0)
        {
            OutError = TEXT("ImportAssetsAutomated returned no objects");
            return false;
        }

        UObject* First = Imported[0];
        UBlueprint* BP = Cast<UBlueprint>(First);
        if (!BP)
        {
            OutError = FString::Printf(
                TEXT("imported object is not a UBlueprint (got %s)"),
                *First->GetClass()->GetName());
            return false;
        }

        OutBlueprintShortName = BP->GetName();
        if (BP->GeneratedClass)
        {
            OutBlueprintClassPath = BP->GeneratedClass->GetPathName();
        }
        else
        {
            // Fall back to the synthesised path; rare on a successful import.
            OutBlueprintClassPath = FString::Printf(
                TEXT("%s/%s.%s_C"), *DestPath,
                *OutBlueprintShortName, *OutBlueprintShortName);
        }
        bOutImportedNow = true;
        return true;
    }

    FString ResolveLevelPath(const FString& NameOrPath)
    {
        if (NameOrPath.StartsWith(TEXT("/Game/"))) return NameOrPath;
        return FString::Printf(TEXT("/Game/Levels/%s"), *NameOrPath);
    }

    namespace
    {
        ULevelEditorSubsystem* GetLevelSub()
        {
            return GEditor ? GEditor->GetEditorSubsystem<ULevelEditorSubsystem>() : nullptr;
        }
    }

    bool CreateLevelSync(const FString& NameOrPath, bool bForceOverwrite,
                         FString& OutLevelPath, FString& OutError)
    {
        ULevelEditorSubsystem* Sub = GetLevelSub();
        if (!Sub) { OutError = TEXT("LevelEditorSubsystem unavailable"); return false; }

        OutLevelPath = ResolveLevelPath(NameOrPath);

        if (bForceOverwrite)
        {
            const FString AssetName = FPaths::GetBaseFilename(OutLevelPath);
            const FString ObjectPath = FString::Printf(TEXT("%s.%s"), *OutLevelPath, *AssetName);
            bool bWasFound = false;
            FString DestroyErr;
            if (!DestroyAssetSync(ObjectPath, bWasFound, DestroyErr))
            {
                OutError = FString::Printf(
                    TEXT("force_overwrite failed to clear %s: %s"),
                    *ObjectPath, *DestroyErr);
                return false;
            }
        }

        if (!Sub->NewLevel(OutLevelPath))
        {
            OutError = FString::Printf(
                TEXT("NewLevel failed for %s (already exists?)"), *OutLevelPath);
            return false;
        }

        // Override the world's default game mode to AGameModeBase so PIE
        // doesn't pull in the project-default game mode (and its transitive
        // BP references). Without this, projects with broken third-party
        // content -- e.g. Marketplace Mannequin AnimBPs that fail to
        // compile -- pop a "Blueprint Compilation Errors" modal during
        // PIE start, which blocks the game thread and times out begin_pie
        // at 30s. AGameModeBase is the minimal default and has no BP refs.
        if (UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr)
        {
            if (AWorldSettings* WS = World->GetWorldSettings())
            {
                WS->DefaultGameMode = AGameModeBase::StaticClass();
                WS->MarkPackageDirty();
            }
        }

        if (!Sub->SaveCurrentLevel())
        {
            OutError = TEXT("NewLevel succeeded but SaveCurrentLevel failed");
            return false;
        }
        return true;
    }

    bool DestroyAssetSync(const FString& ObjectPath, bool& bOutWasFound, FString& OutError)
    {
        bOutWasFound = false;
        OutError.Empty();

        FAssetRegistryModule& AR = FModuleManager::LoadModuleChecked<FAssetRegistryModule>(TEXT("AssetRegistry"));
        FAssetData Existing = AR.Get().GetAssetByObjectPath(FSoftObjectPath(ObjectPath));
        if (!Existing.IsValid())
        {
            UE_LOG(LogURLabEditor, Log,
                TEXT("DestroyAsset: no asset at %s (idempotent no-op)"), *ObjectPath);
            return true;
        }
        bOutWasFound = true;

        // If a UWorld currently loaded, switch off it first so its
        // package can be deleted.
        UWorld* CurWorld = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
        if (CurWorld && CurWorld->GetOutermost() == Existing.GetPackage())
        {
            ULevelEditorSubsystem* Sub = GetLevelSub();
            if (Sub) Sub->LoadLevel(TEXT("/Engine/Maps/Templates/Template_Default"));
        }

        UObject* AssetObj = Existing.GetAsset();
        if (!AssetObj)
        {
            OutError = FString::Printf(
                TEXT("AssetData.GetAsset() returned null for %s"), *ObjectPath);
            return false;
        }
        const FString PathBeforeDelete = AssetObj->GetPathName();
        TArray<UObject*> ToDelete = { AssetObj };
        const int32 NumDeleted = ObjectTools::ForceDeleteObjects(ToDelete, /*bShowConfirmation*/ false);
        UE_LOG(LogURLabEditor, Log,
            TEXT("DestroyAsset: ForceDeleteObjects returned %d for %s"),
            NumDeleted, *PathBeforeDelete);
        if (NumDeleted == 0)
        {
            OutError = FString::Printf(
                TEXT("ForceDeleteObjects refused %s (live references?)"),
                *PathBeforeDelete);
            return false;
        }
        return true;
    }

    bool LoadLevelSync(const FString& NameOrPath, FString& OutLevelPath, FString& OutError)
    {
        ULevelEditorSubsystem* Sub = GetLevelSub();
        if (!Sub) { OutError = TEXT("LevelEditorSubsystem unavailable"); return false; }

        OutLevelPath = ResolveLevelPath(NameOrPath);
        if (!Sub->LoadLevel(OutLevelPath))
        {
            OutError = FString::Printf(TEXT("LoadLevel failed for %s"), *OutLevelPath);
            return false;
        }
        return true;
    }

    bool SaveCurrentLevelSync(FString& OutLevelPath, FString& OutError)
    {
        ULevelEditorSubsystem* Sub = GetLevelSub();
        if (!Sub) { OutError = TEXT("LevelEditorSubsystem unavailable"); return false; }
        if (!Sub->SaveCurrentLevel())
        {
            OutError = TEXT("SaveCurrentLevel failed");
            return false;
        }
        // Best-effort report: editor's current world's package name.
        if (UWorld* W = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr)
        {
            OutLevelPath = W->GetOutermost()->GetName();
        }
        return true;
    }

    namespace
    {
        // Tag prefix for non-articulation actors (props / lights). Pair
        // with the AMjArticulation::ActorId UPROPERTY for articulations.
        const TCHAR* kActorIdTagPrefix = TEXT("URLab.ActorId=");

        FString MakeActorIdTag(const FString& Id)
        {
            return FString::Printf(TEXT("%s%s"), kActorIdTagPrefix, *Id);
        }

        bool ActorMatchesActorId(AActor* A, const FString& Id)
        {
            if (!A) return false;
            if (AMjArticulation* Mj = Cast<AMjArticulation>(A))
            {
                if (Mj->ActorId.Equals(Id)) return true;
            }
            const FName Tag(*MakeActorIdTag(Id));
            return A->Tags.Contains(Tag);
        }

        // /Game/MuJoCoImports/foo  ->  /Game/MuJoCoImports/foo.foo_C
        // /Game/MuJoCoImports/foo.foo_C -> unchanged
        FString ResolveBpClassPath(const FString& In)
        {
            if (In.EndsWith(TEXT("_C"))) return In;
            int32 SlashIdx = INDEX_NONE;
            if (!In.FindLastChar('/', SlashIdx)) return In;
            const FString Stem = In.Mid(SlashIdx + 1);
            return FString::Printf(TEXT("%s.%s_C"), *In, *Stem);
        }
    }

    namespace
    {
        AActor* FindByActorIdOnly(UWorld* World, const FString& Id)
        {
            if (!World || Id.IsEmpty()) return nullptr;
            for (TActorIterator<AActor> It(World); It; ++It)
            {
                AActor* A = *It;
                if (A && ActorMatchesActorId(A, Id))
                {
                    return A;
                }
            }
            return nullptr;
        }
    }

    bool SpawnActorSync(
        const FString& BlueprintNameOrPath,
        const FString& ActorId,
        const FVector& LocationMeters,
        const FQuat&   RotationQuatXyzw,
        const FVector& Scale,
        FString& OutActorName,
        FString& OutActorPath,
        FString& OutBlueprintClassPath,
        bool&    OutWasExisting,
        FString& OutError)
    {
        OutActorName.Empty();
        OutActorPath.Empty();
        OutBlueprintClassPath.Empty();
        OutWasExisting = false;
        OutError.Empty();

        if (!GEditor) { OutError = TEXT("GEditor null"); return false; }
        UWorld* World = GEditor->GetEditorWorldContext().World();
        if (!World) { OutError = TEXT("editor world unavailable"); return false; }

        OutBlueprintClassPath = ResolveBpClassPath(BlueprintNameOrPath);
        UClass* BPClass = LoadObject<UClass>(nullptr, *OutBlueprintClassPath);
        if (!BPClass)
        {
            OutError = FString::Printf(TEXT("blueprint class not found: %s"),
                *OutBlueprintClassPath);
            return false;
        }

        // MJ -> UE: cm + Y-flip + (handedness baked into MjUtils helpers).
        // Accept the conversion routines we already use everywhere else
        // so spawn matches imported geometry to within float precision.
        double MjPos[3] = {LocationMeters.X, LocationMeters.Y, LocationMeters.Z};
        FVector UELoc = MjUtils::MjToUEPosition(MjPos);

        // Quaternion convention is (w, x, y, z) on the wire when read by
        // MjUtils, so re-pack from xyzw.
        double MjQuat[4] = {
            RotationQuatXyzw.W,
            RotationQuatXyzw.X,
            RotationQuatXyzw.Y,
            RotationQuatXyzw.Z};
        FQuat UEQuat = MjUtils::MjToUERotation(MjQuat);
        FRotator UERot = UEQuat.Rotator();

        // Idempotent path: a non-empty ActorId may already be in the world.
        // Update transform on the existing actor instead of duplicating it.
        // Empty ActorId always falls through to fresh spawn.
        if (AActor* Existing = FindByActorIdOnly(World, ActorId))
        {
            if (Existing->GetClass() != BPClass)
            {
                OutError = FString::Printf(
                    TEXT("actor_id '%s' already in world with class %s; "
                         "caller asked for %s. Destroy first or pick a "
                         "different actor_id."),
                    *ActorId,
                    *Existing->GetClass()->GetPathName(),
                    *OutBlueprintClassPath);
                return false;
            }
            Existing->SetActorLocationAndRotation(UELoc, UERot);
            Existing->SetActorScale3D(Scale);
            OutActorName = Existing->GetName();
            OutActorPath = Existing->GetPathName();
            OutWasExisting = true;
            return true;
        }

        FActorSpawnParameters Params;
        Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        AActor* Actor = World->SpawnActor(BPClass, &UELoc, &UERot, Params);
        if (!Actor)
        {
            OutError = FString::Printf(TEXT("SpawnActor returned null for %s"),
                *OutBlueprintClassPath);
            return false;
        }
        Actor->SetActorScale3D(Scale);

        // Stash the actor id on the actor.
        if (!ActorId.IsEmpty())
        {
            if (AMjArticulation* Mj = Cast<AMjArticulation>(Actor))
            {
                Mj->ActorId = ActorId;
            }
            else
            {
                Actor->Tags.AddUnique(FName(*MakeActorIdTag(ActorId)));
            }
        }

        OutActorName = Actor->GetName();
        OutActorPath = Actor->GetPathName();
        return true;
    }

    namespace
    {
        UClass* LightClassForKind(const FString& Kind)
        {
            if (Kind.Equals(TEXT("directional"), ESearchCase::IgnoreCase))
                return ADirectionalLight::StaticClass();
            if (Kind.Equals(TEXT("point"), ESearchCase::IgnoreCase))
                return APointLight::StaticClass();
            if (Kind.Equals(TEXT("spot"), ESearchCase::IgnoreCase))
                return ASpotLight::StaticClass();
            return nullptr;
        }
    }

    bool SpawnLightSync(
        const FString& Kind,
        const FString& ActorId,
        const FVector& LocationMeters,
        const FVector& RotationEulerDegrees,
        float Intensity,
        const FLinearColor& Color,
        FString& OutActorName,
        FString& OutActorPath,
        FString& OutResolvedKind,
        FString& OutError)
    {
        OutActorName.Empty();
        OutActorPath.Empty();
        OutResolvedKind.Empty();
        OutError.Empty();

        if (!GEditor) { OutError = TEXT("GEditor null"); return false; }
        UWorld* World = GEditor->GetEditorWorldContext().World();
        if (!World) { OutError = TEXT("editor world unavailable"); return false; }

        UClass* LightClass = LightClassForKind(Kind);
        if (!LightClass)
        {
            OutError = FString::Printf(
                TEXT("unknown light kind '%s' (expect directional|point|spot)"),
                *Kind);
            return false;
        }
        OutResolvedKind = Kind.ToLower();

        // Position: MJ -> UE (cm + Y-flip + handedness handled by helper).
        double MjPos[3] = {LocationMeters.X, LocationMeters.Y, LocationMeters.Z};
        const FVector UELoc = MjUtils::MjToUEPosition(MjPos);

        // Rotation: input is degrees in (Roll, Pitch, Yaw) along MJ axes
        // (X, Y, Z respectively). FRotator constructor is (Pitch, Yaw, Roll).
        const FRotator UERot(RotationEulerDegrees.Y, RotationEulerDegrees.Z,
                             RotationEulerDegrees.X);

        FActorSpawnParameters Params;
        Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        AActor* Actor = World->SpawnActor(LightClass, &UELoc, &UERot, Params);
        if (!Actor)
        {
            OutError = FString::Printf(TEXT("SpawnActor returned null for light '%s'"), *Kind);
            return false;
        }

        // Apply intensity / colour through the LightComponent.
        ULightComponent* LC = Actor->FindComponentByClass<ULightComponent>();
        if (LC)
        {
            LC->SetIntensity(Intensity);
            LC->SetLightColor(Color);
        }

        // Stash the actor id as a UE actor tag (lights aren't AMjArticulation).
        if (!ActorId.IsEmpty())
        {
            Actor->Tags.AddUnique(FName(*MakeActorIdTag(ActorId)));
        }

        OutActorName = Actor->GetName();
        OutActorPath = Actor->GetPathName();
        return true;
    }

    namespace
    {
        AActor* FindByActorIdOrName(UWorld* World, const FString& Key)
        {
            if (!World) return nullptr;
            for (TActorIterator<AActor> It(World); It; ++It)
            {
                AActor* A = *It;
                if (!A) continue;
                if (ActorMatchesActorId(A, Key) || A->GetName().Equals(Key))
                {
                    return A;
                }
            }
            return nullptr;
        }
    }

    bool DestroyActorSync(const FString& ActorIdOrActorName, FString& OutError)
    {
        OutError.Empty();
        if (!GEditor) { OutError = TEXT("GEditor null"); return false; }
        UWorld* World = GEditor->GetEditorWorldContext().World();
        if (!World) { OutError = TEXT("editor world unavailable"); return false; }

        AActor* Found = FindByActorIdOrName(World, ActorIdOrActorName);
        if (!Found)
        {
            OutError = FString::Printf(TEXT("no actor matching '%s'"),
                *ActorIdOrActorName);
            return false;
        }
        World->DestroyActor(Found);
        return true;
    }

    bool SetActorTransformSync(
        const FString& ActorIdOrActorName,
        const FVector* LocationMeters,
        const FQuat*   RotationQuatXyzw,
        FString& OutActorName,
        FString& OutError)
    {
        OutActorName.Empty();
        OutError.Empty();
        if (!GEditor) { OutError = TEXT("GEditor null"); return false; }
        UWorld* World = GEditor->GetEditorWorldContext().World();
        if (!World) { OutError = TEXT("editor world unavailable"); return false; }

        AActor* A = FindByActorIdOrName(World, ActorIdOrActorName);
        if (!A)
        {
            OutError = FString::Printf(TEXT("no actor matching '%s'"),
                *ActorIdOrActorName);
            return false;
        }
        OutActorName = A->GetName();

        FTransform T = A->GetActorTransform();
        if (LocationMeters)
        {
            double MjPos[3] = {LocationMeters->X, LocationMeters->Y, LocationMeters->Z};
            T.SetLocation(MjUtils::MjToUEPosition(MjPos));
        }
        if (RotationQuatXyzw)
        {
            double MjQuat[4] = {
                RotationQuatXyzw->W,
                RotationQuatXyzw->X,
                RotationQuatXyzw->Y,
                RotationQuatXyzw->Z};
            T.SetRotation(MjUtils::MjToUERotation(MjQuat));
        }
        A->SetActorTransform(T, /*bSweep=*/false, nullptr,
            ETeleportType::TeleportPhysics);
        return true;
    }

    namespace
    {
        FString ExtractActorId(AActor* A)
        {
            if (!A) return FString();
            if (AMjArticulation* Mj = Cast<AMjArticulation>(A))
            {
                if (!Mj->ActorId.IsEmpty()) return Mj->ActorId;
            }
            for (const FName& Tag : A->Tags)
            {
                FString S = Tag.ToString();
                if (S.StartsWith(kActorIdTagPrefix))
                {
                    return S.RightChop(FCString::Strlen(kActorIdTagPrefix));
                }
            }
            return FString();
        }

        bool ShouldListActor(AActor* A)
        {
            if (!A) return false;
            // Skip the editor's transient/utility actors so the outliner
            // shows only user-placed scene content.
            if (A->IsA(ABrush::StaticClass())) return false;
            if (A->GetClass()->GetName().Contains(TEXT("WorldSettings"))) return false;
            if (A->GetClass()->GetName().Contains(TEXT("DefaultPhysicsVolume"))) return false;
            if (A->bHiddenEdLayer) return false;
            // Editor-only / engine actors that clutter the list.
            const FString ClassName = A->GetClass()->GetName();
            if (ClassName.StartsWith(TEXT("AbstractNavData"))) return false;
            if (ClassName == TEXT("WorldDataLayers")) return false;
            if (ClassName == TEXT("LevelInstanceEditorInstanceActor")) return false;
            if (ClassName == TEXT("WorldPartition")) return false;
            return true;
        }
    }

    bool ListActorsSync(
        TArray<TSharedPtr<FJsonValue>>& OutActors,
        FString& OutError)
    {
        OutActors.Reset();
        OutError.Empty();
        if (!GEditor) { OutError = TEXT("GEditor null"); return false; }
        UWorld* World = GEditor->GetEditorWorldContext().World();
        if (!World) { OutError = TEXT("editor world unavailable"); return false; }

        for (TActorIterator<AActor> It(World); It; ++It)
        {
            AActor* A = *It;
            if (!ShouldListActor(A)) continue;

            TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
            Obj->SetStringField(TEXT("name"),  A->GetName());
            // The user-visible label from UE's own outliner (set via the
            // Details > "Actor Label" field). Falls back to GetName if
            // empty so the bridge UI never shows a blank row.
            #if WITH_EDITOR
                FString Label = A->GetActorLabel();
            #else
                FString Label = A->GetName();
            #endif
            if (Label.IsEmpty()) Label = A->GetName();
            Obj->SetStringField(TEXT("label"), Label);
            Obj->SetStringField(TEXT("class"), A->GetClass()->GetName());

            const FString Aid = ExtractActorId(A);
            Obj->SetStringField(TEXT("actor_id"), Aid);
            Obj->SetBoolField(TEXT("is_articulation"),
                A->IsA(AMjArticulation::StaticClass()));
            Obj->SetBoolField(TEXT("is_static_mesh_actor"),
                A->IsA(AStaticMeshActor::StaticClass()));
            Obj->SetBoolField(TEXT("is_light"),
                A->IsA(ALight::StaticClass()));

            // MJ-native pose: convert UE cm -> metres, UE rotation -> MJ quat.
            const FVector  UELoc = A->GetActorLocation();
            const FQuat    UEQ   = A->GetActorQuat();
            double MjPos[3], MjQuat[4];
            MjUtils::UEToMjPosition(UELoc, MjPos);
            MjUtils::UEToMjRotation(UEQ,   MjQuat);
            TArray<TSharedPtr<FJsonValue>> LocOut;
            for (int i = 0; i < 3; ++i)
                LocOut.Add(MakeShared<FJsonValueNumber>(MjPos[i]));
            Obj->SetArrayField(TEXT("location"), LocOut);
            // MJ quat is wxyz; bridge convention is xyzw, match the rest of
            // the wire format.
            TArray<TSharedPtr<FJsonValue>> QuatOut;
            QuatOut.Add(MakeShared<FJsonValueNumber>(MjQuat[1]));
            QuatOut.Add(MakeShared<FJsonValueNumber>(MjQuat[2]));
            QuatOut.Add(MakeShared<FJsonValueNumber>(MjQuat[3]));
            QuatOut.Add(MakeShared<FJsonValueNumber>(MjQuat[0]));
            Obj->SetArrayField(TEXT("rotation_quat"), QuatOut);

            // Quick-convert summary, when present.
            if (UMjQuickConvertComponent* QC =
                    A->FindComponentByClass<UMjQuickConvertComponent>())
            {
                Obj->SetBoolField(TEXT("has_quick_convert"), true);
                TSharedPtr<FJsonObject> QcObj = MakeShared<FJsonObject>();
                QcObj->SetBoolField(TEXT("static"),           QC->Static);
                QcObj->SetBoolField(TEXT("complex_mesh"),     QC->ComplexMeshRequired);
                QcObj->SetNumberField(TEXT("coacd_threshold"), QC->CoACDThreshold);
                QcObj->SetBoolField(TEXT("driven_by_unreal"), QC->bDrivenByUnreal);
                TArray<TSharedPtr<FJsonValue>> Friction;
                Friction.Add(MakeShared<FJsonValueNumber>(QC->friction.X));
                Friction.Add(MakeShared<FJsonValueNumber>(QC->friction.Y));
                Friction.Add(MakeShared<FJsonValueNumber>(QC->friction.Z));
                QcObj->SetArrayField(TEXT("friction"), Friction);
                Obj->SetObjectField(TEXT("quick_convert"), QcObj);
            }
            else
            {
                Obj->SetBoolField(TEXT("has_quick_convert"), false);
            }

            OutActors.Add(MakeShared<FJsonValueObject>(Obj));
        }
        return true;
    }

    bool SelectActorSync(
        const FString& ActorIdOrActorName,
        FString& OutActorName,
        FString& OutError)
    {
        OutActorName.Empty();
        OutError.Empty();
        if (!GEditor) { OutError = TEXT("GEditor null"); return false; }
        UWorld* World = GEditor->GetEditorWorldContext().World();
        if (!World) { OutError = TEXT("editor world unavailable"); return false; }

        AActor* A = FindByActorIdOrName(World, ActorIdOrActorName);
        if (!A)
        {
            OutError = FString::Printf(TEXT("no actor matching '%s'"),
                *ActorIdOrActorName);
            return false;
        }
        GEditor->SelectNone(/*bNoteSelectionChange=*/false,
                            /*bDeselectBSPSurfs=*/true,
                            /*bWarnAboutManyActors=*/false);
        GEditor->SelectActor(A, /*bInSelected=*/true,
                             /*bNotify=*/true,
                             /*bSelectEvenIfHidden=*/true);
        OutActorName = A->GetName();
        return true;
    }

    bool AddQuickConvertSync(
        const FString& ActorIdOrActorName,
        bool bStatic,
        bool bComplexMesh,
        float CoACDThreshold,
        bool bDrivenByUnreal,
        const FVector& Friction,
        FString& OutActorName,
        FString& OutError)
    {
        OutActorName.Empty();
        OutError.Empty();
        if (!GEditor) { OutError = TEXT("GEditor null"); return false; }
        UWorld* World = GEditor->GetEditorWorldContext().World();
        if (!World) { OutError = TEXT("editor world unavailable"); return false; }

        AActor* A = FindByActorIdOrName(World, ActorIdOrActorName);
        if (!A)
        {
            OutError = FString::Printf(TEXT("no actor matching '%s'"),
                *ActorIdOrActorName);
            return false;
        }
        OutActorName = A->GetName();

        UMjQuickConvertComponent* QC =
            A->FindComponentByClass<UMjQuickConvertComponent>();
        if (!QC)
        {
            QC = NewObject<UMjQuickConvertComponent>(A);
            if (!QC) { OutError = TEXT("NewObject UMjQuickConvertComponent failed"); return false; }
            QC->RegisterComponent();
            A->AddInstanceComponent(QC);
        }
        QC->Static               = bStatic;
        QC->ComplexMeshRequired  = bComplexMesh;
        QC->CoACDThreshold       = CoACDThreshold;
        QC->bDrivenByUnreal      = bDrivenByUnreal;
        QC->friction             = FVector3d(Friction.X, Friction.Y, Friction.Z);
        A->Modify();
        return true;
    }

    bool ListBlueprintsSync(
        TArray<TSharedPtr<FJsonValue>>& OutBlueprints,
        FString& OutError)
    {
        OutBlueprints.Reset();
        OutError.Empty();

        // Articulation blueprints don't live in one canonical folder --
        // import factories drop them in `/Game/MJCF_Importing` while
        // older flows used `/Game/MuJoCoImports`, and users can hand-place
        // BPs anywhere. Source-of-truth: any UBlueprint whose parent
        // class derives from AMjArticulation is a candidate.
        FAssetRegistryModule& AssetRegistry =
            FModuleManager::LoadModuleChecked<FAssetRegistryModule>(TEXT("AssetRegistry"));
        IAssetRegistry& Registry = AssetRegistry.Get();

        FARFilter Filter;
        Filter.ClassPaths.Add(UBlueprint::StaticClass()->GetClassPathName());
        Filter.bRecursivePaths = true;
        Filter.PackagePaths.Add(FName(TEXT("/Game")));
        TArray<FAssetData> Assets;
        Registry.GetAssets(Filter, Assets);

        const FName ParentClassTag(TEXT("ParentClass"));
        for (const FAssetData& Data : Assets)
        {
            // Read the cached ParentClass tag (asset-registry path) so we
            // don't have to LoadObject every BP just to check ancestry.
            FString ParentClassExportPath;
            if (!Data.GetTagValue(ParentClassTag, ParentClassExportPath)) continue;

            // ParentClass is stored as e.g. `/Script/URLab.MjArticulation`
            // (engine class) or `BlueprintGeneratedClass'/Game/.../parent.parent_C'`
            // (BP-derived parent). Resolve via FindObject so we can ask UClass
            // about its inheritance chain.
            const FString ResolvedPath =
                FPackageName::ExportTextPathToObjectPath(ParentClassExportPath);
            UClass* ParentClass = FindObject<UClass>(nullptr, *ResolvedPath);
            if (!ParentClass)
            {
                ParentClass = LoadObject<UClass>(nullptr, *ResolvedPath);
            }
            if (!ParentClass) continue;
            if (!ParentClass->IsChildOf(AMjArticulation::StaticClass())) continue;

            const FString PackagePath = Data.PackageName.ToString();
            const FString ShortName   = Data.AssetName.ToString();
            const FString ClassPath   = FString::Printf(TEXT("%s.%s_C"),
                *PackagePath, *ShortName);

            TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
            Obj->SetStringField(TEXT("blueprint_class_path"), ClassPath);
            Obj->SetStringField(TEXT("blueprint_short_name"), ShortName);
            Obj->SetStringField(TEXT("package_path"),         PackagePath);
            Obj->SetStringField(TEXT("parent_class"),         ParentClass->GetName());
            OutBlueprints.Add(MakeShared<FJsonValueObject>(Obj));
        }
        // Sort alphabetically by short name for a stable UI ordering.
        OutBlueprints.Sort([](const TSharedPtr<FJsonValue>& A,
                              const TSharedPtr<FJsonValue>& B)
        {
            FString An, Bn;
            A->AsObject()->TryGetStringField(TEXT("blueprint_short_name"), An);
            B->AsObject()->TryGetStringField(TEXT("blueprint_short_name"), Bn);
            return An < Bn;
        });
        return true;
    }

    bool RemoveQuickConvertSync(
        const FString& ActorIdOrActorName,
        FString& OutActorName,
        FString& OutError)
    {
        OutActorName.Empty();
        OutError.Empty();
        if (!GEditor) { OutError = TEXT("GEditor null"); return false; }
        UWorld* World = GEditor->GetEditorWorldContext().World();
        if (!World) { OutError = TEXT("editor world unavailable"); return false; }

        AActor* A = FindByActorIdOrName(World, ActorIdOrActorName);
        if (!A)
        {
            OutError = FString::Printf(TEXT("no actor matching '%s'"),
                *ActorIdOrActorName);
            return false;
        }
        OutActorName = A->GetName();

        UMjQuickConvertComponent* QC =
            A->FindComponentByClass<UMjQuickConvertComponent>();
        if (!QC)
        {
            OutError = TEXT("actor has no UMjQuickConvertComponent");
            return false;
        }
        A->RemoveInstanceComponent(QC);
        QC->DestroyComponent();
        A->Modify();
        return true;
    }

    // ====================================================================
    // Scene introspection ops.
    // ====================================================================

    namespace
    {
        // Build the same per-actor row shape ListActorsSync emits. Lifted
        // out so FindActorsSync + SnapshotSceneSync share it.
        TSharedPtr<FJsonObject> BuildActorRow(AActor* A)
        {
            TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
            Obj->SetStringField(TEXT("name"), A->GetName());
            #if WITH_EDITOR
                FString Label = A->GetActorLabel();
            #else
                FString Label = A->GetName();
            #endif
            if (Label.IsEmpty()) Label = A->GetName();
            Obj->SetStringField(TEXT("label"), Label);
            Obj->SetStringField(TEXT("class"), A->GetClass()->GetName());
            Obj->SetStringField(TEXT("actor_id"), ExtractActorId(A));

            const FVector UELoc = A->GetActorLocation();
            const FQuat   UEQ   = A->GetActorQuat();
            double MjPos[3], MjQuat[4];
            MjUtils::UEToMjPosition(UELoc, MjPos);
            MjUtils::UEToMjRotation(UEQ,   MjQuat);
            TArray<TSharedPtr<FJsonValue>> LocOut;
            for (int i = 0; i < 3; ++i)
                LocOut.Add(MakeShared<FJsonValueNumber>(MjPos[i]));
            Obj->SetArrayField(TEXT("location"), LocOut);
            // Bridge convention: xyzw on the wire, MJ stores wxyz.
            TArray<TSharedPtr<FJsonValue>> QuatOut;
            QuatOut.Add(MakeShared<FJsonValueNumber>(MjQuat[1]));
            QuatOut.Add(MakeShared<FJsonValueNumber>(MjQuat[2]));
            QuatOut.Add(MakeShared<FJsonValueNumber>(MjQuat[3]));
            QuatOut.Add(MakeShared<FJsonValueNumber>(MjQuat[0]));
            Obj->SetArrayField(TEXT("rotation_quat"), QuatOut);

            TArray<TSharedPtr<FJsonValue>> TagsOut;
            for (const FName& Tag : A->Tags)
                TagsOut.Add(MakeShared<FJsonValueString>(Tag.ToString()));
            Obj->SetArrayField(TEXT("tags"), TagsOut);
            return Obj;
        }

        UWorld* PickWorld(bool bWantPie, bool& bOutChosePie)
        {
            bOutChosePie = false;
            if (!GEditor) return nullptr;
            UWorld* PieWorld = GEditor->PlayWorld;
            if (bWantPie && PieWorld)
            {
                bOutChosePie = true;
                return PieWorld;
            }
            // No PIE running, or caller asked for editor world.
            return GEditor->GetEditorWorldContext().World();
        }
    }

    bool FindActorsSync(
        const FString& ClassFilter,
        const FString& TagFilter,
        const FString& NamePrefix,
        bool bSearchPieWorld,
        TArray<TSharedPtr<FJsonValue>>& OutActors,
        bool& bOutSearchedPie,
        FString& OutError)
    {
        OutActors.Reset();
        OutError.Empty();
        bOutSearchedPie = false;

        UWorld* World = PickWorld(bSearchPieWorld, bOutSearchedPie);
        if (!World) { OutError = TEXT("no world available"); return false; }

        for (TActorIterator<AActor> It(World); It; ++It)
        {
            AActor* A = *It;
            if (!ShouldListActor(A)) continue;
            if (!ClassFilter.IsEmpty())
            {
                // Walk the class hierarchy so a filter like
                // "AMjArticulation" matches BP-derived classes
                // (e.g. golden_scene_C). UE's UClass::GetName() returns
                // the unprefixed form ("MjArticulation"), but most
                // callers write the C++ class name with the A/U/F
                // prefix — accept either form by also comparing
                // against the filter with one leading char stripped.
                const TCHAR FirstChar = ClassFilter.Len() > 1 ? ClassFilter[0] : 0;
                const bool bHasTypePrefix = FirstChar == TEXT('A') || FirstChar == TEXT('U') || FirstChar == TEXT('F');
                const FString FilterStripped = bHasTypePrefix ? ClassFilter.Mid(1) : FString();
                bool bMatch = false;
                for (UClass* C = A->GetClass(); C != nullptr; C = C->GetSuperClass())
                {
                    const FString& N = C->GetName();
                    if (N.Equals(ClassFilter, ESearchCase::IgnoreCase) ||
                        (!FilterStripped.IsEmpty() && N.Equals(FilterStripped, ESearchCase::IgnoreCase)))
                    {
                        bMatch = true;
                        break;
                    }
                }
                if (!bMatch) continue;
            }
            if (!TagFilter.IsEmpty() && !A->Tags.Contains(FName(*TagFilter)))
                continue;
            if (!NamePrefix.IsEmpty() && !A->GetName().StartsWith(NamePrefix))
                continue;
            OutActors.Add(MakeShared<FJsonValueObject>(BuildActorRow(A)));
        }
        return true;
    }

    bool GetActorBoundsSync(
        const FString& ActorKey,
        bool bComponentsOnly,
        double OutMin[3],
        double OutMax[3],
        FString& OutResolvedName,
        FString& OutError)
    {
        OutError.Empty();
        if (!GEditor) { OutError = TEXT("GEditor null"); return false; }
        UWorld* World = GEditor->GetEditorWorldContext().World();
        if (!World) { OutError = TEXT("editor world unavailable"); return false; }
        AActor* A = FindByActorIdOrName(World, ActorKey);
        if (!A)
        {
            OutError = FString::Printf(TEXT("no actor matching '%s'"), *ActorKey);
            return false;
        }
        OutResolvedName = A->GetName();

        const FBox Box = A->GetComponentsBoundingBox(bComponentsOnly);
        // UE cm -> MJ metres, +Y -> -Y flip baked into MjUtils.
        double MjMin[3], MjMax[3];
        MjUtils::UEToMjPosition(Box.Min, MjMin);
        MjUtils::UEToMjPosition(Box.Max, MjMax);
        // The Y-flip swaps min/max on that axis; normalise.
        for (int i = 0; i < 3; ++i)
        {
            OutMin[i] = FMath::Min(MjMin[i], MjMax[i]);
            OutMax[i] = FMath::Max(MjMin[i], MjMax[i]);
        }
        return true;
    }

    bool SnapshotSceneSync(
        TArray<TSharedPtr<FJsonValue>>& OutActors,
        bool& bOutInPie,
        FString& OutLevelPath,
        FString& OutError)
    {
        OutActors.Reset();
        OutError.Empty();
        if (!GEditor) { OutError = TEXT("GEditor null"); return false; }
        UWorld* PieWorld = GEditor->PlayWorld;
        UWorld* World = PieWorld ? PieWorld : GEditor->GetEditorWorldContext().World();
        if (!World) { OutError = TEXT("no world available"); return false; }
        bOutInPie = (World == GEditor->PlayWorld);
        OutLevelPath = World->GetOutermost()->GetName();

        for (TActorIterator<AActor> It(World); It; ++It)
        {
            AActor* A = *It;
            if (!ShouldListActor(A)) continue;
            TSharedPtr<FJsonObject> Row = BuildActorRow(A);

            // URLab-specific metadata: only present for AMjArticulation actors.
            if (AMjArticulation* Mj = Cast<AMjArticulation>(A))
            {
                TSharedPtr<FJsonObject> Urlab = MakeShared<FJsonObject>();
                Urlab->SetStringField(TEXT("mj_class"), TEXT("articulation"));

                TArray<UMjJoint*> Joints;
                Mj->GetComponents<UMjJoint>(Joints);
                TArray<TSharedPtr<FJsonValue>> JointNames;
                for (UMjJoint* J : Joints)
                    if (J && !J->bIsDefault)
                        JointNames.Add(MakeShared<FJsonValueString>(J->GetMjName()));
                Urlab->SetArrayField(TEXT("joints"), JointNames);

                TArray<UMjActuator*> Acts;
                Mj->GetComponents<UMjActuator>(Acts);
                TArray<TSharedPtr<FJsonValue>> ActNames;
                for (UMjActuator* Act : Acts)
                    if (Act && !Act->bIsDefault)
                        ActNames.Add(MakeShared<FJsonValueString>(Act->GetMjName()));
                Urlab->SetArrayField(TEXT("actuators"), ActNames);

                TArray<UMjSensor*> Sensors;
                Mj->GetComponents<UMjSensor>(Sensors);
                TArray<TSharedPtr<FJsonValue>> SensorNames;
                for (UMjSensor* S : Sensors)
                    if (S && !S->bIsDefault)
                        SensorNames.Add(MakeShared<FJsonValueString>(S->GetMjName()));
                Urlab->SetArrayField(TEXT("sensors"), SensorNames);

                TArray<UMjCamera*> Cams;
                Mj->GetComponents<UMjCamera>(Cams);
                TArray<TSharedPtr<FJsonValue>> CamNames;
                for (UMjCamera* C : Cams)
                    if (C && !C->bIsDefault)
                        CamNames.Add(MakeShared<FJsonValueString>(C->GetName()));
                Urlab->SetArrayField(TEXT("cameras"), CamNames);

                Row->SetObjectField(TEXT("urlab"), Urlab);
            }
            OutActors.Add(MakeShared<FJsonValueObject>(Row));
        }
        return true;
    }

    bool DuplicateActorSync(
        const FString& SrcKey,
        const FString& NewActorId,
        bool bHasOverrideLocation,
        const FVector& OverrideLocationMeters,
        FString& OutActorName,
        FString& OutActorPath,
        FString& OutBlueprintClassPath,
        FString& OutError)
    {
        OutError.Empty();
        if (!GEditor) { OutError = TEXT("GEditor null"); return false; }
        UWorld* World = GEditor->GetEditorWorldContext().World();
        if (!World) { OutError = TEXT("editor world unavailable"); return false; }
        AActor* Src = FindByActorIdOrName(World, SrcKey);
        if (!Src)
        {
            OutError = FString::Printf(TEXT("no actor matching '%s'"), *SrcKey);
            return false;
        }

        // MJ metres throughout.
        FVector SourceMjPos;
        {
            double MjPos[3];
            MjUtils::UEToMjPosition(Src->GetActorLocation(), MjPos);
            SourceMjPos = FVector(MjPos[0], MjPos[1], MjPos[2]);
        }
        FQuat   SrcQuatXyzw;
        {
            double MjQuat[4];
            MjUtils::UEToMjRotation(Src->GetActorQuat(), MjQuat);
            // MjUtils emits wxyz; pack as xyzw for SpawnActorSync.
            SrcQuatXyzw = FQuat(MjQuat[1], MjQuat[2], MjQuat[3], MjQuat[0]);
        }
        const FVector SrcScale = Src->GetActorScale3D();

        const FVector SpawnLoc = bHasOverrideLocation
            ? OverrideLocationMeters
            : SourceMjPos + FVector(1.0, 0.0, 0.0);  // default: 1m on +X

        bool bWasExisting = false;
        return SpawnActorSync(
            Src->GetClass()->GetPathName(),
            NewActorId, SpawnLoc, SrcQuatXyzw, SrcScale,
            OutActorName, OutActorPath, OutBlueprintClassPath,
            bWasExisting, OutError);
    }

    bool ActorHierarchySync(
        const FString& ActorKey,
        TSharedPtr<FJsonObject>& OutRoot,
        FString& OutError)
    {
        OutError.Empty();
        if (!GEditor) { OutError = TEXT("GEditor null"); return false; }
        UWorld* PieWorld = GEditor->PlayWorld;
        UWorld* World = PieWorld ? PieWorld : GEditor->GetEditorWorldContext().World();
        if (!World) { OutError = TEXT("no world available"); return false; }
        AActor* A = FindByActorIdOrName(World, ActorKey);
        if (!A)
        {
            OutError = FString::Printf(TEXT("no actor matching '%s'"), *ActorKey);
            return false;
        }

        // Walk via Actor->GetAttachedActors recursively. UE allows actor
        // attachment trees independent of component trees.
        TFunction<TSharedPtr<FJsonObject>(AActor*)> BuildNode;
        BuildNode = [&BuildNode](AActor* N) -> TSharedPtr<FJsonObject>
        {
            TSharedPtr<FJsonObject> Node = MakeShared<FJsonObject>();
            Node->SetStringField(TEXT("name"), N->GetName());
            Node->SetStringField(TEXT("class"), N->GetClass()->GetName());

            double MjPos[3];
            MjUtils::UEToMjPosition(N->GetActorLocation(), MjPos);
            TArray<TSharedPtr<FJsonValue>> LocOut;
            for (int i = 0; i < 3; ++i)
                LocOut.Add(MakeShared<FJsonValueNumber>(MjPos[i]));
            Node->SetArrayField(TEXT("location"), LocOut);

            TArray<AActor*> Attached;
            N->GetAttachedActors(Attached, /*bResetArray=*/true, /*bRecurse=*/false);
            TArray<TSharedPtr<FJsonValue>> Children;
            for (AActor* C : Attached)
                if (C) Children.Add(MakeShared<FJsonValueObject>(BuildNode(C)));
            Node->SetArrayField(TEXT("children"), Children);
            return Node;
        };
        OutRoot = BuildNode(A);
        return true;
    }
}
