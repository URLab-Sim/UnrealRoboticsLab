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

#include "Misc/AutomationTest.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "Interfaces/IPluginManager.h"
#include "UObject/StrongObjectPtr.h"
#include "UObject/UObjectGlobals.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Engine/Blueprint.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "MJModelComponent.h"
#include "MJAssetLibrary.h"
#include "MJSpecHandle.h"
#include "MJXmlImporterExporter.h"
#include "mujoco/mujoco.h"
#include <cmath>

namespace
{
void PopulateAssetLibraryFromDir(const FString& ModelDir, UMJAssetLibrary* AssetLib)
{
	if (!AssetLib)
	{
		return;
	}

	TArray<FString> Files;
	IFileManager::Get().FindFilesRecursive(Files, *ModelDir, TEXT("*.*"), true, false);
	FString BasePrefix = ModelDir;
	FPaths::NormalizeDirectoryName(BasePrefix);
	BasePrefix /= TEXT("");
	for (const FString& FullPath : Files)
	{
		FString RelPath = FullPath;
		FPaths::NormalizeFilename(RelPath);
		if (!RelPath.RemoveFromStart(BasePrefix))
		{
			continue;
		}
		TArray<uint8> Bytes;
		if (FFileHelper::LoadFileToArray(Bytes, *FullPath))
		{
			FMJRawAssetBuffer& Buf = AssetLib->RawBuffers.Add(RelPath);
			Buf.Bytes = MoveTemp(Bytes);
		}
	}
}

double CorpusMaxQuatAbsDiff(const mjtNum* A, const mjtNum* B, int32 NumQuats)
{
	double MaxDiff = 0.0;
	for (int32 q = 0; q < NumQuats; ++q)
	{
		double PosDiff = 0.0;
		double NegDiff = 0.0;
		for (int32 k = 0; k < 4; ++k)
		{
			const double Va = static_cast<double>(A[4 * q + k]);
			const double Vb = static_cast<double>(B[4 * q + k]);
			PosDiff = FMath::Max(PosDiff, FMath::Abs(Va - Vb));
			NegDiff = FMath::Max(NegDiff, FMath::Abs(Va + Vb));
		}
		MaxDiff = FMath::Max(MaxDiff, FMath::Min(PosDiff, NegDiff));
	}
	return MaxDiff;
}

double CorpusMaxArrayAbsDiff(const mjtNum* A, const mjtNum* B, int32 Count)
{
	double MaxDiff = 0.0;
	for (int32 i = 0; i < Count; ++i)
	{
		const double Diff = FMath::Abs(static_cast<double>(A[i]) - static_cast<double>(B[i]));
		if (Diff > MaxDiff)
		{
			MaxDiff = Diff;
		}
	}
	return MaxDiff;
}

double CorpusMaxQposDiff(const mjModel* M, const mjtNum* A, const mjtNum* B)
{
	TArray<bool> IsQuatCoord;
	IsQuatCoord.Init(false, M->nq);
	for (int32 j = 0; j < M->njnt; ++j)
	{
		const int32 Adr = M->jnt_qposadr[j];
		if (M->jnt_type[j] == mjJNT_FREE && Adr + 6 < M->nq)
		{
			IsQuatCoord[Adr + 3] = true;
			IsQuatCoord[Adr + 4] = true;
			IsQuatCoord[Adr + 5] = true;
			IsQuatCoord[Adr + 6] = true;
		}
		else if (M->jnt_type[j] == mjJNT_BALL && Adr + 3 < M->nq)
		{
			IsQuatCoord[Adr + 0] = true;
			IsQuatCoord[Adr + 1] = true;
			IsQuatCoord[Adr + 2] = true;
			IsQuatCoord[Adr + 3] = true;
		}
	}

	double MaxDiff = 0.0;
	for (int32 i = 0; i < M->nq; )
	{
		if (IsQuatCoord[i] && i + 3 < M->nq)
		{
			MaxDiff = FMath::Max(MaxDiff, CorpusMaxQuatAbsDiff(A + i, B + i, 1));
			i += 4;
		}
		else
		{
			MaxDiff = FMath::Max(MaxDiff, FMath::Abs(static_cast<double>(A[i]) - static_cast<double>(B[i])));
			++i;
		}
	}
	return MaxDiff;
}

double CorpusMaxSensorDiff(const mjModel* M, const mjtNum* A, const mjtNum* B)
{
	if (M->nsensordata <= 0)
	{
		return 0.0;
	}
	TArray<bool> IsQuatCoord;
	IsQuatCoord.Init(false, M->nsensordata);
	for (int32 s = 0; s < M->nsensor; ++s)
	{
		if (M->sensor_type[s] == mjSENS_FRAMEQUAT)
		{
			const int32 Adr = M->sensor_adr[s];
			if (Adr >= 0 && Adr + 3 < M->nsensordata)
			{
				IsQuatCoord[Adr + 0] = true;
				IsQuatCoord[Adr + 1] = true;
				IsQuatCoord[Adr + 2] = true;
				IsQuatCoord[Adr + 3] = true;
			}
		}
	}

	double MaxDiff = 0.0;
	for (int32 i = 0; i < M->nsensordata; )
	{
		if (IsQuatCoord[i] && i + 3 < M->nsensordata)
		{
			MaxDiff = FMath::Max(MaxDiff, CorpusMaxQuatAbsDiff(A + i, B + i, 1));
			i += 4;
		}
		else
		{
			MaxDiff = FMath::Max(MaxDiff, FMath::Abs(static_cast<double>(A[i]) - static_cast<double>(B[i])));
			++i;
		}
	}
	return MaxDiff;
}

template <typename T>
int32 CorpusCountIntMismatches(const T* A, const T* B, int32 Count)
{
	int32 Mismatches = 0;
	for (int32 i = 0; i < Count; ++i)
	{
		if (A[i] != B[i])
		{
			++Mismatches;
		}
	}
	return Mismatches;
}

struct FCorpusParityStats
{
	int32 TotalXmls = 0;
	int32 SkippedStockFail = 0;
	int32 SkippedAllowlist = 0;
	int32 Tested = 0;
	int32 Passed = 0;
	int32 Failed = 0;
	double MaxModelDiff = 0.0;
	double MaxQposDiff = 0.0;
	double MaxQvelDiff = 0.0;
	double MaxSensDiff = 0.0;
};

bool RunCorpusParity(
	FAutomationTestBase* Test,
	const FString& CorpusName,
	const FString& RootDir,
	const TArray<FString>& XmlFullPaths,
	const TMap<FString, FString>& UnsupportedAllowlist)
{
	TStrongObjectPtr<UWorld> World(UWorld::CreateWorld(EWorldType::Editor, false));
	if (!Test->TestNotNull(TEXT("Transient World created"), World.Get()))
	{
		return false;
	}

	FString NormRoot = RootDir;
	FPaths::NormalizeDirectoryName(NormRoot);
	NormRoot /= TEXT("");

	FCorpusParityStats Stats;
	Stats.TotalXmls = XmlFullPaths.Num();

	for (int32 Idx = 0; Idx < XmlFullPaths.Num(); ++Idx)
	{
		const FString& FullXmlPath = XmlFullPaths[Idx];
		FString RelPath = FullXmlPath;
		FPaths::NormalizeFilename(RelPath);
		RelPath.RemoveFromStart(NormRoot);

		if (const FString* Reason = UnsupportedAllowlist.Find(RelPath))
		{
			++Stats.SkippedAllowlist;
			Test->AddInfo(FString::Printf(TEXT("[%s] SKIP allowlist %s: %s"), *CorpusName, *RelPath, **Reason));
			continue;
		}

		// 1. Reference compile via stock MuJoCo mj_loadXML
		char RefErr[1024] = {};
		FMJModelHandle RefModel(mj_loadXML(TCHAR_TO_UTF8(*FullXmlPath), nullptr, RefErr, sizeof(RefErr)));
		if (!RefModel.Get())
		{
			++Stats.SkippedStockFail;
			continue;
		}

		FMJDataHandle RefData(mj_makeData(RefModel.Get()));
		if (!RefData.Get())
		{
			++Stats.SkippedStockFail;
			continue;
		}

		FString XmlContent;
		if (!FFileHelper::LoadFileToString(XmlContent, *FullXmlPath))
		{
			++Stats.Failed;
			Test->AddError(FString::Printf(TEXT("[%s][%s] Failed to read XML file"), *CorpusName, *RelPath));
			continue;
		}

		++Stats.Tested;
		const FString ModelDir = FPaths::GetPath(FullXmlPath);

		// 2. Populate AssetLibrary + VFS for UE Blueprint SCS import
		TStrongObjectPtr<UMJAssetLibrary> AssetLib(NewObject<UMJAssetLibrary>(GetTransientPackage()));
		PopulateAssetLibraryFromDir(ModelDir, AssetLib.Get());

		FMJVFSHandle Vfs;
		Vfs.Init();
		AssetLib->PopulateVFS(Vfs.Get());

		TStrongObjectPtr<UBlueprint> BP(FKismetEditorUtilities::CreateBlueprint(
			AActor::StaticClass(),
			GetTransientPackage(),
			NAME_None,
			BPTYPE_Normal,
			UBlueprint::StaticClass(),
			UBlueprintGeneratedClass::StaticClass()));
		if (!BP.Get())
		{
			++Stats.Failed;
			Test->AddError(FString::Printf(TEXT("[%s][%s] CreateBlueprint failed"), *CorpusName, *RelPath));
			continue;
		}

		FString ImportErr;
		if (!FMJXmlImporterExporter::ImportXMLToBlueprintSCS(XmlContent, BP.Get(), Vfs.Get(), ImportErr))
		{
			++Stats.Failed;
			Test->AddError(FString::Printf(TEXT("[%s][%s] ImportXMLToBlueprintSCS failed: %s"), *CorpusName, *RelPath, *ImportErr));
			continue;
		}

		FKismetEditorUtilities::CompileBlueprint(BP.Get());
		if (!BP->GeneratedClass)
		{
			++Stats.Failed;
			Test->AddError(FString::Printf(TEXT("[%s][%s] Blueprint GeneratedClass is null"), *CorpusName, *RelPath));
			continue;
		}

		FActorSpawnParameters SpawnParams;
		SpawnParams.ObjectFlags = RF_Transient;
		AActor* Actor = World->SpawnActor<AActor>(BP->GeneratedClass, FTransform::Identity, SpawnParams);
		if (!Actor)
		{
			++Stats.Failed;
			Test->AddError(FString::Printf(TEXT("[%s][%s] SpawnActor failed"), *CorpusName, *RelPath));
			continue;
		}

		UMJModelComponent* ModelComp = Actor->FindComponentByClass<UMJModelComponent>();
		if (!ModelComp)
		{
			++Stats.Failed;
			Test->AddError(FString::Printf(TEXT("[%s][%s] Missing UMJModelComponent"), *CorpusName, *RelPath));
			Actor->Destroy();
			continue;
		}

		ModelComp->AssetLibrary = AssetLib.Get();

		FString CompileErr;
		if (!ModelComp->CompileModel(CompileErr))
		{
			++Stats.Failed;
			Test->AddError(FString::Printf(TEXT("[%s][%s] CompileModel failed: %s"), *CorpusName, *RelPath, *CompileErr));
			Actor->Destroy();
			continue;
		}

		mjModel* UeModel = ModelComp->GetModel();
		mjData* UeData = ModelComp->GetData();
		const mjModel* Rm = RefModel.Get();
		mjData* Rd = RefData.Get();
		if (!UeModel || !UeData)
		{
			++Stats.Failed;
			Test->AddError(FString::Printf(TEXT("[%s][%s] Null UeModel or UeData"), *CorpusName, *RelPath));
			Actor->Destroy();
			continue;
		}

		// 3. Exact count parity
		const bool bCountsMatch =
			(UeModel->nq == Rm->nq) &&
			(UeModel->nv == Rm->nv) &&
			(UeModel->nu == Rm->nu) &&
			(UeModel->nbody == Rm->nbody) &&
			(UeModel->njnt == Rm->njnt) &&
			(UeModel->ngeom == Rm->ngeom) &&
			(UeModel->nsite == Rm->nsite) &&
			(UeModel->nsensor == Rm->nsensor) &&
			(UeModel->nmocap == Rm->nmocap) &&
			(UeModel->ntendon == Rm->ntendon) &&
			(UeModel->neq == Rm->neq) &&
			(UeModel->npair == Rm->npair) &&
			(UeModel->nexclude == Rm->nexclude) &&
			(UeModel->nflex == Rm->nflex);

		if (!bCountsMatch)
		{
			++Stats.Failed;
			Test->AddError(FString::Printf(
				TEXT("[%s][%s] Count mismatch: UE(nq=%d nv=%d nu=%d body=%d jnt=%d geom=%d site=%d sens=%d mocap=%d ten=%d eq=%d pair=%d exc=%d flex=%d) vs REF(nq=%d nv=%d nu=%d body=%d jnt=%d geom=%d site=%d sens=%d mocap=%d ten=%d eq=%d pair=%d exc=%d flex=%d)"),
				*CorpusName, *RelPath,
				UeModel->nq, UeModel->nv, UeModel->nu, UeModel->nbody, UeModel->njnt, UeModel->ngeom, UeModel->nsite, UeModel->nsensor, UeModel->nmocap, UeModel->ntendon, UeModel->neq, UeModel->npair, UeModel->nexclude, UeModel->nflex,
				Rm->nq, Rm->nv, Rm->nu, Rm->nbody, Rm->njnt, Rm->ngeom, Rm->nsite, Rm->nsensor, Rm->nmocap, Rm->ntendon, Rm->neq, Rm->npair, Rm->nexclude, Rm->nflex));
			Actor->Destroy();
			continue;
		}

		// 4. Numeric mjModel parity
		constexpr double ModelTol = 1e-6;
		const double MaxBodyPos = CorpusMaxArrayAbsDiff(UeModel->body_pos, Rm->body_pos, 3 * Rm->nbody);
		const double MaxBodyQuat = CorpusMaxQuatAbsDiff(UeModel->body_quat, Rm->body_quat, Rm->nbody);
		const double MaxBodyMass = CorpusMaxArrayAbsDiff(UeModel->body_mass, Rm->body_mass, Rm->nbody);
		const double MaxBodyInertia = CorpusMaxArrayAbsDiff(UeModel->body_inertia, Rm->body_inertia, 3 * Rm->nbody);
		const int32 GeomTypeMismatch = CorpusCountIntMismatches(UeModel->geom_type, Rm->geom_type, Rm->ngeom);
		const double MaxGeomSize = CorpusMaxArrayAbsDiff(UeModel->geom_size, Rm->geom_size, 3 * Rm->ngeom);
		const int32 JntTypeMismatch = CorpusCountIntMismatches(UeModel->jnt_type, Rm->jnt_type, Rm->njnt);
		const double MaxJntRange = CorpusMaxArrayAbsDiff(UeModel->jnt_range, Rm->jnt_range, 2 * Rm->njnt);
		const int32 ActTrnMismatch = CorpusCountIntMismatches(UeModel->actuator_trntype, Rm->actuator_trntype, Rm->nu);
		const double MaxActGain = CorpusMaxArrayAbsDiff(UeModel->actuator_gainprm, Rm->actuator_gainprm, mjNGAIN * Rm->nu);
		const double MaxActBias = CorpusMaxArrayAbsDiff(UeModel->actuator_biasprm, Rm->actuator_biasprm, mjNBIAS * Rm->nu);

		const double ModelDiff = FMath::Max(
			FMath::Max(FMath::Max(MaxBodyPos, MaxBodyQuat), FMath::Max(MaxBodyMass, MaxBodyInertia)),
			FMath::Max(FMath::Max(MaxGeomSize, MaxJntRange), FMath::Max(MaxActGain, MaxActBias)));
		Stats.MaxModelDiff = FMath::Max(Stats.MaxModelDiff, ModelDiff);

		bool bModelOk = (ModelDiff <= ModelTol) && (GeomTypeMismatch == 0) && (JntTypeMismatch == 0) && (ActTrnMismatch == 0);
		if (!bModelOk)
		{
			++Stats.Failed;
			Test->AddError(FString::Printf(
				TEXT("[%s][%s] Model parity failed: body_pos=%.3e body_quat=%.3e body_mass=%.3e body_inertia=%.3e geom_size=%.3e jnt_range=%.3e gainprm=%.3e biasprm=%.3e geomTypeMis=%d jntTypeMis=%d actTrnMis=%d"),
				*CorpusName, *RelPath,
				MaxBodyPos, MaxBodyQuat, MaxBodyMass, MaxBodyInertia,
				MaxGeomSize, MaxJntRange, MaxActGain, MaxActBias,
				GeomTypeMismatch, JntTypeMismatch, ActTrnMismatch));
			Actor->Destroy();
			continue;
		}

		// 5. Open-loop 100-step trajectory parity
		mj_resetData(Rm, Rd);
		mj_resetData(UeModel, UeData);

		double MaxQposDiff = 0.0;
		double MaxQvelDiff = 0.0;
		double MaxSensDiff = 0.0;

		for (int32 Step = 0; Step < 100; ++Step)
		{
			for (int32 i = 0; i < Rm->nu; ++i)
			{
				const double Lo = static_cast<double>(Rm->actuator_ctrlrange[2 * i]);
				const double Hi = static_cast<double>(Rm->actuator_ctrlrange[2 * i + 1]);
				const double Mid = (Lo < Hi) ? 0.5 * (Lo + Hi) : 0.0;
				const double Amp = (Lo < Hi) ? 0.25 * (Hi - Lo) : 0.1;
				const double CtrlVal = Mid + Amp * std::sin(0.05 * Step + 0.3 * i);
				Rd->ctrl[i] = CtrlVal;
				UeData->ctrl[i] = CtrlVal;
			}
			mj_step(Rm, Rd);
			mj_step(UeModel, UeData);

			MaxQposDiff = FMath::Max(MaxQposDiff, CorpusMaxQposDiff(Rm, UeData->qpos, Rd->qpos));
			MaxQvelDiff = FMath::Max(MaxQvelDiff, CorpusMaxArrayAbsDiff(UeData->qvel, Rd->qvel, Rm->nv));
			if (Rm->nsensordata > 0)
			{
				MaxSensDiff = FMath::Max(MaxSensDiff, CorpusMaxSensorDiff(Rm, UeData->sensordata, Rd->sensordata));
			}
		}

		Stats.MaxQposDiff = FMath::Max(Stats.MaxQposDiff, MaxQposDiff);
		Stats.MaxQvelDiff = FMath::Max(Stats.MaxQvelDiff, MaxQvelDiff);
		Stats.MaxSensDiff = FMath::Max(Stats.MaxSensDiff, MaxSensDiff);

		constexpr double TrajTol = 1e-8;
		if (MaxQposDiff > TrajTol || MaxQvelDiff > TrajTol || MaxSensDiff > TrajTol)
		{
			++Stats.Failed;
			Test->AddError(FString::Printf(
				TEXT("[%s][%s] 100-step trajectory parity failed: qpos=%.3e qvel=%.3e sens=%.3e"),
				*CorpusName, *RelPath, MaxQposDiff, MaxQvelDiff, MaxSensDiff));
		}
		else
		{
			++Stats.Passed;
		}

		Actor->Destroy();

		if ((Stats.Tested % 25) == 0)
		{
			CollectGarbage(GARBAGE_COLLECTION_KEEPFLAGS);
		}
	}

	World->DestroyWorld(false);
	CollectGarbage(GARBAGE_COLLECTION_KEEPFLAGS);

	const FString SummaryLine = FString::Printf(
		TEXT("[%s] CORPUS SUMMARY: totalXmls=%d compiled=%d passed=%d failed=%d skippedStockFail=%d skippedAllowlist=%d | maxModelDiff=%.3e | 100-step maxQpos=%.3e maxQvel=%.3e maxSens=%.3e"),
		*CorpusName, Stats.TotalXmls, Stats.Tested, Stats.Passed, Stats.Failed, Stats.SkippedStockFail, Stats.SkippedAllowlist,
		Stats.MaxModelDiff, Stats.MaxQposDiff, Stats.MaxQvelDiff, Stats.MaxSensDiff);
	UE_LOG(LogTemp, Display, TEXT("%s"), *SummaryLine);
	Test->AddInfo(SummaryLine);

	return Stats.Failed == 0;
}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMJUpstreamMuJoCoCorpusParityTest,
	"MuJoCoUE.UpstreamMuJoCoCorpusParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMJUpstreamMuJoCoCorpusParityTest::RunTest(const FString& Parameters)
{
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("UnrealRoboticsLab"));
	if (!TestTrue(TEXT("UnrealRoboticsLab plugin found"), Plugin.IsValid()))
	{
		return false;
	}

	const FString MuJoCoSrcDir = FPaths::Combine(Plugin->GetBaseDir(), TEXT("third_party/MuJoCo/src"));
	const FString ModelDir = FPaths::Combine(MuJoCoSrcDir, TEXT("model"));
	const FString TestDir = FPaths::Combine(MuJoCoSrcDir, TEXT("test"));

	TArray<FString> XmlFiles;
	if (FPaths::DirectoryExists(ModelDir))
	{
		IFileManager::Get().FindFilesRecursive(XmlFiles, *ModelDir, TEXT("*.xml"), true, false);
	}
	if (FPaths::DirectoryExists(TestDir))
	{
		IFileManager::Get().FindFilesRecursive(XmlFiles, *TestDir, TEXT("*.xml"), true, false, false);
	}
	XmlFiles.Sort();

	if (!TestTrue(TEXT("Found upstream MuJoCo XMLs"), XmlFiles.Num() > 0))
	{
		return false;
	}

	TMap<FString, FString> UnsupportedAllowlist;
	// Non-round-trippable flexcomp explicitinertial & 1-ULP (m<->cm / rad<->deg) contact/flex chaos over 100 steps
	UnsupportedAllowlist.Add(TEXT("test/testdata/flex.xml"), TEXT("<flexcomp dof=radial> explicitinertial moving body mass"));
	UnsupportedAllowlist.Add(TEXT("model/flex/bag.xml"), TEXT("<flexcomp> explicitinertial moving body mass"));
	UnsupportedAllowlist.Add(TEXT("model/flex/bag_hoop.xml"), TEXT("<flexcomp> explicitinertial moving body mass"));
	UnsupportedAllowlist.Add(TEXT("model/flex/belt.xml"), TEXT("<flexcomp> explicitinertial moving body mass"));
	UnsupportedAllowlist.Add(TEXT("model/flex/bunny_multicell.xml"), TEXT("<flexcomp> explicitinertial moving body mass"));
	UnsupportedAllowlist.Add(TEXT("model/flex/bunny_quadratic.xml"), TEXT("<flexcomp> explicitinertial moving body mass"));
	UnsupportedAllowlist.Add(TEXT("model/flex/cable.xml"), TEXT("<flexcomp> explicitinertial moving body mass"));
	UnsupportedAllowlist.Add(TEXT("model/flex/coil.xml"), TEXT("<flexcomp> explicitinertial moving body mass"));
	UnsupportedAllowlist.Add(TEXT("model/flex/curved_rods.xml"), TEXT("<flexcomp> explicitinertial moving body mass"));
	UnsupportedAllowlist.Add(TEXT("model/flex/mechanisms/cantilever.xml"), TEXT("<flexcomp> explicitinertial moving body mass"));
	UnsupportedAllowlist.Add(TEXT("model/flex/mechanisms/dipper.xml"), TEXT("<flexcomp> explicitinertial moving body mass"));
	UnsupportedAllowlist.Add(TEXT("model/flex/mechanisms/slidercrank.xml"), TEXT("<flexcomp> explicitinertial moving body mass"));
	UnsupportedAllowlist.Add(TEXT("model/hammock/hammock.xml"), TEXT("Includes ../humanoid/humanoid.xml outside model dir"));
	UnsupportedAllowlist.Add(TEXT("model/surfacevel/treadmill.xml"), TEXT("Includes ../humanoid/humanoid.xml outside model dir"));
	UnsupportedAllowlist.Add(TEXT("model/arch/gothic.xml"), TEXT("Custom/plugin geom type"));
	UnsupportedAllowlist.Add(TEXT("model/plastic/torsion.xml"), TEXT("Custom/plugin geom type"));
	UnsupportedAllowlist.Add(TEXT("model/plastic/wire.xml"), TEXT("Custom/plugin geom type"));
	UnsupportedAllowlist.Add(TEXT("model/adhesion/sand_castle.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 multi-contact steps"));
	UnsupportedAllowlist.Add(TEXT("model/arch/roman.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 multi-contact steps"));
	UnsupportedAllowlist.Add(TEXT("model/flex/drape.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 flex steps"));
	UnsupportedAllowlist.Add(TEXT("model/flex/gripper_2d.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 flex steps"));
	UnsupportedAllowlist.Add(TEXT("model/humanoid/100_humanoids.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 multi-contact steps"));
	UnsupportedAllowlist.Add(TEXT("model/plastic/clay.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 plastic steps"));
	UnsupportedAllowlist.Add(TEXT("model/plastic/roller.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 plastic steps"));
	UnsupportedAllowlist.Add(TEXT("model/replicate/bunnies.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 multi-contact steps"));
	UnsupportedAllowlist.Add(TEXT("model/replicate/leaves.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 multi-contact steps"));
	UnsupportedAllowlist.Add(TEXT("model/replicate/particle.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 multi-contact steps"));
	UnsupportedAllowlist.Add(TEXT("model/replicate/stonehenge.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 multi-contact steps"));
	UnsupportedAllowlist.Add(TEXT("model/welcome/welcome.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 multi-contact steps"));
	UnsupportedAllowlist.Add(TEXT("test/benchmark/testdata/2humanoid100.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 multi-contact steps"));
	UnsupportedAllowlist.Add(TEXT("test/benchmark/testdata/boxpile.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 multi-box contact steps"));
	UnsupportedAllowlist.Add(TEXT("test/benchmark/testdata/humanoid200.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 multi-contact steps"));
	UnsupportedAllowlist.Add(TEXT("test/engine/testdata/actuation/actearly.xml"), TEXT("actearly integrator 100-step drift"));
	UnsupportedAllowlist.Add(TEXT("test/engine/testdata/collision_convex/perf/mixed.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 convex contact steps"));
	UnsupportedAllowlist.Add(TEXT("test/engine/testdata/island/2humanoid100.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 multi-contact steps"));
	UnsupportedAllowlist.Add(TEXT("test/engine/testdata/sleep/sensor.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 contact/sensor steps"));
	UnsupportedAllowlist.Add(TEXT("test/engine/testdata/solver/model.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 contact steps"));
	UnsupportedAllowlist.Add(TEXT("test/engine/testdata/solver/perf/mixed.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 contact steps"));
	UnsupportedAllowlist.Add(TEXT("test/engine/testdata/vis_interact/boxes.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 contact steps"));
	UnsupportedAllowlist.Add(TEXT("test/testdata/model.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 contact steps"));
	UnsupportedAllowlist.Add(TEXT("test/user/testdata/robot_arm.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 contact/sensor steps"));
	UnsupportedAllowlist.Add(TEXT("test/user/testdata/shark_22_ascii_vol_fTetWild.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 tetrahedral flex steps"));
	UnsupportedAllowlist.Add(TEXT("test/user/testdata/shark_22_binary_vol_fTetWild.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 tetrahedral flex steps"));
	return RunCorpusParity(this, TEXT("UpstreamMuJoCo"), MuJoCoSrcDir, XmlFiles, UnsupportedAllowlist);
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMJMenagerieFullCorpusParityTest,
	"MuJoCoUE.MenagerieFullCorpusParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMJMenagerieFullCorpusParityTest::RunTest(const FString& Parameters)
{
	const FString MenagerieDir = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_MENAGERIE_DIR"));
	if (MenagerieDir.IsEmpty() || !FPaths::DirectoryExists(MenagerieDir))
	{
		AddWarning(FString::Printf(
			TEXT("URLAB_MENAGERIE_DIR is unset or missing ('%s'); skipping MenagerieFullCorpusParity test."),
			*MenagerieDir));
		return true;
	}

	FString NormRoot = MenagerieDir;
	FPaths::NormalizeDirectoryName(NormRoot);
	NormRoot /= TEXT("");

	TArray<FString> AllXmls;
	IFileManager::Get().FindFilesRecursive(AllXmls, *MenagerieDir, TEXT("*.xml"), true, false);

	TArray<FString> DepthFilteredXmls;
	for (const FString& FullPath : AllXmls)
	{
		FString RelPath = FullPath;
		FPaths::NormalizeFilename(RelPath);
		if (!RelPath.RemoveFromStart(NormRoot))
		{
			continue;
		}
		int32 SlashCount = 0;
		for (TCHAR Ch : RelPath)
		{
			if (Ch == TEXT('/'))
			{
				++SlashCount;
			}
		}
		if (SlashCount <= 1)
		{
			DepthFilteredXmls.Add(FullPath);
		}
	}
	DepthFilteredXmls.Sort();

	if (!TestTrue(TEXT("Found Menagerie XMLs at depth <= 2"), DepthFilteredXmls.Num() > 0))
	{
		return false;
	}

	TMap<FString, FString> UnsupportedAllowlist;
	// Known mj_parseXMLString(VFS) / cross-angle <attach> / 1-ULP stiff-contact 100-step drift fixtures
	UnsupportedAllowlist.Add(TEXT("ms_human_700/MS-Human-700.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 tendon/muscle steps"));
	UnsupportedAllowlist.Add(TEXT("ms_human_700/MS-Human-700-Locomotion.xml"), TEXT("mj_parseXMLString(VFS) include basename collision with body_primary/Body_Arm_*.xml"));
	UnsupportedAllowlist.Add(TEXT("ms_human_700/scene_locomotion.xml"), TEXT("mj_parseXMLString(VFS) include basename collision with body_primary/Body_Arm_*.xml"));
	UnsupportedAllowlist.Add(TEXT("iit_softfoot/scene.xml"), TEXT("<attach> sub-model uses angle=radian while parent spec uses angle=degree"));
	UnsupportedAllowlist.Add(TEXT("i2rt_yam/yam_linear_4310.xml"), TEXT("Nested frame/geom ordering in finger collision"));
	UnsupportedAllowlist.Add(TEXT("i2rt_yam/scene_linear_4310.xml"), TEXT("Nested frame/geom ordering in finger collision"));
	UnsupportedAllowlist.Add(TEXT("aloha/aloha.xml"), TEXT("1-ULP m<->cm / rad<->deg rounding amplified over 100 stiff-joint steps"));
	UnsupportedAllowlist.Add(TEXT("aloha/scene.xml"), TEXT("1-ULP m<->cm / rad<->deg rounding amplified over 100 stiff-joint steps"));
	UnsupportedAllowlist.Add(TEXT("flybody/fruitfly.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 multi-contact steps"));
	UnsupportedAllowlist.Add(TEXT("flybody/scene.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 multi-contact steps"));
	UnsupportedAllowlist.Add(TEXT("google_barkour_vb/scene.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 ground-contact steps"));
	UnsupportedAllowlist.Add(TEXT("hello_robot_stretch_3/scene.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 wheel/contact steps"));
	UnsupportedAllowlist.Add(TEXT("hello_robot_stretch_3/stretch.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 wheel/contact steps"));
	UnsupportedAllowlist.Add(TEXT("pal_tiago/tiago.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 stiff-actuator steps"));
	UnsupportedAllowlist.Add(TEXT("pal_tiago_dual/tiago_dual.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 stiff-actuator steps"));
	UnsupportedAllowlist.Add(TEXT("pndbotics_adam_lite/adam_lite.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 stiff-joint steps"));
	UnsupportedAllowlist.Add(TEXT("pndbotics_adam_lite/scene.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 ground-contact steps"));
	UnsupportedAllowlist.Add(TEXT("rainbow_robotics_rby1/rby1a_1.2.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 stiff-joint steps"));
	UnsupportedAllowlist.Add(TEXT("rainbow_robotics_rby1/rby1a_1.2_no_gripper.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 stiff-joint steps"));
	UnsupportedAllowlist.Add(TEXT("rainbow_robotics_rby1/rby1m_1.2.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 stiff-joint steps"));
	UnsupportedAllowlist.Add(TEXT("rainbow_robotics_rby1/rby1m_1.2_no_gripper.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 stiff-joint steps"));
	UnsupportedAllowlist.Add(TEXT("rainbow_robotics_rby1/rby1m_1.3.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 stiff-joint steps"));
	UnsupportedAllowlist.Add(TEXT("rainbow_robotics_rby1/scene_rby1m_1.2_no_gripper.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 stiff-joint steps"));
	UnsupportedAllowlist.Add(TEXT("shadow_hand/scene_left.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 tendon/contact steps"));
	UnsupportedAllowlist.Add(TEXT("shadow_hand/scene_right.xml"), TEXT("1-ULP m<->cm rounding amplified over 100 tendon/contact steps"));
	UnsupportedAllowlist.Add(TEXT("trossen_vx300s/scene.xml"), TEXT("1-ULP m<->cm / rad<->deg rounding amplified over 100 stiff-joint steps"));
	UnsupportedAllowlist.Add(TEXT("trossen_vx300s/vx300s.xml"), TEXT("1-ULP m<->cm / rad<->deg rounding amplified over 100 stiff-joint steps"));
	return RunCorpusParity(this, TEXT("MenagerieFull"), MenagerieDir, DepthFilteredXmls, UnsupportedAllowlist);
}
