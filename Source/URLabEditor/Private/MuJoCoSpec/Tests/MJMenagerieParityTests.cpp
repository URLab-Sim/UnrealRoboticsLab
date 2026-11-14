#include "Misc/AutomationTest.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "UObject/StrongObjectPtr.h"
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
void PopulateAssetLibraryFromDirectory(const FString& ModelDir, UMJAssetLibrary* AssetLib)
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

double MaxQuatArrayAbsDiff(const mjtNum* A, const mjtNum* B, int32 NumQuats)
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

double MaxArrayAbsDiff(const mjtNum* A, const mjtNum* B, int32 Count)
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

int32 CountIntMismatches(const int* A, const int* B, int32 Count)
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
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMJMenagerieParityTest,
	"MuJoCoUE.MenagerieParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMJMenagerieParityTest::RunTest(const FString& Parameters)
{
	const FString MenagerieDir = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_MENAGERIE_DIR"));
	if (MenagerieDir.IsEmpty() || !FPaths::DirectoryExists(MenagerieDir))
	{
		AddWarning(FString::Printf(
			TEXT("URLAB_MENAGERIE_DIR is unset or missing ('%s'); skipping Menagerie parity test."),
			*MenagerieDir));
		return true;
	}

	const TArray<FString> ModelRelPaths = {
		TEXT("franka_emika_panda/panda.xml"),
		TEXT("universal_robots_ur5e/ur5e.xml"),
		TEXT("unitree_go2/go2.xml"),
		TEXT("skydio_x2/x2.xml"),
		TEXT("rethink_robotics_sawyer/sawyer.xml"),
		TEXT("shadow_hand/right_hand.xml"),
	};

	TStrongObjectPtr<UWorld> World(UWorld::CreateWorld(EWorldType::Editor, false));
	if (!TestNotNull(TEXT("Transient World created"), World.Get()))
	{
		return false;
	}

	for (const FString& RelXmlPath : ModelRelPaths)
	{
		const FString FullXmlPath = FPaths::Combine(MenagerieDir, RelXmlPath);
		const FString ModelDir = FPaths::GetPath(FullXmlPath);
		FString XmlContent;
		if (!TestTrue(FString::Printf(TEXT("Read %s"), *RelXmlPath), FFileHelper::LoadFileToString(XmlContent, *FullXmlPath)))
		{
			continue;
		}

		// 1. Reference compile via stock MuJoCo mj_loadXML
		char RefErr[1024] = {};
		FMJModelHandle RefModel(mj_loadXML(TCHAR_TO_UTF8(*FullXmlPath), nullptr, RefErr, sizeof(RefErr)));
		if (!TestTrue(FString::Printf(TEXT("[%s] Stock mj_loadXML: %s"), *RelXmlPath, UTF8_TO_TCHAR(RefErr)), RefModel.Get() != nullptr))
		{
			continue;
		}
		FMJDataHandle RefData(mj_makeData(RefModel.Get()));
		if (!TestTrue(FString::Printf(TEXT("[%s] Ref mj_makeData"), *RelXmlPath), RefData.Get() != nullptr))
		{
			continue;
		}

		// 2. Populate AssetLibrary + VFS for UE Blueprint SCS import
		TStrongObjectPtr<UMJAssetLibrary> AssetLib(NewObject<UMJAssetLibrary>(GetTransientPackage()));
		PopulateAssetLibraryFromDirectory(ModelDir, AssetLib.Get());

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
		if (!TestNotNull(FString::Printf(TEXT("[%s] CreateBlueprint"), *RelXmlPath), BP.Get()))
		{
			continue;
		}

		FString ImportErr;
		const bool bImported = FMJXmlImporterExporter::ImportXMLToBlueprintSCS(XmlContent, BP.Get(), Vfs.Get(), ImportErr);
		if (!TestTrue(FString::Printf(TEXT("[%s] ImportXMLToBlueprintSCS: %s"), *RelXmlPath, *ImportErr), bImported))
		{
			continue;
		}

		FKismetEditorUtilities::CompileBlueprint(BP.Get());
		if (!TestNotNull(FString::Printf(TEXT("[%s] BP->GeneratedClass"), *RelXmlPath), BP->GeneratedClass.Get()))
		{
			continue;
		}

		AActor* Actor = World->SpawnActor<AActor>(BP->GeneratedClass);
		if (!TestNotNull(FString::Printf(TEXT("[%s] SpawnActor from BP"), *RelXmlPath), Actor))
		{
			continue;
		}

		UMJModelComponent* ModelComp = Actor->FindComponentByClass<UMJModelComponent>();
		if (!TestNotNull(FString::Printf(TEXT("[%s] Find UMJModelComponent"), *RelXmlPath), ModelComp))
		{
			continue;
		}

		ModelComp->AssetLibrary = AssetLib.Get();

		FString CompileErr;
		const bool bCompiled = ModelComp->CompileModel(CompileErr);
		if (!TestTrue(FString::Printf(TEXT("[%s] UMJModelComponent::CompileModel: %s"), *RelXmlPath, *CompileErr), bCompiled))
		{
			Actor->Destroy();
			continue;
		}

		const mjModel* UeModel = ModelComp->GetModel();
		mjData* UeData = ModelComp->GetData();
		const mjModel* Rm = RefModel.Get();
		mjData* Rd = RefData.Get();

		// 3. Structural mjModel parity
		TestEqual(FString::Printf(TEXT("[%s] nq"), *RelXmlPath), static_cast<int32>(UeModel->nq), static_cast<int32>(Rm->nq));
		TestEqual(FString::Printf(TEXT("[%s] nv"), *RelXmlPath), static_cast<int32>(UeModel->nv), static_cast<int32>(Rm->nv));
		TestEqual(FString::Printf(TEXT("[%s] nu"), *RelXmlPath), static_cast<int32>(UeModel->nu), static_cast<int32>(Rm->nu));
		TestEqual(FString::Printf(TEXT("[%s] nbody"), *RelXmlPath), static_cast<int32>(UeModel->nbody), static_cast<int32>(Rm->nbody));
		TestEqual(FString::Printf(TEXT("[%s] njnt"), *RelXmlPath), static_cast<int32>(UeModel->njnt), static_cast<int32>(Rm->njnt));
		TestEqual(FString::Printf(TEXT("[%s] ngeom"), *RelXmlPath), static_cast<int32>(UeModel->ngeom), static_cast<int32>(Rm->ngeom));
		TestEqual(FString::Printf(TEXT("[%s] nsite"), *RelXmlPath), static_cast<int32>(UeModel->nsite), static_cast<int32>(Rm->nsite));
		TestEqual(FString::Printf(TEXT("[%s] nsensor"), *RelXmlPath), static_cast<int32>(UeModel->nsensor), static_cast<int32>(Rm->nsensor));
		TestEqual(FString::Printf(TEXT("[%s] nmocap"), *RelXmlPath), static_cast<int32>(UeModel->nmocap), static_cast<int32>(Rm->nmocap));

		const bool bCountsMatch =
			UeModel->nq == Rm->nq &&
			UeModel->nv == Rm->nv &&
			UeModel->nu == Rm->nu &&
			UeModel->nbody == Rm->nbody &&
			UeModel->njnt == Rm->njnt &&
			UeModel->ngeom == Rm->ngeom &&
			UeModel->nsite == Rm->nsite &&
			UeModel->nsensor == Rm->nsensor &&
			UeModel->nmocap == Rm->nmocap;

		if (!bCountsMatch)
		{
			AddError(FString::Printf(
				TEXT("[%s] Count mismatch: UE(nq=%d nv=%d nu=%d nbody=%d njnt=%d ngeom=%d nsite=%d nsens=%d nmocap=%d) vs Ref(nq=%d nv=%d nu=%d nbody=%d njnt=%d ngeom=%d nsite=%d nsens=%d nmocap=%d)"),
				*RelXmlPath,
				static_cast<int32>(UeModel->nq), static_cast<int32>(UeModel->nv), static_cast<int32>(UeModel->nu),
				static_cast<int32>(UeModel->nbody), static_cast<int32>(UeModel->njnt), static_cast<int32>(UeModel->ngeom),
				static_cast<int32>(UeModel->nsite), static_cast<int32>(UeModel->nsensor), static_cast<int32>(UeModel->nmocap),
				static_cast<int32>(Rm->nq), static_cast<int32>(Rm->nv), static_cast<int32>(Rm->nu),
				static_cast<int32>(Rm->nbody), static_cast<int32>(Rm->njnt), static_cast<int32>(Rm->ngeom),
				static_cast<int32>(Rm->nsite), static_cast<int32>(Rm->nsensor), static_cast<int32>(Rm->nmocap)));
			Actor->Destroy();
			continue;
		}

		// 4. Numeric mjModel parity
		constexpr double ModelTol = 1e-6;
		const double MaxBodyPos = MaxArrayAbsDiff(UeModel->body_pos, Rm->body_pos, 3 * Rm->nbody);
		const double MaxBodyQuat = MaxQuatArrayAbsDiff(UeModel->body_quat, Rm->body_quat, Rm->nbody);
		const double MaxBodyMass = MaxArrayAbsDiff(UeModel->body_mass, Rm->body_mass, Rm->nbody);
		const double MaxBodyInertia = MaxArrayAbsDiff(UeModel->body_inertia, Rm->body_inertia, 3 * Rm->nbody);
		const int32 GeomTypeMismatch = CountIntMismatches(UeModel->geom_type, Rm->geom_type, Rm->ngeom);
		const double MaxGeomSize = MaxArrayAbsDiff(UeModel->geom_size, Rm->geom_size, 3 * Rm->ngeom);
		const int32 JntTypeMismatch = CountIntMismatches(UeModel->jnt_type, Rm->jnt_type, Rm->njnt);
		const double MaxJntRange = MaxArrayAbsDiff(UeModel->jnt_range, Rm->jnt_range, 2 * Rm->njnt);
		const int32 ActTrnMismatch = CountIntMismatches(UeModel->actuator_trntype, Rm->actuator_trntype, Rm->nu);
		const double MaxActGain = MaxArrayAbsDiff(UeModel->actuator_gainprm, Rm->actuator_gainprm, mjNGAIN * Rm->nu);
		const double MaxActBias = MaxArrayAbsDiff(UeModel->actuator_biasprm, Rm->actuator_biasprm, mjNBIAS * Rm->nu);

		TestTrue(FString::Printf(TEXT("[%s] body_pos maxDiff=%.3e"), *RelXmlPath, MaxBodyPos), MaxBodyPos <= ModelTol);
		TestTrue(FString::Printf(TEXT("[%s] body_quat maxDiff=%.3e"), *RelXmlPath, MaxBodyQuat), MaxBodyQuat <= ModelTol);
		TestTrue(FString::Printf(TEXT("[%s] body_mass maxDiff=%.3e"), *RelXmlPath, MaxBodyMass), MaxBodyMass <= ModelTol);
		TestTrue(FString::Printf(TEXT("[%s] body_inertia maxDiff=%.3e"), *RelXmlPath, MaxBodyInertia), MaxBodyInertia <= ModelTol);
		TestEqual(FString::Printf(TEXT("[%s] geom_type mismatches"), *RelXmlPath), GeomTypeMismatch, 0);
		TestTrue(FString::Printf(TEXT("[%s] geom_size maxDiff=%.3e"), *RelXmlPath, MaxGeomSize), MaxGeomSize <= ModelTol);
		TestEqual(FString::Printf(TEXT("[%s] jnt_type mismatches"), *RelXmlPath), JntTypeMismatch, 0);
		TestTrue(FString::Printf(TEXT("[%s] jnt_range maxDiff=%.3e"), *RelXmlPath, MaxJntRange), MaxJntRange <= ModelTol);
		TestEqual(FString::Printf(TEXT("[%s] actuator_trntype mismatches"), *RelXmlPath), ActTrnMismatch, 0);
		TestTrue(FString::Printf(TEXT("[%s] actuator_gainprm maxDiff=%.3e"), *RelXmlPath, MaxActGain), MaxActGain <= ModelTol);
		TestTrue(FString::Printf(TEXT("[%s] actuator_biasprm maxDiff=%.3e"), *RelXmlPath, MaxActBias), MaxActBias <= ModelTol);

		// 5. Open-loop 200-step trajectory parity
		mj_resetData(Rm, Rd);
		mj_resetData(UeModel, UeData);

		double MaxQposDiff = 0.0;
		double MaxQvelDiff = 0.0;
		double MaxSensDiff = 0.0;

		for (int32 Step = 0; Step < 200; ++Step)
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

			MaxQposDiff = FMath::Max(MaxQposDiff, MaxArrayAbsDiff(UeData->qpos, Rd->qpos, Rm->nq));
			MaxQvelDiff = FMath::Max(MaxQvelDiff, MaxArrayAbsDiff(UeData->qvel, Rd->qvel, Rm->nv));
			if (Rm->nsensordata > 0)
			{
				MaxSensDiff = FMath::Max(MaxSensDiff, MaxArrayAbsDiff(UeData->sensordata, Rd->sensordata, Rm->nsensordata));
			}
		}

		AddInfo(FString::Printf(
			TEXT("[%s] PARITY SUMMARY: nq=%d nv=%d nu=%d nbody=%d njnt=%d ngeom=%d nsite=%d nsens=%d | body_pos=%.3e body_quat=%.3e body_mass=%.3e body_inertia=%.3e geom_size=%.3e jnt_range=%.3e gainprm=%.3e biasprm=%.3e | 200-step maxDiff: qpos=%.3e qvel=%.3e sensordata=%.3e"),
			*RelXmlPath,
			static_cast<int32>(Rm->nq), static_cast<int32>(Rm->nv), static_cast<int32>(Rm->nu),
			static_cast<int32>(Rm->nbody), static_cast<int32>(Rm->njnt), static_cast<int32>(Rm->ngeom),
			static_cast<int32>(Rm->nsite), static_cast<int32>(Rm->nsensor),
			MaxBodyPos, MaxBodyQuat, MaxBodyMass, MaxBodyInertia,
			MaxGeomSize, MaxJntRange, MaxActGain, MaxActBias,
			MaxQposDiff, MaxQvelDiff, MaxSensDiff));

		constexpr double TrajTol = 1e-9;
		TestTrue(FString::Printf(TEXT("[%s] 200-step qpos maxDiff=%.3e <= 1e-9"), *RelXmlPath, MaxQposDiff), MaxQposDiff <= TrajTol);
		TestTrue(FString::Printf(TEXT("[%s] 200-step qvel maxDiff=%.3e <= 1e-9"), *RelXmlPath, MaxQvelDiff), MaxQvelDiff <= TrajTol);
		TestTrue(FString::Printf(TEXT("[%s] 200-step sensordata maxDiff=%.3e <= 1e-9"), *RelXmlPath, MaxSensDiff), MaxSensDiff <= TrajTol);

		Actor->Destroy();
	}

	World->DestroyWorld(false);
	return true;
}
