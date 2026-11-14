#include "Misc/AutomationTest.h"
#include "MJUnitConverter.h"
#include "MJModelComponent.h"
#include "MJSceneComponents.h"
#include "MJActorComponents.h"
#include "MJAssetLibrary.h"
#include "MJXmlImporterExporter.h"
#include "Generated/MJSpecMarshal.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "mujoco/mujoco.h"

namespace
{
// Minimal valid 2x2 RGB PNG (zlib compressed)
TArray<uint8> MakeTiny2x2Png()
{
	static const uint8 PngBytes[] = {
		0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A,
		0x00, 0x00, 0x00, 0x0D, 0x49, 0x48, 0x44, 0x52,
		0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02,
		0x08, 0x02, 0x00, 0x00, 0x00, 0xFD, 0xD4, 0x9A, 0x73,
		0x00, 0x00, 0x00, 0x12, 0x49, 0x44, 0x41, 0x54,
		0x78, 0x9C, 0x63, 0xF8, 0xCF, 0xC0, 0xC0, 0x00,
		0xC2, 0x0C, 0xFF, 0x81, 0x00, 0x00, 0x1F, 0xEE,
		0x05, 0xFB, 0x0B, 0xD9, 0x68, 0x8B,
		0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4E, 0x44,
		0xAE, 0x42, 0x60, 0x82
	};
	return TArray<uint8>(PngBytes, UE_ARRAY_COUNT(PngBytes));
}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMJCodegenAndUnitsTest,
	"MuJoCoUE.CodegenAndUnits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMJCodegenAndUnitsTest::RunTest(const FString& Parameters)
{
	// 1. Position LHS cm <-> RHS m
	const FVector UEPos(100.0, 250.0, -50.0);
	double MJPos[3] = {};
	FMJUnitConverter::UEPositionToMJ(UEPos, MJPos);
	TestTrue(TEXT("MJPos X"), FMath::IsNearlyEqual(MJPos[0], 1.0, 1e-6));
	TestTrue(TEXT("MJPos Y negated"), FMath::IsNearlyEqual(MJPos[1], -2.5, 1e-6));
	TestTrue(TEXT("MJPos Z"), FMath::IsNearlyEqual(MJPos[2], -0.5, 1e-6));

	FVector RoundTripPos;
	FMJUnitConverter::MJPositionToUE(MJPos, RoundTripPos);
	TestTrue(TEXT("Position round-trip"), RoundTripPos.Equals(UEPos, 1e-5));

	// 2. Quaternion LHS <-> RHS involution + rotated vector consistency
	const FQuat UEQuat(FRotator(15.0, -35.0, 40.0));
	double MJQuat[4] = {};
	FMJUnitConverter::UEQuatToMJ(UEQuat, MJQuat);
	FQuat RoundTripQuat;
	FMJUnitConverter::MJQuatToUE(MJQuat, RoundTripQuat);
	TestTrue(TEXT("Quat round-trip"), UEQuat.Equals(RoundTripQuat, 1e-5));

	// 3. Sparse override preservation on FMJGeomSpec
	FMJSpecHandle Spec(mj_makeSpec());
	mjsDefault* Def = mjs_addDefault(Spec.Get(), "green_box", nullptr);
	Def->geom->rgba[0] = 0.0f; Def->geom->rgba[1] = 1.0f; Def->geom->rgba[2] = 0.0f; Def->geom->rgba[3] = 1.0f;
	Def->geom->density = 500.0;

	mjsBody* World = mjs_findBody(Spec.Get(), "world");
	mjsGeom* Geom = mjs_addGeom(World, Def);

	FMJGeomSpec UESpec;
	FMJSpecMarshal::FromMJSpec(Geom, UESpec, Def->geom, true);
	TestFalse(TEXT("Non-overridden Rgba should not be flagged"), UESpec.bOverride_Rgba);
	TestFalse(TEXT("Non-overridden Density should not be flagged"), UESpec.bOverride_Density);

	// Override only Density, keep Rgba inherited
	UESpec.bOverride_Density = true;
	UESpec.Density = 800.0;
	FMJSpecMarshal::ToMJSpec(UESpec, Geom, true);
	TestTrue(TEXT("Density updated"), FMath::IsNearlyEqual(Geom->density, 800.0, 1e-6));
	TestTrue(TEXT("Rgba inherited from default"), FMath::IsNearlyEqual(static_cast<double>(Geom->rgba[1]), 1.0, 1e-6));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMJXmlRoundTripTest,
	"MuJoCoUE.XmlRoundTrip",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMJXmlRoundTripTest::RunTest(const FString& Parameters)
{
	const FString InputXml = TEXT(R"(
<mujoco model="roundtrip_test">
  <default>
    <default class="blue_geom">
      <geom rgba="0 0 1 1"/>
    </default>
  </default>
  <worldbody>
    <body name="arm" pos="0 0 1">
      <frame name="tip_frame" pos="0.5 0 0">
        <geom name="link_cap" class="blue_geom" type="capsule" fromto="0 0 0 0 0 0.5" size="0.05"/>
      </frame>
      <joint name="hinge1" type="hinge" axis="0 1 0"/>
    </body>
  </worldbody>
  <actuator>
    <position name="servo1" joint="hinge1" kp="150" kv="15"/>
  </actuator>
</mujoco>
)");

	UWorld* World = UWorld::CreateWorld(EWorldType::Editor, false);
	TestNotNull(TEXT("Transient World created"), World);
	AActor* Actor = World->SpawnActor<AActor>();
	TestNotNull(TEXT("Transient Actor spawned"), Actor);

	FString Err;
	const bool bImported = FMJXmlImporterExporter::ImportXMLToActor(InputXml, Actor, nullptr, Err);
	TestTrue(FString::Printf(TEXT("ImportXMLToActor succeeded: %s"), *Err), bImported);

	UMJActuatorComponent* Act = Actor->FindComponentByClass<UMJActuatorComponent>();
	TestNotNull(TEXT("Actuator component created"), Act);
	if (Act)
	{
		TestEqual(TEXT("Actuator shortcut is Position"), Act->Shortcut, EMJActuatorShortcut::Position);
		TestTrue(TEXT("Kp imported as 150"), FMath::IsNearlyEqual(Act->ShortcutParams.Kp, 150.0, 1e-5));
		Act->ShortcutParams.Kp = 225.0;
	}

	FString ExportedXml;
	const bool bExported = FMJXmlImporterExporter::ExportActorToXML(Actor, ExportedXml, Err);
	TestTrue(FString::Printf(TEXT("ExportActorToXML succeeded: %s"), *Err), bExported);
	UE_LOG(LogTemp, Display, TEXT("XmlRoundTrip ExportedXml:\n%s"), *ExportedXml);

	TestTrue(TEXT("Exported XML preserves <position"), ExportedXml.Contains(TEXT("<position")));
	TestTrue(TEXT("Exported XML has updated kp=225"), ExportedXml.Contains(TEXT("kp=\"225\"")));
	TestTrue(TEXT("Exported XML preserves kv=15"), ExportedXml.Contains(TEXT("kv=\"15\"")));
	TestTrue(TEXT("Exported XML preserves fromto"), ExportedXml.Contains(TEXT("fromto=")));
	TestTrue(TEXT("Exported XML preserves <frame"), ExportedXml.Contains(TEXT("<frame")));
	TestTrue(TEXT("Exported XML preserves <default"), ExportedXml.Contains(TEXT("<default")));

	World->DestroyWorld(false);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMJCompileStepRecompileTest,
	"MuJoCoUE.CompileStepRecompile",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMJCompileStepRecompileTest::RunTest(const FString& Parameters)
{
	UWorld* World = UWorld::CreateWorld(EWorldType::Editor, false);
	AActor* Actor = World->SpawnActor<AActor>();

	USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("Root"));
	Root->RegisterComponent();
	Actor->SetRootComponent(Root);

	UMJModelComponent* ModelComp = NewObject<UMJModelComponent>(Actor, TEXT("Model"));
	ModelComp->RegisterComponent();

	UMJBodyComponent* Body = NewObject<UMJBodyComponent>(Actor, TEXT("Pendulum"));
	Body->ElementName = TEXT("pendulum");
	Body->SetRelativeLocation(FVector(0.0, 0.0, 100.0));
	Body->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
	Body->RegisterComponent();

	UMJJointComponent* Joint = NewObject<UMJJointComponent>(Actor, TEXT("Hinge"));
	Joint->ElementName = TEXT("hinge");
	Joint->Spec.bOverride_Type = true;
	Joint->Spec.Type = EMJJoint::Hinge;
	Joint->AttachToComponent(Body, FAttachmentTransformRules::KeepRelativeTransform);
	Joint->RegisterComponent();

	UMJGeomComponent* Geom = NewObject<UMJGeomComponent>(Actor, TEXT("Ball"));
	Geom->ElementName = TEXT("ball");
	Geom->Spec.bOverride_Type = true;
	Geom->Spec.Type = EMJGeom::Sphere;
	Geom->Spec.bOverride_Size = true;
	Geom->Spec.Size = { 10.0, 0.0, 0.0 }; // 10 cm -> 0.1 m
	Geom->SetRelativeLocation(FVector(0.0, 0.0, -50.0));
	Geom->AttachToComponent(Body, FAttachmentTransformRules::KeepRelativeTransform);
	Geom->RegisterComponent();

	FString Err;
	TestTrue(FString::Printf(TEXT("CompileModel: %s"), *Err), ModelComp->CompileModel(Err));
	TestNotNull(TEXT("mjModel valid"), ModelComp->GetModel());
	TestNotNull(TEXT("mjData valid"), ModelComp->GetData());

	ModelComp->GetData()->qpos[0] = 0.42;
	ModelComp->GetData()->qvel[0] = 1.25;
	ModelComp->StepSimulation(0.01);

	const double QposBefore = ModelComp->GetData()->qpos[0];
	const double QvelBefore = ModelComp->GetData()->qvel[0];

	// Edit a parameter and recompile in-place
	Geom->Spec.bOverride_Friction = true;
	Geom->Spec.Friction = { 0.9, 0.005, 0.0001 };
	TestTrue(FString::Printf(TEXT("RecompileModel: %s"), *Err), ModelComp->RecompileModel(Err));

	TestTrue(
		FString::Printf(TEXT("qpos preserved across mj_recompile (before=%f after=%f)"), QposBefore, ModelComp->GetData()->qpos[0]),
		FMath::IsNearlyEqual(ModelComp->GetData()->qpos[0], QposBefore, 1e-9));
	TestTrue(
		FString::Printf(TEXT("qvel preserved across mj_recompile (before=%f after=%f)"), QvelBefore, ModelComp->GetData()->qvel[0]),
		FMath::IsNearlyEqual(ModelComp->GetData()->qvel[0], QvelBefore, 1e-9));

	World->DestroyWorld(false);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMJVfsAndByRefTextureTest,
	"MuJoCoUE.VfsAndByRefTexture",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMJVfsAndByRefTextureTest::RunTest(const FString& Parameters)
{
	UMJAssetLibrary* AssetLib = NewObject<UMJAssetLibrary>();
	FMJRawAssetBuffer& Buf = AssetLib->RawBuffers.Add(TEXT("checker.png"));
	Buf.Bytes = MakeTiny2x2Png();

	FMJVFSHandle Vfs;
	Vfs.Init();
	const int32 Added = AssetLib->PopulateVFS(Vfs.Get());
	TestEqual(TEXT("Added 1 buffer to mjVFS"), Added, 1);

	const FString Xml = TEXT(R"(
<mujoco model="vfs_tex_test">
  <asset>
    <texture name="tex0" type="2d" file="checker.png"/>
    <material name="mat0" texture="tex0"/>
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1" material="mat0"/>
  </worldbody>
</mujoco>
)");

	char ErrBuf[1024] = {};
	FMJSpecHandle Spec(mj_parseXMLString(TCHAR_TO_UTF8(*Xml), Vfs.Get(), ErrBuf, sizeof(ErrBuf)));
	TestTrue(FString::Printf(TEXT("mj_parseXMLString with VFS: %s"), UTF8_TO_TCHAR(ErrBuf)), Spec.Get() != nullptr);

	FMJModelHandle Model(mj_compile(Spec.Get(), Vfs.Get()));
	TestTrue(TEXT("mj_compile with VFS succeeded"), Model.Get() != nullptr);
	TestEqual(TEXT("ntex == 1"), Model.Get()->ntex, 1);

	// Decode texture via mj_loadTexture (works for both embedded and by-reference tex_adr == -1)
	TArray<uint8> Pixels;
	int32 W = 0, H = 0, CS = 0;
	FString Err;
	const bool bLoaded = UMJAssetLibrary::LoadTextureFromModel(Model.Get(), 0, Pixels, W, H, CS, Err);
	TestTrue(FString::Printf(TEXT("LoadTextureFromModel: %s"), *Err), bLoaded);
	TestEqual(TEXT("Texture width == 2"), W, 2);
	TestEqual(TEXT("Texture height == 2"), H, 2);

	// Test mj_embedAssets
	FMJModelHandle Embedded;
	const bool bEmbedded = UMJAssetLibrary::EmbedModelAssets(Model.Get(), Embedded, Err);
	TestTrue(FString::Printf(TEXT("EmbedModelAssets: %s"), *Err), bEmbedded);
	TestTrue(TEXT("Embedded model has tex_adr >= 0"), Embedded.Get() && Embedded.Get()->tex_adr[0] >= 0);

	return true;
}
