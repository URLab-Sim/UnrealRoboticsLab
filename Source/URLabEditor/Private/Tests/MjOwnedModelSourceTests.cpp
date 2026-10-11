// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Misc/AutomationTest.h"

#include "Interfaces/IPluginManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "MuJoCo/Core/MjOwnedModel.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

#if WITH_DEV_AUTOMATION_TESTS
namespace
{
const char* OwnedSourceAssetXml = R"(
<mujoco model="owned_source_asset">
  <asset><mesh name="tetra" file="tetra.obj"/></asset>
  <worldbody><body><freejoint/><geom type="mesh" mesh="tetra" mass="1"/></body></worldbody>
</mujoco>)";

const char* OwnedSourceTetraObj = R"(v 0 0 0
v 0.1 0 0
v 0 0.1 0
v 0 0 0.1
f 1 3 2
f 1 2 4
f 2 3 4
f 3 1 4
)";

TArray<uint8> OwnedSourceBytes(const char* Text)
{
	TArray<uint8> Out;
	Out.Append(reinterpret_cast<const uint8*>(Text), FCStringAnsi::Strlen(Text));
	return Out;
}

bool OwnedSourceSaveMjb(const FMjOwnedModelCandidate& Candidate, TArray<uint8>& Out)
{
	return Candidate.VisitPrepared([&Out](const mjModel* Model, const mjData*)
	{
		const mjtSize Size = mj_sizeModel(Model);
		if (Size <= 0 || Size > MAX_int32) return false;
		Out.SetNumUninitialized(static_cast<int32>(Size));
		mj_saveModel(Model, nullptr, Out.GetData(), Out.Num());
		return true;
	});
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabOwnedModelPrimarySourcesTest,
	"URLab.OwnedModel.PrimarySourcesWithoutSimulation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabOwnedModelPrimarySourcesTest::RunTest(const FString&)
{
	FString Error;
	TMap<FString, TArray<uint8>> Assets;
	Assets.Add(TEXT("tetra.obj"), OwnedSourceBytes(OwnedSourceTetraObj));
	FMjOwnedModelCandidate Xml;
	if (!TestTrue(TEXT("XML and owned assets prepare directly"),
		FMjOwnedModelCandidate::FromBytes(EMjModelSourceKind::XmlWithAssets,
			OwnedSourceBytes(OwnedSourceAssetXml), Assets, Xml, Error, TEXT("owned XML"))))
	{
		AddError(Error);
		return false;
	}
	TestEqual(TEXT("neutral source metadata is retained"), Xml.GetSourceInfo().Kind,
		EMjModelSourceKind::XmlWithAssets);
	TestTrue(TEXT("asset model is available without a simulation runtime"),
		Xml.VisitPrepared([](const mjModel* Model, const mjData* Data)
		{
			return Model->nmesh == 1 && Data->signature == Model->signature && Data->time == 0.0;
		}));

	TArray<uint8> MjbBytes;
	TestTrue(TEXT("prepared source serializes as MJB"), OwnedSourceSaveMjb(Xml, MjbBytes));
	FMjOwnedModelCandidate Mjb;
	TestTrue(TEXT("MJB prepares directly"), FMjOwnedModelCandidate::FromBytes(
		EMjModelSourceKind::Mjb, MjbBytes, {}, Mjb, Error, TEXT("owned MJB")));
	TestTrue(TEXT("MJB owns a coherent model/data pair"), Mjb.IsValid());

	mjModel* Borrowed = nullptr;
	Xml.VisitPrepared([&Borrowed](const mjModel* Model, const mjData*)
	{
		Borrowed = mj_copyModel(nullptr, Model);
		return Borrowed != nullptr;
	});
	FMjOwnedModelCandidate Compiled;
	const bool bCopied = FMjOwnedModelCandidate::CopyCompiled(
		Borrowed, Compiled, Error, TEXT("owned compiled copy"));
	mj_deleteModel(Borrowed);
	TestTrue(TEXT("in-process compiled source is copied"), bCopied);
	TestTrue(TEXT("compiled candidate survives source deletion"), Compiled.IsValid());
	TestEqual(TEXT("compiled source metadata is neutral"), Compiled.GetSourceInfo().Kind,
		EMjModelSourceKind::CompiledModel);

	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("UnrealRoboticsLab"));
	if (!TestTrue(TEXT("plugin fixture root exists"), Plugin.IsValid())) return false;
	TArray<uint8> MjzBytes;
	const FString MjzPath = FPaths::Combine(Plugin->GetBaseDir(),
		TEXT("third_party/MuJoCo/src/test/testdata/model.mjz"));
	if (!TestTrue(TEXT("MJZ fixture reads"), FFileHelper::LoadFileToArray(MjzBytes, *MjzPath)))
	{
		AddError(MjzPath);
		return false;
	}
	FMjOwnedModelCandidate Mjz;
	if (!TestTrue(TEXT("MJZ prepares directly"), FMjOwnedModelCandidate::FromBytes(
		EMjModelSourceKind::Mjz, MjzBytes, {}, Mjz, Error, TEXT("owned MJZ"))))
	{
		AddError(Error);
		return false;
	}
	TestTrue(TEXT("MJZ internal assets survive in owned model"), Mjz.VisitPrepared(
		[](const mjModel* Model, const mjData*) { return Model->nmesh > 0 && Model->ntex > 0; }));

	TMap<FString, TArray<uint8>> InvalidAssets;
	InvalidAssets.Add(TEXT("tetra.obj"), TArray<uint8>());
	TestFalse(TEXT("failed replacement is rejected"), FMjOwnedModelCandidate::FromBytes(
		EMjModelSourceKind::XmlWithAssets, OwnedSourceBytes(OwnedSourceAssetXml),
		InvalidAssets, Mjz, Error));
	TestTrue(TEXT("failed replacement preserves output candidate"), Mjz.IsValid());
	return !HasAnyErrors();
}
#endif // WITH_DEV_AUTOMATION_TESTS
