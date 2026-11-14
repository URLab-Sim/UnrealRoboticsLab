// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/AutomationTest.h"
#include "Misc/Crc.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"

#include "MuJoCo/Entity/MjModelSource.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

#if WITH_DEV_AUTOMATION_TESTS
namespace URLabModelSourceSecurityTest
{
const char* TetraObj = "v 0 0 0\nv 0.1 0 0\nv 0 0.1 0\nv 0 0 0.1\nf 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4\n";
TArray<uint8> Utf8(const FString& Text)
{
	const FTCHARToUTF8 Converted(*Text, Text.Len());
	TArray<uint8> Out;
	Out.Append(reinterpret_cast<const uint8*>(Converted.Get()), Converted.Length());
	return Out;
}

TArray<uint8> Raw(const char* Bytes, int32 Length)
{
	TArray<uint8> Out;
	Out.Append(reinterpret_cast<const uint8*>(Bytes), Length);
	return Out;
}

FString MeshModel(const FString& AssetMarkup)
{
	return FString::Printf(TEXT("<mujoco model=\"probe\"><asset>%s</asset><worldbody>")
		TEXT("<geom type=\"mesh\" mesh=\"tetra\"/></worldbody></mujoco>"), *AssetMarkup);
}

TMap<FString, TArray<uint8>> TetraAssets()
{
	TMap<FString, TArray<uint8>> Assets;
	Assets.Add(TEXT("tetra.obj"), Raw(TetraObj, FCStringAnsi::Strlen(TetraObj)));
	return Assets;
}

bool Compiles(const FString& Xml, const TMap<FString, TArray<uint8>>& Assets, FString& OutError)
{
	mjModel* const Model = MjModelSource::FromBytes(Utf8(Xml), TEXT("xml"), Assets, OutError);
	if (Model == nullptr) return false;
	mj_deleteModel(Model);
	return true;
}

void ExpectRejected(FAutomationTestBase& Test, const TCHAR* What, const FString& Xml,
	const TMap<FString, TArray<uint8>>& Assets, const TCHAR* ExpectedFragment)
{
	FString Error;
	Test.TestFalse(What, Compiles(Xml, Assets, Error));
	Test.TestTrue(FString::Printf(TEXT("%s reports '%s' (got '%s')"), What, ExpectedFragment, *Error),
		Error.Contains(ExpectedFragment));
}

/** Minimal stored (uncompressed) zip writer for archive-shaped fixtures. */
TArray<uint8> StoredZip(const TArray<TPair<FString, TArray<uint8>>>& Entries)
{
	TArray<uint8> Out;
	TArray<uint8> Directory;
	const auto U16 = [](TArray<uint8>& To, uint32 Value)
	{
		To.Add(Value & 0xFF);
		To.Add((Value >> 8) & 0xFF);
	};
	const auto U32 = [&U16](TArray<uint8>& To, uint32 Value)
	{
		U16(To, Value & 0xFFFF);
		U16(To, Value >> 16);
	};
	for (const TPair<FString, TArray<uint8>>& Entry : Entries)
	{
		const TArray<uint8> Name = Utf8(Entry.Key);
		const uint32 Crc = FCrc::MemCrc32(Entry.Value.GetData(), Entry.Value.Num());
		const uint32 Offset = Out.Num();
		U32(Out, 0x04034b50);
		U16(Out, 20);
		U16(Out, 0);
		U16(Out, 0);
		U32(Out, 0);
		U32(Out, Crc);
		U32(Out, Entry.Value.Num());
		U32(Out, Entry.Value.Num());
		U16(Out, Name.Num());
		U16(Out, 0);
		Out.Append(Name);
		Out.Append(Entry.Value);

		U32(Directory, 0x02014b50);
		U16(Directory, 20);
		U16(Directory, 20);
		U16(Directory, 0);
		U16(Directory, 0);
		U32(Directory, 0);
		U32(Directory, Crc);
		U32(Directory, Entry.Value.Num());
		U32(Directory, Entry.Value.Num());
		U16(Directory, Name.Num());
		U16(Directory, 0);
		U16(Directory, 0);
		U16(Directory, 0);
		U16(Directory, 0);
		U32(Directory, 0);
		U32(Directory, Offset);
		Directory.Append(Name);
	}
	const uint32 DirectoryOffset = Out.Num();
	Out.Append(Directory);
	U32(Out, 0x06054b50);
	U16(Out, 0);
	U16(Out, 0);
	U16(Out, Entries.Num());
	U16(Out, Entries.Num());
	U32(Out, Directory.Num());
	U32(Out, DirectoryOffset);
	U16(Out, 0);
	return Out;
}

bool MjzCompiles(const TArray<uint8>& Archive, FString& OutError)
{
	mjModel* const Model = MjModelSource::FromBytes(Archive, TEXT("mjz"), {}, OutError);
	if (Model == nullptr) return false;
	mj_deleteModel(Model);
	return true;
}

TSet<FString> MjzStagingFiles()
{
	TSet<FString> Found;
	for (const FString& Dir : {FPaths::ProjectIntermediateDir(), FString(FPlatformProcess::UserTempDir())})
	{
		TArray<FString> Names;
		IFileManager::Get().FindFiles(Names, *FPaths::Combine(Dir, TEXT("urlab_mjz_*")), true, false);
		for (const FString& Name : Names) Found.Add(FPaths::Combine(Dir, Name));
	}
	return Found;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabModelSourceXmlVfsOnlyTest,
	"URLab.Bridge.ModelSource.XmlNeverReadsServerFiles",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabModelSourceXmlVfsOnlyTest::RunTest(const FString&)
{
	using namespace URLabModelSourceSecurityTest;
	FString Error;
	const TMap<FString, TArray<uint8>> Tetra = TetraAssets();
	if (!TestTrue(TEXT("legit VFS mesh compiles"),
		Compiles(MeshModel(TEXT("<mesh name=\"tetra\" file=\"tetra.obj\"/>")), Tetra, Error)))
	{
		AddError(Error);
	}
	TestTrue(TEXT("meshdir prefix and case-insensitive name still resolve in the VFS"), Compiles(
		TEXT("<mujoco><compiler meshdir=\"assets/\"/><asset><mesh name=\"tetra\" file=\"TETRA.obj\"/></asset>")
		TEXT("<worldbody><geom type=\"mesh\" mesh=\"tetra\"/></worldbody></mujoco>"), Tetra, Error));
	TestTrue(TEXT("commented-out path is ignored"), Compiles(
		MeshModel(TEXT("<!-- <include file=\"/etc/hostname\"/> --><mesh name=\"tetra\" file=\"tetra.obj\"/>")),
		Tetra, Error));

	const FString ProbeDir = FPaths::Combine(FPlatformProcess::UserTempDir(),
		FString::Printf(TEXT("urlab_vfs_probe_%s"), *FGuid::NewGuid().ToString()));
	const FString ProbeInclude = FPaths::Combine(ProbeDir, TEXT("leak.xml"));
	TestTrue(TEXT("on-disk include probe writes"), FFileHelper::SaveStringToFile(
		TEXT("<mujocoinclude><worldbody><body name=\"leaked\"/></worldbody></mujocoinclude>"), *ProbeInclude));
	TestTrue(TEXT("probe file exists on the server"), FPaths::FileExists(ProbeInclude));
	ExpectRejected(*this, TEXT("absolute include of an existing server file"),
		FString::Printf(TEXT("<mujoco><include file=\"%s\"/></mujoco>"), *ProbeInclude), {},
		TEXT("relative path"));
	TMap<FString, TArray<uint8>> LeakNamed;
	LeakNamed.Add(TEXT("leak.xml"), Utf8(TEXT("<mujocoinclude/>")));
	ExpectRejected(*this, TEXT("absolute include whose basename is uploaded"),
		FString::Printf(TEXT("<mujoco><include file=\"%s\"/></mujoco>"), *ProbeInclude), LeakNamed,
		TEXT("relative path"));
	IFileManager::Get().DeleteDirectory(*ProbeDir, false, true);

	ExpectRejected(*this, TEXT("/etc/hostname include"),
		TEXT("<mujoco><include file=\"/etc/hostname\"/></mujoco>"), {}, TEXT("relative path"));
	ExpectRejected(*this, TEXT("entity-encoded absolute include"),
		TEXT("<mujoco><include file=\"&#47;etc&#x2F;hostname\"/></mujoco>"), {}, TEXT("relative path"));
	ExpectRejected(*this, TEXT("relative include not in the VFS"),
		TEXT("<mujoco><include file=\"hostname\"/></mujoco>"), {}, TEXT("names no uploaded asset"));

	TMap<FString, TArray<uint8>> Texture;
	Texture.Add(TEXT("x.png"), Utf8(TEXT("texture bytes")));
	ExpectRejected(*this, TEXT("../x.png texture"),
		TEXT("<mujoco><asset><texture name=\"t\" type=\"2d\" file=\"../x.png\"/></asset></mujoco>"),
		Texture, TEXT("relative path"));
	ExpectRejected(*this, TEXT("absolute mesh path"),
		MeshModel(TEXT("<mesh name=\"tetra\" file=\"/tmp/tetra.obj\"/>")), Tetra, TEXT("relative path"));
	ExpectRejected(*this, TEXT("Windows drive mesh path"),
		MeshModel(TEXT("<mesh name=\"tetra\" file=\"C:\\models\\tetra.obj\"/>")), Tetra, TEXT("relative path"));
	ExpectRejected(*this, TEXT("UNC mesh path"),
		MeshModel(TEXT("<mesh name=\"tetra\" file=\"\\\\server\\share\\tetra.obj\"/>")), Tetra,
		TEXT("relative path"));
	ExpectRejected(*this, TEXT("cube texture face outside the VFS"),
		TEXT("<mujoco><asset><texture name=\"t\" type=\"cube\" fileright=\"/etc/x.png\"/></asset></mujoco>"),
		Texture, TEXT("relative path"));
	ExpectRejected(*this, TEXT("mesh missing from the VFS"),
		MeshModel(TEXT("<mesh name=\"tetra\" file=\"missing.obj\"/>")), Tetra, TEXT("names no uploaded asset"));
	ExpectRejected(*this, TEXT("absolute meshdir"),
		TEXT("<mujoco><compiler meshdir=\"/etc\"/><asset><mesh name=\"tetra\" file=\"tetra.obj\"/></asset>")
		TEXT("<worldbody><geom type=\"mesh\" mesh=\"tetra\"/></worldbody></mujoco>"), Tetra,
		TEXT("relative directory"));
	ExpectRejected(*this, TEXT("parent texturedir"),
		TEXT("<mujoco><compiler texturedir=\"../\"/></mujoco>"), {}, TEXT("relative directory"));

	TMap<FString, TArray<uint8>> Evil = TetraAssets();
	Evil.Add(TEXT("../evil.obj"), Raw(TetraObj, FCStringAnsi::Strlen(TetraObj)));
	ExpectRejected(*this, TEXT("traversing asset name"),
		MeshModel(TEXT("<mesh name=\"tetra\" file=\"tetra.obj\"/>")), Evil, TEXT("asset name"));

	TMap<FString, TArray<uint8>> Included = TetraAssets();
	Included.Add(TEXT("parts/body.xml"), Utf8(TEXT(
		"<mujocoinclude><asset><mesh name=\"tetra\" file=\"tetra.obj\"/></asset></mujocoinclude>")));
	TestTrue(TEXT("VFS include resolves"), Compiles(
		TEXT("<mujoco><include file=\"parts/body.xml\"/><worldbody><geom type=\"mesh\" mesh=\"tetra\"/>")
		TEXT("</worldbody></mujoco>"), Included, Error));
	Included.Add(TEXT("parts/leak.xml"), Utf8(TEXT("<mujocoinclude><include file=\"/etc/hostname\"/></mujocoinclude>")));
	ExpectRejected(*this, TEXT("nested include escaping the VFS"),
		TEXT("<mujoco><include file=\"parts/leak.xml\"/></mujoco>"), Included, TEXT("relative path"));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabModelSourceMjzInMemoryTest,
	"URLab.Bridge.ModelSource.MjzInMemoryAndVfsOnly",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabModelSourceMjzInMemoryTest::RunTest(const FString&)
{
	using namespace URLabModelSourceSecurityTest;
	const TSet<FString> StagedBefore = MjzStagingFiles();
	FString Error;

	TArray<TPair<FString, TArray<uint8>>> Legit;
	Legit.Emplace(TEXT("model.xml"), Utf8(MeshModel(TEXT("<mesh name=\"tetra\" file=\"meshes/tetra.obj\"/>"))));
	Legit.Emplace(TEXT("meshes/tetra.obj"), Raw(TetraObj, FCStringAnsi::Strlen(TetraObj)));
	if (!TestTrue(TEXT("stored mjz with a subdirectory asset compiles"), MjzCompiles(StoredZip(Legit), Error)))
	{
		AddError(Error);
	}

	TArray<TPair<FString, TArray<uint8>>> Include;
	Include.Emplace(TEXT("model.xml"), Utf8(TEXT("<mujoco><include file=\"/etc/hostname\"/></mujoco>")));
	TestFalse(TEXT("mjz include of a server file is rejected"), MjzCompiles(StoredZip(Include), Error));
	TestTrue(TEXT("mjz include rejection is explicit"), Error.Contains(TEXT("relative path")));

	TArray<TPair<FString, TArray<uint8>>> Slip = Legit;
	Slip.Emplace(TEXT("../escape.obj"), Raw(TetraObj, FCStringAnsi::Strlen(TetraObj)));
	TestFalse(TEXT("traversing mjz entry is rejected"), MjzCompiles(StoredZip(Slip), Error));
	TestTrue(TEXT("traversing entry rejection is explicit"), Error.Contains(TEXT("mjz entry")));

	TArray<TPair<FString, TArray<uint8>>> NoRoot;
	NoRoot.Emplace(TEXT("scene.xml"), Utf8(MeshModel(TEXT(""))));
	TestFalse(TEXT("mjz without model.xml is rejected"), MjzCompiles(StoredZip(NoRoot), Error));
	TestTrue(TEXT("missing root is explicit"), Error.Contains(TEXT("model.xml")));

	TArray<uint8> Corrupt = StoredZip(Legit);
	Corrupt[Corrupt.Num() / 3] ^= 0x5A;
	TestFalse(TEXT("corrupted mjz is rejected"), MjzCompiles(Corrupt, Error));
	TestFalse(TEXT("garbage is not a zip"), MjzCompiles(Utf8(TEXT("not a zip archive at all")), Error));

	TestTrue(TEXT("mjz parsing stages no files in Intermediate or the user temp dir"),
		MjzStagingFiles().Difference(StagedBefore).IsEmpty());
	return !HasAnyErrors();
}
#endif
