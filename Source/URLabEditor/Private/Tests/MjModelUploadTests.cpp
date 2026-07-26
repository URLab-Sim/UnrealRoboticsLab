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

// ============================================================================
// MjModelUploadTests.cpp
//
// Unit coverage for network model upload (render farm phase 3/4):
//  - FURLabAssetCache SHA-256 vector + atomic Put/Has/GetPath round-trip
//  - upload_model_manifest: upload_id + need lists, cache-hit dedup, caps,
//    path-traversal rejection
//  - upload_model_chunk: reassembly, hash-mismatch reject, oversize reject,
//    unknown-upload reject
//
// The full commit -> import_xml path needs a live editor (import_xml stands up
// a Blueprint asset); it is exercised by the orchestrator's live upload test.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Bridge/AssetCache.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/BridgeServer.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "HAL/FileManager.h"
#include "Misc/Base64.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"

namespace
{
TArray<uint8> RandomBytes(int32 N)
{
	TArray<uint8> Out;
	Out.Reserve(N);
	for (int32 i = 0; i < N; ++i)
		Out.Add(static_cast<uint8>(FMath::Rand() & 0xff));
	return Out;
}

TSharedPtr<FJsonObject> MakeManifest(const FString& Session, const FString& XmlSha,
	const TArray<TPair<FString, FString>>& Assets /* name, sha */,
	const TArray<int64>& Sizes, int64 TotalBytes)
{
	TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
	R->SetStringField(TEXT("op"), TEXT("upload_model_manifest"));
	R->SetStringField(TEXT("session_id"), Session);
	R->SetStringField(TEXT("xml_sha256"), XmlSha);
	TArray<TSharedPtr<FJsonValue>> Arr;
	for (int32 i = 0; i < Assets.Num(); ++i)
	{
		TSharedPtr<FJsonObject> A = MakeShared<FJsonObject>();
		A->SetStringField(TEXT("name"), Assets[i].Key);
		A->SetStringField(TEXT("sha256"), Assets[i].Value);
		A->SetNumberField(TEXT("size"), static_cast<double>(Sizes.IsValidIndex(i) ? Sizes[i] : 0));
		Arr.Add(MakeShared<FJsonValueObject>(A));
	}
	R->SetArrayField(TEXT("assets"), Arr);
	R->SetNumberField(TEXT("total_bytes"), static_cast<double>(TotalBytes));
	return R;
}

TSharedPtr<FJsonObject> MakeChunk(const FString& Session, const FString& UploadId,
	const FString& Kind, const FString& Name, int64 Total, int64 Offset,
	const TArray<uint8>& Data)
{
	TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
	R->SetStringField(TEXT("op"), TEXT("upload_model_chunk"));
	R->SetStringField(TEXT("session_id"), Session);
	R->SetStringField(TEXT("upload_id"), UploadId);
	R->SetStringField(TEXT("kind"), Kind);
	R->SetStringField(TEXT("name"), Name);
	R->SetNumberField(TEXT("total"), static_cast<double>(Total));
	R->SetNumberField(TEXT("offset"), static_cast<double>(Offset));
	// Binary field arrives on the wire under a __b64__ key (see MsgpackHelpers).
	R->SetStringField(TEXT("data__b64__"), FBase64::Encode(Data));
	return R;
}

FString ReplyCode(const TSharedPtr<FJsonObject>& Reply)
{
	FString Code;
	if (Reply.IsValid())
		Reply->TryGetStringField(TEXT("code"), Code);
	return Code;
}
} // namespace

// ---------------------------------------------------------------------------
// 1. Content-addressed cache: SHA-256 vector + atomic Put/Has/Get round-trip.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjModelUploadAssetCache,
	"URLab.ModelUpload.AssetCache",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjModelUploadAssetCache::RunTest(const FString& Parameters)
{
	// Known FIPS 180-4 vector: SHA-256("abc").
	const FString Abc = TEXT("abc");
	FTCHARToUTF8 AbcUtf8(*Abc);
	const FString AbcHash = FURLabAssetCache::Sha256Hex(
		reinterpret_cast<const uint8*>(AbcUtf8.Get()), AbcUtf8.Length());
	TestEqual(TEXT("SHA-256(\"abc\")"), AbcHash,
		FString(TEXT("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")));

	// Empty input vector.
	TestEqual(TEXT("SHA-256(\"\")"), FURLabAssetCache::Sha256Hex(nullptr, 0),
		FString(TEXT("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855")));

	const FString Root = FPaths::Combine(FPaths::ProjectIntermediateDir(),
		TEXT("URLabTest"), FGuid::NewGuid().ToString(EGuidFormats::Digits));
	FURLabAssetCache Cache(Root);

	const TArray<uint8> Bytes = RandomBytes(4096);
	const FString Sha = FURLabAssetCache::Sha256Hex(Bytes);

	TestFalse(TEXT("miss before Put"), Cache.Has(Sha));
	TestTrue(TEXT("Put succeeds"), Cache.Put(Sha, Bytes));
	TestTrue(TEXT("hit after Put"), Cache.Has(Sha));

	FString Path;
	TestTrue(TEXT("GetPath resolves"), Cache.GetPath(Sha, Path));
	TestTrue(TEXT("blob file exists"), IFileManager::Get().FileExists(*Path));
	TestTrue(TEXT("blob sharded under sha[:2]"), Path.Contains(Sha.Left(2)));

	TArray<uint8> Read;
	TestTrue(TEXT("blob reads back"), FFileHelper::LoadFileToArray(Read, *Path));
	TestEqual(TEXT("round-trip bytes match"), Read.Num(), Bytes.Num());
	TestTrue(TEXT("round-trip content identical"), Read == Bytes);

	// Idempotent: a second Put of identical content is a no-op success.
	TestTrue(TEXT("second Put idempotent"), Cache.Put(Sha, Bytes));

	IFileManager::Get().DeleteDirectory(*Root, /*RequireExists=*/false, /*Tree=*/true);
	return true;
}

// ---------------------------------------------------------------------------
// 2. Manifest + chunk: reassembly, dedup, and every 4.6 validation reject.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjModelUploadManifestChunk,
	"URLab.ModelUpload.ManifestChunk",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjModelUploadManifestChunk::RunTest(const FString& Parameters)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->AddToRoot();
	Server->Start(TEXT("")); // dispatcher only, no ZMQ bind
	FURLabRpcDispatcher* Disp = Server->GetDispatcher();
	if (!Disp)
	{
		AddError(TEXT("Server has no dispatcher"));
		Server->RemoveFromRoot();
		return false;
	}
	const FString Session = TEXT("upload-session");
	Disp->SetActiveSessionIdForTest(Session);

	// Random content per run so the cache state (fresh vs seeded) is
	// deterministic regardless of the shared on-disk cache root.
	const TArray<uint8> XmlBytes = RandomBytes(1024);
	const TArray<uint8> MeshBytes = RandomBytes(8192);
	const FString XmlSha = FURLabAssetCache::Sha256Hex(XmlBytes);
	const FString MeshSha = FURLabAssetCache::Sha256Hex(MeshBytes);

	// --- manifest: fresh hashes -> everything is needed. ---
	TSharedPtr<FJsonObject> ManReply = Disp->Dispatch(MakeManifest(Session, XmlSha,
		{{TEXT("mesh.STL"), MeshSha}}, {MeshBytes.Num()}, XmlBytes.Num() + MeshBytes.Num()));
	FString ManOp;
	ManReply->TryGetStringField(TEXT("op"), ManOp);
	TestEqual(TEXT("manifest ok"), ManOp, FString(TEXT("upload_model_manifest_ok")));

	FString UploadId;
	ManReply->TryGetStringField(TEXT("upload_id"), UploadId);
	TestTrue(TEXT("upload_id issued"), !UploadId.IsEmpty());

	bool bNeedXml = false;
	ManReply->TryGetBoolField(TEXT("need_xml"), bNeedXml);
	TestTrue(TEXT("need_xml true (cache miss)"), bNeedXml);

	const TArray<TSharedPtr<FJsonValue>>* Need = nullptr;
	ManReply->TryGetArrayField(TEXT("need_assets"), Need);
	TestTrue(TEXT("need_assets lists the mesh"), Need && Need->Num() == 1);

	double MaxAsset = 0, MaxTotal = 0;
	ManReply->TryGetNumberField(TEXT("max_asset_bytes"), MaxAsset);
	ManReply->TryGetNumberField(TEXT("max_total_bytes"), MaxTotal);
	TestTrue(TEXT("max_asset_bytes advertised"), MaxAsset > 0);
	TestTrue(TEXT("max_total_bytes advertised"), MaxTotal >= MaxAsset);

	// --- chunk: xml in one shot completes. ---
	TSharedPtr<FJsonObject> XmlChunk = Disp->Dispatch(
		MakeChunk(Session, UploadId, TEXT("xml"), TEXT(""), XmlBytes.Num(), 0, XmlBytes));
	bool bXmlComplete = false;
	XmlChunk->TryGetBoolField(TEXT("complete"), bXmlComplete);
	TestTrue(TEXT("xml chunk completes"), bXmlComplete);

	// --- chunk: hash mismatch (correct total, wrong bytes) is rejected. ---
	TArray<uint8> Corrupt = MeshBytes;
	Corrupt[0] ^= 0xff;
	TSharedPtr<FJsonObject> BadChunk = Disp->Dispatch(
		MakeChunk(Session, UploadId, TEXT("asset"), TEXT("mesh.STL"), MeshBytes.Num(), 0, Corrupt));
	TestEqual(TEXT("hash mismatch rejected"), ReplyCode(BadChunk), FString(TEXT("hash_mismatch")));

	// --- chunk: correct bytes complete the asset (retry after mismatch). ---
	TSharedPtr<FJsonObject> GoodChunk = Disp->Dispatch(
		MakeChunk(Session, UploadId, TEXT("asset"), TEXT("mesh.STL"), MeshBytes.Num(), 0, MeshBytes));
	bool bMeshComplete = false;
	GoodChunk->TryGetBoolField(TEXT("complete"), bMeshComplete);
	TestTrue(TEXT("mesh chunk completes after retry"), bMeshComplete);

	// --- re-manifest: both blobs now cached -> nothing needed (dedup). ---
	TSharedPtr<FJsonObject> ReMan = Disp->Dispatch(MakeManifest(Session, XmlSha,
		{{TEXT("mesh.STL"), MeshSha}}, {MeshBytes.Num()}, XmlBytes.Num() + MeshBytes.Num()));
	bool bNeedXml2 = true;
	ReMan->TryGetBoolField(TEXT("need_xml"), bNeedXml2);
	const TArray<TSharedPtr<FJsonValue>>* Need2 = nullptr;
	ReMan->TryGetArrayField(TEXT("need_assets"), Need2);
	TestFalse(TEXT("cache hit: need_xml false"), bNeedXml2);
	TestTrue(TEXT("cache hit: need_assets empty"), Need2 && Need2->Num() == 0);

	// --- manifest: path-traversal asset name is rejected. ---
	TSharedPtr<FJsonObject> Trav = Disp->Dispatch(MakeManifest(Session, XmlSha,
		{{TEXT("../evil.STL"), MeshSha}}, {1}, 1));
	TestEqual(TEXT("path traversal rejected"), ReplyCode(Trav), FString(TEXT("bad_request")));

	TSharedPtr<FJsonObject> Abs = Disp->Dispatch(MakeManifest(Session, XmlSha,
		{{TEXT("C:\\evil.STL"), MeshSha}}, {1}, 1));
	TestEqual(TEXT("drive-letter name rejected"), ReplyCode(Abs), FString(TEXT("bad_request")));

	// --- chunk: oversize (total beyond max_asset_bytes) -> payload_too_large. ---
	TSharedPtr<FJsonObject> Oversize = Disp->Dispatch(MakeChunk(Session, UploadId,
		TEXT("asset"), TEXT("mesh.STL"), static_cast<int64>(MaxAsset) + 1, 0, {0x00}));
	TestEqual(TEXT("oversize chunk -> payload_too_large"),
		ReplyCode(Oversize), FString(TEXT("payload_too_large")));

	// --- chunk: unknown upload_id -> unknown_upload. ---
	TSharedPtr<FJsonObject> Unknown = Disp->Dispatch(MakeChunk(Session,
		FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphens),
		TEXT("xml"), TEXT(""), XmlBytes.Num(), 0, XmlBytes));
	TestEqual(TEXT("unknown upload_id -> unknown_upload"),
		ReplyCode(Unknown), FString(TEXT("unknown_upload")));

	// --- commit: unknown upload_id -> unknown_upload. ---
	TSharedPtr<FJsonObject> CommitReq = MakeShared<FJsonObject>();
	CommitReq->SetStringField(TEXT("op"), TEXT("upload_model_commit"));
	CommitReq->SetStringField(TEXT("session_id"), Session);
	CommitReq->SetStringField(TEXT("upload_id"),
		FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphens));
	TSharedPtr<FJsonObject> CommitReply = Disp->Dispatch(CommitReq);
	TestEqual(TEXT("commit unknown upload_id -> unknown_upload"),
		ReplyCode(CommitReply), FString(TEXT("unknown_upload")));

	// Hygiene: the dispatcher writes verified blobs into the shared process
	// cache. Drop the two we seeded so the test leaves no residue.
	{
		FURLabAssetCache& Cache = FURLabAssetCache::Get();
		FString P;
		if (Cache.GetPath(XmlSha, P))
			IFileManager::Get().Delete(*P, /*RequireExists=*/false, /*EvenReadOnly=*/true);
		if (Cache.GetPath(MeshSha, P))
			IFileManager::Get().Delete(*P, /*RequireExists=*/false, /*EvenReadOnly=*/true);
	}

	Server->Stop();
	Server->RemoveFromRoot();
	return true;
}
