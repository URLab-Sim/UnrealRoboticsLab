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

// Network model upload: a remote client ships raw MJCF XML bytes plus the asset
// bytes the XML references over ZMQ; the server materialises them to a temp dir
// and runs the EXISTING import_xml editor job on the result (no separate compile
// path). Three ops:
//   upload_model_manifest  content-addressed negotiation (which blobs are missing)
//   upload_model_chunk     bulk, chunked bytes; each completed blob is verified + cached
//   upload_model_commit    materialise + invoke import_xml + echo model dims/mjb
// The manifest/chunk ops are pure data staging on the RPC worker thread; only
// commit drives the editor import (via the shared op registry, so no URLabEditor
// link dependency). Assets reuse the vfs_assets msgpack-bin framing and the
// bare-filename flatten convention of the DOWN handshake.

#include "Bridge/RpcDispatcher.h"
#include "Bridge/RpcErrorCodes.h"
#include "Bridge/AssetCache.h"
#include "Bridge/OpRegistry.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "Bridge/MsgpackHelpers.h"
#include "Utils/URLabLogging.h"

#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/Base64.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "Misc/ScopeLock.h"

namespace
{
// Security caps (plan 4.6). Advertised in the manifest reply and enforced on
// every chunk.
constexpr int64 kMaxAssetBytes = 64ll * 1024 * 1024;  // 64 MiB per blob
constexpr int64 kMaxTotalBytes = 512ll * 1024 * 1024; // 512 MiB per upload
constexpr int32 kMaxAssets = 4096;
constexpr int32 kMaxOpenUploads = 16;
constexpr double kUploadTtlSeconds = 300.0;

/** One asset's staging slot. The receive buffer only exists while incomplete;
 *  once verified it is written to the content-addressed cache and dropped, so
 *  materialisation reads the blob back from the cache by hash. */
struct FAssetSlot
{
	FString Sha;
	int64 DeclaredSize = 0;
	TArray<uint8> Buffer;
	int64 Received = 0;
	bool bComplete = false;
};

struct FStagingEntry
{
	double LastTouch = 0.0;
	FString StepMode;

	FString XmlSha;
	TArray<uint8> XmlBuffer;
	int64 XmlReceived = 0;
	int64 XmlTotal = 0;
	bool bXmlComplete = false;

	// keyed by bare filename
	TMap<FString, FAssetSlot> Assets;
};

/** Process-wide staging store. One editor process is one instance, so uploads
 *  are per-process; the map is keyed by upload_id (FGuid) and guarded by a
 *  mutex. Concurrent-upload cap + TTL expiry live here. */
class FUploadStaging
{
public:
	static FUploadStaging& Get()
	{
		static FUploadStaging Instance;
		return Instance;
	}

	FCriticalSection Mutex;
	TMap<FString, FStagingEntry> Entries;

	/** Drop entries idle past the TTL. Caller holds Mutex. */
	void SweepExpired()
	{
		const double Now = FPlatformTime::Seconds();
		for (auto It = Entries.CreateIterator(); It; ++It)
		{
			if (Now - It->Value.LastTouch > kUploadTtlSeconds)
				It.RemoveCurrent();
		}
	}
};

/** True if Name is a bare filename safe to materialise next to the XML: no
 *  path separators, no parent refs, no drive letter, no leading separator,
 *  no NUL. */
bool IsBareFilename(const FString& Name)
{
	if (Name.IsEmpty() || Name == TEXT("."))
		return false;
	if (Name.Contains(TEXT("/")) || Name.Contains(TEXT("\\")))
		return false;
	if (Name.Contains(TEXT("..")))
		return false;
	// Drive letter (C:) or any embedded colon (alternate data stream / drive).
	if (Name.Contains(TEXT(":")))
		return false;
	for (const TCHAR C : Name)
	{
		if (C == TEXT('\0'))
			return false;
	}
	return true;
}

/** Read a msgpack-bin request field. On the wire a bin map-value arrives with
 *  a `__b64__`-suffixed key holding base64 (see MsgpackHelpers); accept a plain
 *  base64 string under the bare name too (JSON clients / tests). */
bool ReadBinField(const TSharedPtr<FJsonObject>& Req, const FString& Name, TArray<uint8>& Out)
{
	FString B64;
	if (Req->TryGetStringField(Name + TEXT("__b64__"), B64) || Req->TryGetStringField(Name, B64))
	{
		Out.Reset();
		return FBase64::Decode(B64, Out);
	}
	return false;
}
} // namespace

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleUploadModelManifest(const TSharedPtr<FJsonObject>& Req)
{
	FString XmlSha;
	if (!Req->TryGetStringField(TEXT("xml_sha256"), XmlSha)
		|| !FURLabAssetCache::IsValidSha256Hex(XmlSha))
	{
		return MakeError(URLabError::BadRequest,
			TEXT("upload_model_manifest requires a valid hex 'xml_sha256'"));
	}

	const TArray<TSharedPtr<FJsonValue>>* AssetsArr = nullptr;
	Req->TryGetArrayField(TEXT("assets"), AssetsArr);
	const int32 AssetCount = AssetsArr ? AssetsArr->Num() : 0;
	if (AssetCount > kMaxAssets)
	{
		return MakeError(URLabError::BadRequest,
			FString::Printf(TEXT("manifest lists %d assets; cap is %d"), AssetCount, kMaxAssets));
	}

	double TotalBytes = 0.0;
	Req->TryGetNumberField(TEXT("total_bytes"), TotalBytes);
	if (static_cast<int64>(TotalBytes) > kMaxTotalBytes)
	{
		return MakeError(TEXT("payload_too_large"),
			FString::Printf(TEXT("total_bytes %.0f exceeds max_total_bytes %lld"),
				TotalBytes, kMaxTotalBytes));
	}

	FString StepMode;
	Req->TryGetStringField(TEXT("step_mode"), StepMode);

	FURLabAssetCache& Cache = FURLabAssetCache::Get();

	FStagingEntry Entry;
	Entry.StepMode = StepMode;
	Entry.XmlSha = XmlSha;
	Entry.bXmlComplete = Cache.Has(XmlSha);

	TArray<TSharedPtr<FJsonValue>> NeedAssets;
	for (int32 i = 0; i < AssetCount; ++i)
	{
		const TSharedPtr<FJsonObject>* AObj = nullptr;
		if (!(*AssetsArr)[i]->TryGetObject(AObj) || !AObj)
		{
			return MakeError(URLabError::BadRequest,
				FString::Printf(TEXT("manifest asset[%d] is not an object"), i));
		}

		FString Name, Sha;
		(*AObj)->TryGetStringField(TEXT("name"), Name);
		(*AObj)->TryGetStringField(TEXT("sha256"), Sha);
		double Size = 0.0;
		(*AObj)->TryGetNumberField(TEXT("size"), Size);

		if (!IsBareFilename(Name))
		{
			return MakeError(URLabError::BadRequest,
				FString::Printf(TEXT("asset name '%s' is not a bare filename"), *Name));
		}
		if (!FURLabAssetCache::IsValidSha256Hex(Sha))
		{
			return MakeError(URLabError::BadRequest,
				FString::Printf(TEXT("asset '%s' has an invalid sha256"), *Name));
		}
		if (static_cast<int64>(Size) > kMaxAssetBytes)
		{
			return MakeError(TEXT("payload_too_large"),
				FString::Printf(TEXT("asset '%s' size %.0f exceeds max_asset_bytes %lld"),
					*Name, Size, kMaxAssetBytes));
		}

		FAssetSlot Slot;
		Slot.Sha = Sha;
		Slot.DeclaredSize = static_cast<int64>(Size);
		Slot.bComplete = Cache.Has(Sha);
		if (!Slot.bComplete)
			NeedAssets.Add(MakeShared<FJsonValueString>(Name));
		Entry.Assets.Add(Name, MoveTemp(Slot));
	}

	const FString UploadId = FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphens);
	Entry.LastTouch = FPlatformTime::Seconds();

	{
		FUploadStaging& Staging = FUploadStaging::Get();
		FScopeLock Lock(&Staging.Mutex);
		Staging.SweepExpired();
		if (Staging.Entries.Num() >= kMaxOpenUploads)
		{
			return MakeError(URLabError::BadRequest,
				FString::Printf(TEXT("too many concurrent uploads (cap %d); retry later"),
					kMaxOpenUploads));
		}
		Staging.Entries.Add(UploadId, MoveTemp(Entry));
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("upload_model_manifest_ok"));
	Reply->SetStringField(TEXT("upload_id"), UploadId);
	Reply->SetBoolField(TEXT("need_xml"), !Cache.Has(XmlSha));
	Reply->SetArrayField(TEXT("need_assets"), NeedAssets);
	Reply->SetNumberField(TEXT("max_asset_bytes"), static_cast<double>(kMaxAssetBytes));
	Reply->SetNumberField(TEXT("max_total_bytes"), static_cast<double>(kMaxTotalBytes));
	return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleUploadModelChunk(const TSharedPtr<FJsonObject>& Req)
{
	// The reassembled blob is verified against the manifest-declared hash, not
	// the chunk's own sha256 field, so that field is accepted but not consulted.
	FString UploadId, Kind, Name;
	Req->TryGetStringField(TEXT("upload_id"), UploadId);
	Req->TryGetStringField(TEXT("kind"), Kind);
	Req->TryGetStringField(TEXT("name"), Name);

	double OffsetD = 0.0, TotalD = 0.0;
	Req->TryGetNumberField(TEXT("offset"), OffsetD);
	Req->TryGetNumberField(TEXT("total"), TotalD);
	const int64 Offset = static_cast<int64>(OffsetD);
	const int64 Total = static_cast<int64>(TotalD);

	if (UploadId.IsEmpty() || (Kind != TEXT("xml") && Kind != TEXT("asset")))
	{
		return MakeError(URLabError::BadRequest,
			TEXT("upload_model_chunk requires 'upload_id' and kind 'xml'|'asset'"));
	}
	if (Offset < 0 || Total < 0)
	{
		return MakeError(URLabError::BadRequest, TEXT("offset/total must be non-negative"));
	}
	if (Total > kMaxAssetBytes)
	{
		return MakeError(TEXT("payload_too_large"),
			FString::Printf(TEXT("blob total %lld exceeds max_asset_bytes %lld"),
				Total, kMaxAssetBytes));
	}

	TArray<uint8> Data;
	if (!ReadBinField(Req, TEXT("data"), Data))
	{
		return MakeError(URLabError::BadRequest, TEXT("upload_model_chunk missing binary 'data'"));
	}
	if (Offset + Data.Num() > Total)
	{
		return MakeError(URLabError::BadRequest,
			TEXT("chunk offset+len exceeds declared total"));
	}
	if (Offset + Data.Num() > kMaxAssetBytes)
	{
		return MakeError(TEXT("payload_too_large"),
			FString::Printf(TEXT("chunk end %lld exceeds max_asset_bytes %lld"),
				Offset + Data.Num(), kMaxAssetBytes));
	}

	FURLabAssetCache& Cache = FURLabAssetCache::Get();
	FUploadStaging& Staging = FUploadStaging::Get();
	FScopeLock Lock(&Staging.Mutex);

	FStagingEntry* Entry = Staging.Entries.Find(UploadId);
	if (!Entry || FPlatformTime::Seconds() - Entry->LastTouch > kUploadTtlSeconds)
	{
		if (Entry)
			Staging.Entries.Remove(UploadId);
		return MakeError(TEXT("unknown_upload"),
			FString::Printf(TEXT("upload_id '%s' is unknown or expired"), *UploadId));
	}
	Entry->LastTouch = FPlatformTime::Seconds();

	// Route to the XML slot or a named asset slot.
	TArray<uint8>* Buffer = nullptr;
	int64* Received = nullptr;
	bool* bComplete = nullptr;
	FString DeclaredSha;
	int64* XmlTotalPtr = nullptr;

	if (Kind == TEXT("xml"))
	{
		Buffer = &Entry->XmlBuffer;
		Received = &Entry->XmlReceived;
		bComplete = &Entry->bXmlComplete;
		DeclaredSha = Entry->XmlSha;
		XmlTotalPtr = &Entry->XmlTotal;
	}
	else
	{
		FAssetSlot* Slot = Entry->Assets.Find(Name);
		if (!Slot)
		{
			return MakeError(URLabError::BadRequest,
				FString::Printf(TEXT("asset '%s' was not declared in the manifest"), *Name));
		}
		if (Slot->DeclaredSize != Total)
		{
			return MakeError(URLabError::BadRequest,
				FString::Printf(TEXT("asset '%s' chunk total %lld != manifest size %lld"),
					*Name, Total, Slot->DeclaredSize));
		}
		Buffer = &Slot->Buffer;
		Received = &Slot->Received;
		bComplete = &Slot->bComplete;
		DeclaredSha = Slot->Sha;
	}

	if (XmlTotalPtr)
		*XmlTotalPtr = Total;

	// Already satisfied (e.g. resolved from the cache at manifest time): ack.
	if (*bComplete && Buffer->Num() == 0)
	{
		TSharedPtr<FJsonObject> Ack = MakeShared<FJsonObject>();
		Ack->SetStringField(TEXT("op"), TEXT("upload_model_chunk_ok"));
		Ack->SetStringField(TEXT("name"), Kind == TEXT("xml") ? TEXT("<xml>") : Name);
		Ack->SetNumberField(TEXT("received"), static_cast<double>(Total));
		Ack->SetBoolField(TEXT("complete"), true);
		return Ack;
	}

	const int32 TotalI = static_cast<int32>(Total); // <= kMaxAssetBytes, fits int32
	if (Buffer->Num() != TotalI)
	{
		Buffer->SetNumZeroed(TotalI);
		*Received = 0;
	}
	if (Data.Num() > 0)
		FMemory::Memcpy(Buffer->GetData() + Offset, Data.GetData(), Data.Num());
	*Received += Data.Num();

	bool bJustCompleted = false;
	if (*Received >= Total)
	{
		const FString ActualSha = FURLabAssetCache::Sha256Hex(*Buffer);
		if (ActualSha != DeclaredSha)
		{
			// Reset the slot so the client can retry cleanly.
			Buffer->Reset();
			*Received = 0;
			return MakeError(TEXT("hash_mismatch"),
				FString::Printf(TEXT("%s sha256 %s does not match declared %s"),
					*(Kind == TEXT("xml") ? FString(TEXT("xml")) : Name), *ActualSha, *DeclaredSha));
		}
		Cache.Put(DeclaredSha, *Buffer);
		Buffer->Empty(); // blob now lives in the cache; free staging memory
		*bComplete = true;
		bJustCompleted = true;
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("upload_model_chunk_ok"));
	Reply->SetStringField(TEXT("name"), Kind == TEXT("xml") ? TEXT("<xml>") : Name);
	Reply->SetNumberField(TEXT("received"),
		static_cast<double>(bJustCompleted ? Total : *Received));
	Reply->SetBoolField(TEXT("complete"), *bComplete);
	return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleUploadModelCommit(const TSharedPtr<FJsonObject>& Req)
{
	FString UploadId;
	Req->TryGetStringField(TEXT("upload_id"), UploadId);
	if (UploadId.IsEmpty())
		return MakeError(URLabError::BadRequest, TEXT("upload_model_commit requires 'upload_id'"));

	// Snapshot the staging entry (xml sha, asset name->sha, step mode) under the
	// lock, verify completeness, then release before the slow materialise+import.
	FString XmlSha, StepMode;
	TMap<FString, FString> AssetShaByName;
	{
		FUploadStaging& Staging = FUploadStaging::Get();
		FScopeLock Lock(&Staging.Mutex);
		FStagingEntry* Entry = Staging.Entries.Find(UploadId);
		if (!Entry || FPlatformTime::Seconds() - Entry->LastTouch > kUploadTtlSeconds)
		{
			if (Entry)
				Staging.Entries.Remove(UploadId);
			return MakeError(TEXT("unknown_upload"),
				FString::Printf(TEXT("upload_id '%s' is unknown or expired"), *UploadId));
		}
		if (!Entry->bXmlComplete)
			return MakeError(URLabError::BadRequest, TEXT("commit before the xml blob completed"));
		for (const TPair<FString, FAssetSlot>& Kv : Entry->Assets)
		{
			if (!Kv.Value.bComplete)
			{
				return MakeError(URLabError::BadRequest,
					FString::Printf(TEXT("commit before asset '%s' completed"), *Kv.Key));
			}
			AssetShaByName.Add(Kv.Key, Kv.Value.Sha);
		}
		XmlSha = Entry->XmlSha;
		StepMode = Entry->StepMode;
		Entry->LastTouch = FPlatformTime::Seconds();
	}

	// Materialise to a per-process temp working dir. Keyed by upload_id under the
	// instance's own ProjectIntermediateDir so farm instances never collide.
	FURLabAssetCache& Cache = FURLabAssetCache::Get();
	const FString WorkDir = FPaths::Combine(FPaths::ProjectIntermediateDir(),
		TEXT("URLabUpload"), UploadId);
	IFileManager::Get().MakeDirectory(*WorkDir, /*Tree=*/true);

	auto Cleanup = [&WorkDir, &UploadId]() {
		IFileManager::Get().DeleteDirectory(*WorkDir, /*RequireExists=*/false, /*Tree=*/true);
		FUploadStaging& Staging = FUploadStaging::Get();
		FScopeLock Lock(&Staging.Mutex);
		Staging.Entries.Remove(UploadId);
	};

	auto CopyBlob = [&Cache](const FString& Sha, const FString& Dest, FString& OutErr) -> bool {
		FString SrcPath;
		if (!Cache.GetPath(Sha, SrcPath))
		{
			OutErr = FString::Printf(TEXT("blob %s missing from cache"), *Sha);
			return false;
		}
		if (IFileManager::Get().Copy(*Dest, *SrcPath) != COPY_OK)
		{
			OutErr = FString::Printf(TEXT("failed to materialise %s"), *Dest);
			return false;
		}
		return true;
	};

	const FString XmlPath = FPaths::Combine(WorkDir, TEXT("model.xml"));
	FString CopyErr;
	if (!CopyBlob(XmlSha, XmlPath, CopyErr))
	{
		Cleanup();
		return MakeError(TEXT("import_failed"), CopyErr);
	}
	for (const TPair<FString, FString>& Kv : AssetShaByName)
	{
		const FString Dest = FPaths::Combine(WorkDir, Kv.Key);
		if (!CopyBlob(Kv.Value, Dest, CopyErr))
		{
			Cleanup();
			return MakeError(TEXT("import_failed"), CopyErr);
		}
	}

	// Run the EXISTING import_xml editor job on the materialised temp XML. It is
	// registered (by URLabEditor) as an async game-thread job returning
	// op_started + job_id; we drive it to completion by polling op_status, the
	// same contract a remote client uses. No separate compile path.
	URLabOpRegistry::FHandler ImportFn = URLabOpRegistry::GetHandler(TEXT("import_xml"));
	URLabOpRegistry::FHandler StatusFn = URLabOpRegistry::GetHandler(TEXT("op_status"));
	if (!ImportFn || !StatusFn)
	{
		Cleanup();
		return MakeError(TEXT("import_failed"),
			TEXT("import_xml op is unavailable (editor module not loaded)"));
	}

	TSharedPtr<FJsonObject> ImportReq = MakeShared<FJsonObject>();
	ImportReq->SetStringField(TEXT("op"), TEXT("import_xml"));
	ImportReq->SetStringField(TEXT("path"), FPaths::ConvertRelativePathToFull(XmlPath));
	ImportReq->SetBoolField(TEXT("force_reimport"), true);

	TSharedPtr<FJsonObject> Started = ImportFn(ImportReq);
	FString JobId, StartedOp;
	if (Started.IsValid())
	{
		Started->TryGetStringField(TEXT("op"), StartedOp);
		Started->TryGetStringField(TEXT("job_id"), JobId);
	}

	// A synchronous handler (no async job) returns the terminal reply directly.
	TSharedPtr<FJsonObject> ImportResult;
	if (StartedOp == TEXT("op_started") && !JobId.IsEmpty())
	{
		// Poll op_status until terminal. No hard deadline (imports vary wildly);
		// the client's recv timeout is the operational bound, and a draining
		// bridge aborts us early.
		for (;;)
		{
			if (IsDraining())
			{
				Cleanup();
				return MakeError(URLabError::ShuttingDown,
					TEXT("bridge draining; upload_model_commit abandoned"));
			}
			TSharedPtr<FJsonObject> StatusReq = MakeShared<FJsonObject>();
			StatusReq->SetStringField(TEXT("op"), TEXT("op_status"));
			StatusReq->SetStringField(TEXT("job_id"), JobId);
			TSharedPtr<FJsonObject> Status = StatusFn(StatusReq);
			FString State;
			if (Status.IsValid())
				Status->TryGetStringField(TEXT("state"), State);
			if (State == TEXT("done") || State == TEXT("failed"))
			{
				const TSharedPtr<FJsonObject>* ResObj = nullptr;
				if (Status->TryGetObjectField(TEXT("result"), ResObj) && ResObj)
					ImportResult = *ResObj;
				break;
			}
			if (State.IsEmpty()) // job vanished (unknown_job)
			{
				break;
			}
			FPlatformProcess::Sleep(0.05f);
		}
	}
	else
	{
		ImportResult = Started;
	}

	FString ResultOp, ResultErr;
	if (ImportResult.IsValid())
	{
		ImportResult->TryGetStringField(TEXT("op"), ResultOp);
		ImportResult->TryGetStringField(TEXT("message"), ResultErr);
	}
	if (ResultOp != TEXT("import_xml_ok"))
	{
		Cleanup();
		return MakeError(TEXT("import_failed"),
			ResultErr.IsEmpty() ? TEXT("import_xml did not complete") : ResultErr);
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("upload_model_commit_ok"));
	Reply->SetBoolField(TEXT("imported"), true);
	{
		FString ClassPath, ShortName;
		ImportResult->TryGetStringField(TEXT("blueprint_class_path"), ClassPath);
		ImportResult->TryGetStringField(TEXT("blueprint_short_name"), ShortName);
		if (!ClassPath.IsEmpty())
			Reply->SetStringField(TEXT("blueprint_class_path"), ClassPath);
		if (!ShortName.IsEmpty())
			Reply->SetStringField(TEXT("blueprint_short_name"), ShortName);
	}

	// Echo model dims + mjb when a live manager already holds a compiled model
	// (i.e. a scene is running). A fresh import only produces the Blueprint
	// asset; standing up a live model requires PIE + spawn, which the client
	// drives after commit. Absent a live model these fields are omitted.
	if (!StepMode.IsEmpty())
	{
		EStepMode Mode = EStepMode::Auto;
		bool bHaveMode = true;
		if (StepMode == TEXT("live"))
			Mode = EStepMode::Live;
		else if (StepMode == TEXT("direct"))
			Mode = EStepMode::Direct;
		else if (StepMode == TEXT("puppet"))
			Mode = EStepMode::Puppet;
		else
			bHaveMode = false;
		if (bHaveMode && OwnerMgr.IsValid())
			SetActiveStepMode(Mode);
	}

	if (AAMjManager* Mgr = OwnerMgr.Get())
	{
		if (Mgr->PhysicsEngine)
		{
			if (mjModel* m = Mgr->PhysicsEngine->GetModel())
			{
				Reply->SetNumberField(TEXT("nq"), m->nq);
				Reply->SetNumberField(TEXT("nv"), m->nv);
				Reply->SetNumberField(TEXT("nu"), m->nu);
				Reply->SetNumberField(TEXT("nbody"), m->nbody);
				Reply->SetNumberField(TEXT("ngeom"), m->ngeom);
				const int Sz = mj_sizeModel(m);
				TArray<uint8> Buf;
				Buf.SetNum(Sz);
				mj_saveModel(m, nullptr, Buf.GetData(), Sz);
				FURLabMsgpackUtil::SetBinaryField(Reply, TEXT("mjb"), Buf.GetData(), Sz);
			}
		}
	}
	Reply->SetArrayField(TEXT("warnings"), TArray<TSharedPtr<FJsonValue>>());

	UE_LOG(LogURLabNet, Log, TEXT("upload_model_commit: imported %s (%d assets)"),
		*UploadId, AssetShaByName.Num());

	// Opportunistic LRU eviction (no-op unless URLAB_ASSET_CACHE_MAX_BYTES is set).
	Cache.EvictToBudget();

	Cleanup();
	return Reply;
}
