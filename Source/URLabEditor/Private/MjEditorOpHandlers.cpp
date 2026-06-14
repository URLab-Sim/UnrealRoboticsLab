// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MjEditorOpHandlers.h"
#include "MjLevelOps.h"

#include "Editor.h"
#include "EngineUtils.h"
#include "LevelEditorSubsystem.h"
#include "LevelEditor.h"
#include "ILevelEditor.h"
#include "IAssetViewport.h"
#include "SLevelViewport.h"
#include "Settings/LevelEditorPlaySettings.h"
#include "Async/Async.h"
#include "Containers/Ticker.h"
#include "DrawDebugHelpers.h"
#include "EditorViewportClient.h"
#include "Engine/Engine.h"
#include "HAL/Event.h"
#include "HAL/PlatformProcess.h"
#include "Kismet/GameplayStatics.h"
#include "LevelEditorViewport.h"

#include "MuJoCo/Utils/MjUtils.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeServerProvider.h"
#include "Bridge/OpRegistry.h"
#include "Bridge/OpHelpers.h"
#include "Bridge/RpcDispatcher.h"

namespace
{
using URLabOpHelpers::ReadRotation;
using URLabOpHelpers::ReadVec3;
using URLabOpHelpers::ResolveActorKey;

// Local alias so existing call sites can keep saying MakeJsonError(...).
inline TSharedPtr<FJsonObject> MakeJsonError(const FString& Code,
	const FString& Message)
{
	return URLabOpHelpers::MakeError(Code, Message);
}

/** Run an editor op body on the game thread synchronously.
 *
 *  GEditor / asset-manipulation calls require the game thread. The
 *  dispatcher runs on a ZMQ worker thread, so we hop over to the
 *  game thread and poll until done. Polling (vs blocking wait) lets
 *  us exit early when the dispatcher is draining.
 *
 *  Marshalling is done via FTSTicker, not AsyncTask, deliberately:
 *  AsyncTask schedules a task-graph task, and any handler that calls
 *  back into the task graph (notably ObjectTools::ForceDeleteObjects
 *  on a UWorld, which triggers NewMap -> EndPlayMap -> ProcessTasksUntilIdle)
 *  recurses into the outer task-graph processing and asserts. Tickers
 *  run during FEngineLoop::Tick, outside the task-graph processing
 *  call stack, so nested ProcessTasksUntilIdle is safe. */
template <typename Lambda>
TSharedPtr<FJsonObject> RunOnGameThreadSync(Lambda&& Body)
{
	if (IsInGameThread())
	{
		return Body();
	}

	struct FState
	{
		TSharedPtr<FJsonObject> Result;
		std::atomic<bool> bDone{false};
	};
	TSharedRef<FState> State = MakeShared<FState>();

	// FTSTicker callback runs on game thread during the next engine
	// tick, NOT under task-graph processing. Returning false removes
	// the one-shot ticker.
	FTSTicker::GetCoreTicker().AddTicker(
		FTickerDelegate::CreateLambda(
			[State, BodyCopy = Forward<Lambda>(Body)](float /*DeltaTime*/) mutable -> bool {
				State->Result = BodyCopy();
				State->bDone.store(true, std::memory_order_release);
				return false;
			}));

	UURLabBridgeServer* BridgeServer = URLabBridgeProvider::ResolveEditorServer();
	FURLabRpcDispatcher* Dispatcher = BridgeServer ? BridgeServer->GetDispatcher() : nullptr;

	// No server-side deadline. Editor ops vary wildly in duration
	// (import_xml shells out to clean_meshes.py + drives the BP
	// factory, create_level touches the asset registry, etc.) and
	// returning a `timeout` reply while the game-thread ticker is
	// still in flight makes the client retry -- the retry races the
	// in-flight op and FactoryCreateFile asserts on a duplicate BP.
	// Block until the op completes or the bridge is asked to drain
	// (server shutdown / editor close). The python client's recv
	// timeout is the operational deadline.
	while (!State->bDone.load(std::memory_order_acquire))
	{
		if (Dispatcher && Dispatcher->IsDraining())
		{
			return URLabOpHelpers::MakeError(TEXT("shutting_down"),
				TEXT("editor op aborted: dispatcher draining"));
		}
		FPlatformProcess::Sleep(0.05f);
	}
	return State->Result;
}

// ---- import_xml --------------------------------------------------------
TSharedPtr<FJsonObject> HandleImportXml(const TSharedPtr<FJsonObject>& Req)
{
	FString Path;
	if (!Req->TryGetStringField(TEXT("path"), Path) || Path.IsEmpty())
	{
		return MakeJsonError(TEXT("missing_field"),
			TEXT("import_xml requires a non-empty 'path'"));
	}
	bool bForceReimport = false;
	Req->TryGetBoolField(TEXT("force_reimport"), bForceReimport);

	FString ClassPath, ShortName, Err;
	bool bImportedNow = false;
	const bool bOk = URLabLevelOps::ImportXmlSync(
		Path, bForceReimport, ClassPath, ShortName, bImportedNow, Err);
	if (!bOk)
	{
		return MakeJsonError(TEXT("import_failed"), Err);
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("import_xml_ok"));
	Reply->SetStringField(TEXT("blueprint_class_path"), ClassPath);
	Reply->SetStringField(TEXT("blueprint_short_name"), ShortName);
	Reply->SetBoolField(TEXT("imported_now"), bImportedNow);
	return Reply;
}

// ---- create_level / load_level / save_level ----------------------------

TSharedPtr<FJsonObject> HandleCreateLevel(const TSharedPtr<FJsonObject>& Req)
{
	FString Name;
	if (!Req->TryGetStringField(TEXT("name"), Name) || Name.IsEmpty())
	{
		return MakeJsonError(TEXT("missing_field"),
			TEXT("create_level requires non-empty 'name'"));
	}
	bool bForceOverwrite = false;
	Req->TryGetBoolField(TEXT("force_overwrite"), bForceOverwrite);
	FString OutPath, Err;
	if (!URLabLevelOps::CreateLevelSync(Name, bForceOverwrite, OutPath, Err))
	{
		return MakeJsonError(TEXT("create_level_failed"), Err);
	}
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("create_level_ok"));
	Reply->SetStringField(TEXT("level_path"), OutPath);
	return Reply;
}

TSharedPtr<FJsonObject> HandleCurrentLevel(const TSharedPtr<FJsonObject>& /*Req*/)
{
	if (!GEditor)
		return MakeJsonError(TEXT("not_in_editor"), TEXT("GEditor null"));
	UWorld* World = GEditor->GetEditorWorldContext().World();
	if (!World)
		return MakeJsonError(TEXT("no_world"), TEXT("editor world unavailable"));

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("current_level_ok"));
	Reply->SetStringField(TEXT("level_path"), World->GetOutermost()->GetName());
	return Reply;
}

TSharedPtr<FJsonObject> HandleDestroyAsset(const TSharedPtr<FJsonObject>& Req)
{
	FString AssetPath;
	if (!Req->TryGetStringField(TEXT("asset_path"), AssetPath) || AssetPath.IsEmpty())
	{
		return MakeJsonError(TEXT("missing_field"),
			TEXT("destroy_asset requires non-empty 'asset_path'"));
	}
	bool bWasFound = false;
	FString Err;
	if (!URLabLevelOps::DestroyAssetSync(AssetPath, bWasFound, Err))
	{
		return MakeJsonError(TEXT("destroy_asset_failed"), Err);
	}
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("destroy_asset_ok"));
	Reply->SetStringField(TEXT("asset_path"), AssetPath);
	Reply->SetBoolField(TEXT("was_found"), bWasFound);
	return Reply;
}

TSharedPtr<FJsonObject> HandleEnsureManager(const TSharedPtr<FJsonObject>& /*Req*/)
{
	if (!GEditor)
		return MakeJsonError(TEXT("not_in_editor"), TEXT("GEditor null"));
	UWorld* World = GEditor->GetEditorWorldContext().World();
	if (!World)
		return MakeJsonError(TEXT("no_world"), TEXT("editor world unavailable"));

	AAMjManager* Existing = nullptr;
	for (TActorIterator<AAMjManager> It(World); It; ++It)
	{
		Existing = *It;
		break;
	}
	const bool bWasExisting = (Existing != nullptr);

	if (!Existing)
	{
		FActorSpawnParameters Params;
		Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		Existing = World->SpawnActor<AAMjManager>(
			AAMjManager::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, Params);
		if (!Existing)
		{
			return MakeJsonError(TEXT("spawn_failed"),
				TEXT("SpawnActor returned null for AAMjManager"));
		}
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("ensure_manager_ok"));
	Reply->SetStringField(TEXT("actor_name"), Existing->GetName());
	Reply->SetStringField(TEXT("actor_path"), Existing->GetPathName());
	Reply->SetBoolField(TEXT("was_existing"), bWasExisting);
	return Reply;
}

TSharedPtr<FJsonObject> HandleLoadLevel(const TSharedPtr<FJsonObject>& Req)
{
	FString Path;
	// Accept either 'level_path' (full /Game/...) or 'name'.
	Req->TryGetStringField(TEXT("level_path"), Path);
	if (Path.IsEmpty())
		Req->TryGetStringField(TEXT("name"), Path);
	if (Path.IsEmpty())
	{
		return MakeJsonError(TEXT("missing_field"),
			TEXT("load_level requires 'level_path' or 'name'"));
	}
	FString OutPath, Err;
	if (!URLabLevelOps::LoadLevelSync(Path, OutPath, Err))
	{
		return MakeJsonError(TEXT("load_level_failed"), Err);
	}
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("load_level_ok"));
	Reply->SetStringField(TEXT("level_path"), OutPath);
	return Reply;
}

TSharedPtr<FJsonObject> HandleSaveLevel(const TSharedPtr<FJsonObject>& /*Req*/)
{
	FString OutPath, Err;
	if (!URLabLevelOps::SaveCurrentLevelSync(OutPath, Err))
	{
		return MakeJsonError(TEXT("save_level_failed"), Err);
	}
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("save_level_ok"));
	Reply->SetStringField(TEXT("level_path"), OutPath);
	return Reply;
}

// ---- spawn_actor / destroy_actor ---------------------------------------

TSharedPtr<FJsonObject> HandleSpawnActor(const TSharedPtr<FJsonObject>& Req)
{
	FString Blueprint;
	if (!Req->TryGetStringField(TEXT("blueprint"), Blueprint) || Blueprint.IsEmpty())
	{
		return MakeJsonError(TEXT("missing_field"),
			TEXT("spawn_actor requires non-empty 'blueprint'"));
	}
	FString ActorId;
	Req->TryGetStringField(TEXT("actor_id"), ActorId);

	FVector Loc, Scale;
	ReadVec3(Req, TEXT("location"), Loc, FVector::ZeroVector);
	ReadVec3(Req, TEXT("scale"), Scale, FVector::OneVector);
	FQuat Quat;
	ReadRotation(Req, Quat);

	FString ActorName, ActorPath, BPClassPath, Err;
	bool bWasExisting = false;
	const bool bOk = URLabLevelOps::SpawnActorSync(
		Blueprint, ActorId, Loc, Quat, Scale,
		ActorName, ActorPath, BPClassPath, bWasExisting, Err);
	if (!bOk)
	{
		return MakeJsonError(TEXT("spawn_failed"), Err);
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("spawn_actor_ok"));
	Reply->SetStringField(TEXT("actor_id"), ActorId);
	Reply->SetStringField(TEXT("actor_name"), ActorName);
	Reply->SetStringField(TEXT("actor_path"), ActorPath);
	Reply->SetStringField(TEXT("blueprint_class_path"), BPClassPath);
	Reply->SetBoolField(TEXT("was_existing"), bWasExisting);
	// Echo back the requested transform for the Python dataclass.
	TArray<TSharedPtr<FJsonValue>> LocOut;
	LocOut.Add(MakeShared<FJsonValueNumber>(Loc.X / 100.0));  // back to MJ metres
	LocOut.Add(MakeShared<FJsonValueNumber>(-Loc.Y / 100.0)); // re-flip Y
	LocOut.Add(MakeShared<FJsonValueNumber>(Loc.Z / 100.0));
	Reply->SetArrayField(TEXT("location"), LocOut);
	TArray<TSharedPtr<FJsonValue>> QuatOut;
	QuatOut.Add(MakeShared<FJsonValueNumber>(Quat.X));
	QuatOut.Add(MakeShared<FJsonValueNumber>(Quat.Y));
	QuatOut.Add(MakeShared<FJsonValueNumber>(Quat.Z));
	QuatOut.Add(MakeShared<FJsonValueNumber>(Quat.W));
	Reply->SetArrayField(TEXT("rotation_quat"), QuatOut);
	// PIE state echo: requires_pie_restart=true if the editor is mid-PIE,
	// so callers know the change won't take effect this session.
	Reply->SetBoolField(TEXT("requires_pie_restart"),
		GEditor ? GEditor->IsPlayingSessionInEditor() : false);
	return Reply;
}

TSharedPtr<FJsonObject> HandleSpawnGrid(const TSharedPtr<FJsonObject>& Req)
{
	FString Blueprint;
	if (!Req->TryGetStringField(TEXT("blueprint"), Blueprint) || Blueprint.IsEmpty())
		return MakeJsonError(TEXT("missing_field"),
			TEXT("spawn_grid requires non-empty 'blueprint'"));

	FString BaseId;
	if (!Req->TryGetStringField(TEXT("base_actor_id"), BaseId) || BaseId.IsEmpty())
		return MakeJsonError(TEXT("missing_field"),
			TEXT("spawn_grid requires non-empty 'base_actor_id'"));

	int32 CountX = 0, CountY = 0;
	Req->TryGetNumberField(TEXT("count_x"), CountX);
	Req->TryGetNumberField(TEXT("count_y"), CountY);
	if (CountX <= 0 || CountY <= 0)
		return MakeJsonError(TEXT("invalid_count"),
			TEXT("spawn_grid requires count_x > 0 and count_y > 0"));
	// Reasonable upper bound: 1024 cells. Higher and we risk
	// spending several seconds in the dispatcher while the editor
	// hangs. Callers can chunk.
	if (CountX * CountY > 1024)
		return MakeJsonError(TEXT("too_many_cells"),
			FString::Printf(TEXT("spawn_grid capped at 1024 cells (got %d)"),
				CountX * CountY));

	FVector Spacing, Origin, Scale;
	ReadVec3(Req, TEXT("spacing"), Spacing, FVector(1.0, 1.0, 0.0));
	ReadVec3(Req, TEXT("origin"), Origin, FVector::ZeroVector);
	ReadVec3(Req, TEXT("scale"), Scale, FVector::OneVector);
	FQuat Quat;
	ReadRotation(Req, Quat);

	TArray<TSharedPtr<FJsonValue>> Spawned;
	FString FirstBPClassPath;
	for (int32 j = 0; j < CountY; ++j)
	{
		for (int32 i = 0; i < CountX; ++i)
		{
			const FVector CellLoc(
				Origin.X + Spacing.X * (double)i,
				Origin.Y + Spacing.Y * (double)j,
				Origin.Z + Spacing.Z * 0.0);
			const FString ActorId = FString::Printf(TEXT("%s_%d_%d"), *BaseId, i, j);

			FString ActorName, ActorPath, BPClassPath, Err;
			bool bWasExisting = false;
			const bool bOk = URLabLevelOps::SpawnActorSync(
				Blueprint, ActorId, CellLoc, Quat, Scale,
				ActorName, ActorPath, BPClassPath, bWasExisting, Err);
			if (!bOk)
				return MakeJsonError(TEXT("spawn_failed"),
					FString::Printf(TEXT("cell (%d,%d): %s"), i, j, *Err));

			if (FirstBPClassPath.IsEmpty())
				FirstBPClassPath = BPClassPath;

			TSharedPtr<FJsonObject> A = MakeShared<FJsonObject>();
			A->SetStringField(TEXT("actor_id"), ActorId);
			A->SetStringField(TEXT("actor_name"), ActorName);
			A->SetStringField(TEXT("actor_path"), ActorPath);
			A->SetBoolField(TEXT("was_existing"), bWasExisting);
			TArray<TSharedPtr<FJsonValue>> LocOut;
			LocOut.Add(MakeShared<FJsonValueNumber>(CellLoc.X));
			LocOut.Add(MakeShared<FJsonValueNumber>(CellLoc.Y));
			LocOut.Add(MakeShared<FJsonValueNumber>(CellLoc.Z));
			A->SetArrayField(TEXT("location"), LocOut);
			Spawned.Add(MakeShared<FJsonValueObject>(A));
		}
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("spawn_grid_ok"));
	Reply->SetNumberField(TEXT("count"), Spawned.Num());
	Reply->SetStringField(TEXT("blueprint_class_path"), FirstBPClassPath);
	Reply->SetArrayField(TEXT("actors"), Spawned);
	Reply->SetBoolField(TEXT("requires_pie_restart"),
		GEditor ? GEditor->IsPlayingSessionInEditor() : false);
	return Reply;
}

TSharedPtr<FJsonObject> HandleSpawnLight(const TSharedPtr<FJsonObject>& Req)
{
	FString Kind = TEXT("directional");
	Req->TryGetStringField(TEXT("kind"), Kind);

	FString ActorId;
	Req->TryGetStringField(TEXT("actor_id"), ActorId);

	FVector Loc, RotEulerDeg;
	ReadVec3(Req, TEXT("location"), Loc, FVector::ZeroVector);
	ReadVec3(Req, TEXT("rotation_euler"), RotEulerDeg, FVector::ZeroVector);

	double Intensity = 5000.0;
	Req->TryGetNumberField(TEXT("intensity"), Intensity);

	// Color: read as 3-vec (RGB in [0,1]); default white.
	FVector ColorVec(1, 1, 1);
	ReadVec3(Req, TEXT("color"), ColorVec, FVector(1, 1, 1));
	const FLinearColor Color(ColorVec.X, ColorVec.Y, ColorVec.Z, 1.0f);

	FString ActorName, ActorPath, ResolvedKind, Err;
	const bool bOk = URLabLevelOps::SpawnLightSync(
		Kind, ActorId, Loc, RotEulerDeg,
		static_cast<float>(Intensity), Color,
		ActorName, ActorPath, ResolvedKind, Err);
	if (!bOk)
	{
		return MakeJsonError(TEXT("spawn_failed"), Err);
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("spawn_light_ok"));
	Reply->SetStringField(TEXT("actor_id"), ActorId);
	Reply->SetStringField(TEXT("actor_name"), ActorName);
	Reply->SetStringField(TEXT("actor_path"), ActorPath);
	Reply->SetStringField(TEXT("kind"), ResolvedKind);
	Reply->SetNumberField(TEXT("intensity"), Intensity);
	{
		TArray<TSharedPtr<FJsonValue>> RGB;
		RGB.Add(MakeShared<FJsonValueNumber>(Color.R));
		RGB.Add(MakeShared<FJsonValueNumber>(Color.G));
		RGB.Add(MakeShared<FJsonValueNumber>(Color.B));
		Reply->SetArrayField(TEXT("color"), RGB);
		TArray<TSharedPtr<FJsonValue>> LocOut;
		LocOut.Add(MakeShared<FJsonValueNumber>(Loc.X));
		LocOut.Add(MakeShared<FJsonValueNumber>(Loc.Y));
		LocOut.Add(MakeShared<FJsonValueNumber>(Loc.Z));
		Reply->SetArrayField(TEXT("location"), LocOut);
		TArray<TSharedPtr<FJsonValue>> EulerOut;
		EulerOut.Add(MakeShared<FJsonValueNumber>(RotEulerDeg.X));
		EulerOut.Add(MakeShared<FJsonValueNumber>(RotEulerDeg.Y));
		EulerOut.Add(MakeShared<FJsonValueNumber>(RotEulerDeg.Z));
		Reply->SetArrayField(TEXT("rotation_euler"), EulerOut);
	}
	Reply->SetBoolField(TEXT("requires_pie_restart"),
		GEditor ? GEditor->IsPlayingSessionInEditor() : false);
	return Reply;
}

// ---- begin_pie / stop_pie / pie_status ---------------------------------

TSharedPtr<FJsonObject> HandleBeginPie(const TSharedPtr<FJsonObject>& Req)
{
	FString LevelPath;
	Req->TryGetStringField(TEXT("level_path"), LevelPath);

	double TimeoutS = 30.0;
	Req->TryGetNumberField(TEXT("timeout_s"), TimeoutS);

	FString SessionId;
	Req->TryGetStringField(TEXT("session_id"), SessionId);

	// Skip RequestPlaySession when PIE is already ready — restarting it
	// would briefly tear down the world and risks the recompile wedge.
	// Callers that want a forced restart can pass an explicit level_path.
	if (LevelPath.IsEmpty())
	{
		AAMjManager* ExistingMgr = AAMjManager::Instance;
		const bool bAlreadyReady =
			GEditor && GEditor->IsPlayingSessionInEditor()
			&& ExistingMgr
			&& ExistingMgr->PhysicsEngine
			&& ExistingMgr->PhysicsEngine->IsInitialized()
			&& ExistingMgr->GetLastCompileError().IsEmpty();
		if (bAlreadyReady)
		{
			TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
			Reply->SetStringField(TEXT("op"), TEXT("begin_pie_ok"));
			Reply->SetStringField(TEXT("state"), TEXT("ready"));
			Reply->SetStringField(TEXT("compile_error"), TEXT(""));
			TSharedPtr<FJsonObject> Hs = FURLabRpcDispatcher::BuildHandshakePayload(
				ExistingMgr, SessionId, TEXT("urlab/0.1"));
			if (Hs.IsValid())
			{
				Reply->SetObjectField(TEXT("handshake_payload"), Hs);
			}
			return Reply;
		}
	}

	// Dispatcher runs on a worker thread; PIE start must fire on the
	// game thread. Hop over and wait for the request to be queued.
	FEvent* QueuedEvent = FPlatformProcess::GetSynchEventFromPool(false);
	AsyncTask(ENamedThreads::GameThread, [LevelPath, QueuedEvent]() {
		if (!GEditor)
		{
			QueuedEvent->Trigger();
			return;
		}
		if (!LevelPath.IsEmpty())
		{
			if (ULevelEditorSubsystem* LSub =
					GEditor->GetEditorSubsystem<ULevelEditorSubsystem>())
			{
				LSub->LoadLevel(LevelPath);
			}
		}
		FRequestPlaySessionParams Params;
		Params.WorldType = EPlaySessionWorldType::PlayInEditor;

		// RequestPlaySession ignores "Selected Viewport" without an explicit
		// DestinationSlateViewport — hand it the active one.
		const ULevelEditorPlaySettings* PlaySettings =
			GetDefault<ULevelEditorPlaySettings>();
		if (PlaySettings && PlaySettings->LastExecutedPlayModeType == PlayMode_InViewPort)
		{
			FLevelEditorModule& LE =
				FModuleManager::LoadModuleChecked<FLevelEditorModule>(
					TEXT("LevelEditor"));
			if (TSharedPtr<ILevelEditor> LevelEditor =
					LE.GetFirstLevelEditor())
			{
				if (TSharedPtr<SLevelViewport> Vp =
						LevelEditor->GetActiveViewportInterface())
				{
					Params.DestinationSlateViewport = Vp;
				}
			}
		}
		GEditor->RequestPlaySession(Params);
		QueuedEvent->Trigger();
	});
	// Bounded — a modal dialog (PIE recompile error / asset-in-use) blocks
	// the game thread; unbounded Wait() would hang the worker.
	bool bQueued = false;
	{
		const double QueueDeadline = FPlatformTime::Seconds() + 10.0;
		while (FPlatformTime::Seconds() < QueueDeadline)
		{
			if (QueuedEvent->Wait(FTimespan::FromMilliseconds(50)))
			{
				bQueued = true;
				break;
			}
			UURLabBridgeServer* Bridge = URLabBridgeProvider::ResolveEditorServer();
			if (Bridge)
			{
				if (FURLabRpcDispatcher* D = Bridge->GetDispatcher())
				{
					if (D->IsDraining())
						break;
				}
			}
		}
	}
	FPlatformProcess::ReturnSynchEventToPool(QueuedEvent);
	if (!bQueued)
	{
		TSharedPtr<FJsonObject> TimeoutReply = MakeShared<FJsonObject>();
		TimeoutReply->SetStringField(TEXT("op"), TEXT("begin_pie_ok"));
		TimeoutReply->SetStringField(TEXT("state"), TEXT("timeout"));
		TimeoutReply->SetStringField(TEXT("compile_error"),
			TEXT("game thread blocked (modal dialog?); PIE request never queued"));
		return TimeoutReply;
	}

	// Poll for AAMjManager::Instance + compile completion. The manager's
	// BeginPlay registers with the bridge server (via the resolver), so
	// by the time PhysicsEngine->IsInitialized() returns true, the
	// dispatcher's OwnerMgr has been re-bound to the new manager.
	// Also exit early if the bridge is draining (e.g. user toggled
	// the editor toolbar's Stop while begin_pie was mid-flight) —
	// checking the bridge's dispatcher each tick.
	const double Deadline = FPlatformTime::Seconds() + TimeoutS;
	AAMjManager* Mgr = nullptr;
	bool bCompiledOrFailed = false;
	bool bDraining = false;
	while (FPlatformTime::Seconds() < Deadline)
	{
		UURLabBridgeServer* Bridge = URLabBridgeProvider::ResolveEditorServer();
		if (Bridge)
		{
			if (FURLabRpcDispatcher* D = Bridge->GetDispatcher())
			{
				if (D->IsDraining())
				{
					bDraining = true;
					break;
				}
			}
		}
		Mgr = AAMjManager::Instance;
		if (Mgr && Mgr->PhysicsEngine)
		{
			if (Mgr->PhysicsEngine->IsInitialized())
			{
				bCompiledOrFailed = true;
				break;
			}
			if (!Mgr->GetLastCompileError().IsEmpty())
			{
				bCompiledOrFailed = true;
				break;
			}
		}
		FPlatformProcess::Sleep(0.05f);
	}
	if (bDraining)
	{
		return MakeJsonError(TEXT("shutting_down"),
			TEXT("Bridge stopping; begin_pie abandoned"));
	}

	// pie_state enum on the wire. Five values:
	// off / compiling / compile_failed / timeout / ready.
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("begin_pie_ok"));

	if (!bCompiledOrFailed)
	{
		Reply->SetStringField(TEXT("state"), TEXT("timeout"));
		Reply->SetStringField(TEXT("compile_error"), TEXT(""));
		return Reply;
	}

	const FString CompileErr = Mgr->GetLastCompileError();
	if (!CompileErr.IsEmpty())
	{
		Reply->SetStringField(TEXT("state"), TEXT("compile_failed"));
		Reply->SetStringField(TEXT("compile_error"), CompileErr);
		return Reply;
	}

	// Compile ok: embed a fresh handshake-shaped payload so the bridge
	// can re-discover model + articulations against the new PIE world
	// without an extra hello round-trip.
	Reply->SetStringField(TEXT("state"), TEXT("ready"));
	Reply->SetStringField(TEXT("compile_error"), TEXT(""));

	TSharedPtr<FJsonObject> Hs = FURLabRpcDispatcher::BuildHandshakePayload(
		Mgr, SessionId, TEXT("urlab/0.1"));
	if (Hs.IsValid())
	{
		Reply->SetObjectField(TEXT("handshake_payload"), Hs);
	}
	return Reply;
}

TSharedPtr<FJsonObject> HandleStopPie(const TSharedPtr<FJsonObject>& /*Req*/)
{
	FEvent* DoneEvent = FPlatformProcess::GetSynchEventFromPool(false);
	AsyncTask(ENamedThreads::GameThread, [DoneEvent]() {
		if (GEditor && GEditor->IsPlayingSessionInEditor())
		{
			GEditor->RequestEndPlayMap();
		}
		DoneEvent->Trigger();
	});
	// Bounded wait — same wedge concern as HandleBeginPie. If the
	// game thread is blocked we return rather than hang the worker.
	bool bTriggered = false;
	{
		const double Deadline = FPlatformTime::Seconds() + 10.0;
		while (FPlatformTime::Seconds() < Deadline)
		{
			if (DoneEvent->Wait(FTimespan::FromMilliseconds(50)))
			{
				bTriggered = true;
				break;
			}
			UURLabBridgeServer* Bridge = URLabBridgeProvider::ResolveEditorServer();
			if (Bridge)
			{
				if (FURLabRpcDispatcher* D = Bridge->GetDispatcher())
				{
					if (D->IsDraining())
						break;
				}
			}
		}
	}
	FPlatformProcess::ReturnSynchEventToPool(DoneEvent);
	if (!bTriggered)
	{
		return MakeJsonError(TEXT("timeout"),
			TEXT("game thread blocked (modal dialog?); stop_pie did not complete"));
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("stop_pie_ok"));
	return Reply;
}

TSharedPtr<FJsonObject> HandlePieStatus(const TSharedPtr<FJsonObject>& /*Req*/)
{
	// Single `state` enum: off / compiling / compile_failed /
	// timeout / ready.
	// (timeout is unobservable from pie_status — only emitted by
	//  begin_pie when the deadline elapses; pie_status callers see
	//  the underlying state once compile finishes one way or other.)
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("pie_status_ok"));

	const bool bInPie = GEditor && GEditor->IsPlayingSessionInEditor();
	AAMjManager* Mgr = AAMjManager::Instance;
	FString State;
	FString CompileErr;
	if (!bInPie)
	{
		State = TEXT("off");
	}
	else if (!Mgr || !Mgr->PhysicsEngine)
	{
		State = TEXT("compiling");
	}
	else
	{
		CompileErr = Mgr->GetLastCompileError();
		if (!CompileErr.IsEmpty())
		{
			State = TEXT("compile_failed");
		}
		else if (Mgr->PhysicsEngine->IsInitialized())
		{
			State = TEXT("ready");
		}
		else
		{
			State = TEXT("compiling");
		}
	}

	Reply->SetStringField(TEXT("state"), State);
	Reply->SetStringField(TEXT("compile_error"), CompileErr);
	if (Mgr && Mgr->PhysicsEngine && Mgr->PhysicsEngine->m_data)
	{
		Reply->SetNumberField(TEXT("sim_time"),
			Mgr->PhysicsEngine->m_data->time);
	}
	return Reply;
}

TSharedPtr<FJsonObject> HandleSetActorTransform(const TSharedPtr<FJsonObject>& Req)
{
	FString Key, Err;
	bool bByName = false;
	if (!ResolveActorKey(Req, Key, bByName, Err))
	{
		return MakeJsonError(TEXT("missing_field"),
			FString::Printf(TEXT("set_actor_transform: %s"), *Err));
	}

	FVector Loc;
	const bool bHasLoc = ReadVec3(Req, TEXT("location"), Loc, FVector::ZeroVector);

	FQuat Q;
	const bool bHasRot = ReadRotation(Req, Q);

	FString ActorName;
	const bool bOk = URLabLevelOps::SetActorTransformSync(
		Key,
		bHasLoc ? &Loc : nullptr,
		bHasRot ? &Q : nullptr,
		ActorName, Err);
	if (!bOk)
		return MakeJsonError(TEXT("set_actor_transform_failed"), Err);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_actor_transform_ok"));
	Reply->SetStringField(TEXT("actor_name"), ActorName);
	Reply->SetBoolField(TEXT("requires_pie_restart"),
		GEditor ? GEditor->IsPlayingSessionInEditor() : false);
	return Reply;
}

// ---- list_actors / select_actor / add_quick_convert / remove_quick_convert

TSharedPtr<FJsonObject> HandleFindActors(const TSharedPtr<FJsonObject>& Req)
{
	FString ClassFilter, TagFilter, NamePrefix;
	Req->TryGetStringField(TEXT("class_filter"), ClassFilter);
	Req->TryGetStringField(TEXT("tag"), TagFilter);
	Req->TryGetStringField(TEXT("name_prefix"), NamePrefix);
	bool bInPie = false;
	Req->TryGetBoolField(TEXT("in_pie"), bInPie);

	TArray<TSharedPtr<FJsonValue>> Actors;
	bool bSearchedPie = false;
	FString Err;
	if (!URLabLevelOps::FindActorsSync(
			ClassFilter, TagFilter, NamePrefix, bInPie,
			Actors, bSearchedPie, Err))
	{
		return MakeJsonError(TEXT("find_actors_failed"), Err);
	}
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("find_actors_ok"));
	Reply->SetArrayField(TEXT("actors"), Actors);
	Reply->SetBoolField(TEXT("in_pie"), bSearchedPie);
	return Reply;
}

TSharedPtr<FJsonObject> HandleGetActorBounds(const TSharedPtr<FJsonObject>& Req)
{
	FString Key, Err;
	bool bByName = false;
	if (!ResolveActorKey(Req, Key, bByName, Err))
	{
		return MakeJsonError(TEXT("missing_field"),
			FString::Printf(TEXT("get_actor_bounds: %s"), *Err));
	}
	bool bComponentsOnly = false;
	Req->TryGetBoolField(TEXT("components_only"), bComponentsOnly);
	double Min[3], Max[3];
	FString ResolvedName;
	if (!URLabLevelOps::GetActorBoundsSync(
			Key, bComponentsOnly, Min, Max, ResolvedName, Err))
	{
		return MakeJsonError(TEXT("get_actor_bounds_failed"), Err);
	}
	auto Vec = [](const double V[3]) {
		TArray<TSharedPtr<FJsonValue>> A;
		for (int i = 0; i < 3; ++i)
			A.Add(MakeShared<FJsonValueNumber>(V[i]));
		return A;
	};
	const double Center[3] = {
		(Min[0] + Max[0]) * 0.5, (Min[1] + Max[1]) * 0.5, (Min[2] + Max[2]) * 0.5};
	const double Extents[3] = {
		(Max[0] - Min[0]) * 0.5, (Max[1] - Min[1]) * 0.5, (Max[2] - Min[2]) * 0.5};
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("get_actor_bounds_ok"));
	Reply->SetStringField(TEXT("actor_name"), ResolvedName);
	Reply->SetArrayField(TEXT("min"), Vec(Min));
	Reply->SetArrayField(TEXT("max"), Vec(Max));
	Reply->SetArrayField(TEXT("center"), Vec(Center));
	Reply->SetArrayField(TEXT("extents"), Vec(Extents));
	return Reply;
}

TSharedPtr<FJsonObject> HandleSnapshotScene(const TSharedPtr<FJsonObject>& /*Req*/)
{
	TArray<TSharedPtr<FJsonValue>> Actors;
	bool bInPie = false;
	FString LevelPath, Err;
	if (!URLabLevelOps::SnapshotSceneSync(Actors, bInPie, LevelPath, Err))
	{
		return MakeJsonError(TEXT("snapshot_failed"), Err);
	}
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("snapshot_ok"));
	Reply->SetArrayField(TEXT("actors"), Actors);
	Reply->SetBoolField(TEXT("in_pie"), bInPie);
	Reply->SetStringField(TEXT("level_path"), LevelPath);
	return Reply;
}

TSharedPtr<FJsonObject> HandleDuplicateActor(const TSharedPtr<FJsonObject>& Req)
{
	FString Key, Err;
	bool bByName = false;
	if (!ResolveActorKey(Req, Key, bByName, Err))
	{
		return MakeJsonError(TEXT("missing_field"),
			FString::Printf(TEXT("duplicate_actor: %s"), *Err));
	}
	FString NewActorId;
	if (!Req->TryGetStringField(TEXT("new_actor_id"), NewActorId) || NewActorId.IsEmpty())
	{
		return MakeJsonError(TEXT("missing_field"),
			TEXT("duplicate_actor requires non-empty 'new_actor_id'"));
	}

	FVector OverrideLoc;
	const bool bHasLoc = ReadVec3(Req, TEXT("location"), OverrideLoc, FVector::ZeroVector);

	FString OutName, OutPath, BPClassPath;
	if (!URLabLevelOps::DuplicateActorSync(
			Key, NewActorId, bHasLoc, OverrideLoc,
			OutName, OutPath, BPClassPath, Err))
	{
		return MakeJsonError(TEXT("duplicate_failed"), Err);
	}
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("duplicate_actor_ok"));
	Reply->SetStringField(TEXT("actor_id"), NewActorId);
	Reply->SetStringField(TEXT("actor_name"), OutName);
	Reply->SetStringField(TEXT("actor_path"), OutPath);
	Reply->SetStringField(TEXT("blueprint_class_path"), BPClassPath);
	return Reply;
}

TSharedPtr<FJsonObject> HandleActorHierarchy(const TSharedPtr<FJsonObject>& Req)
{
	FString Key, Err;
	bool bByName = false;
	if (!ResolveActorKey(Req, Key, bByName, Err))
	{
		return MakeJsonError(TEXT("missing_field"),
			FString::Printf(TEXT("actor_hierarchy: %s"), *Err));
	}
	TSharedPtr<FJsonObject> Root;
	if (!URLabLevelOps::ActorHierarchySync(Key, Root, Err))
	{
		return MakeJsonError(TEXT("actor_hierarchy_failed"), Err);
	}
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("actor_hierarchy_ok"));
	Reply->SetObjectField(TEXT("root"), Root);
	return Reply;
}

TSharedPtr<FJsonObject> HandleListActors(const TSharedPtr<FJsonObject>& /*Req*/)
{
	TArray<TSharedPtr<FJsonValue>> Actors;
	FString Err;
	if (!URLabLevelOps::ListActorsSync(Actors, Err))
	{
		return MakeJsonError(TEXT("list_actors_failed"), Err);
	}
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("list_actors_ok"));
	Reply->SetArrayField(TEXT("actors"), Actors);
	Reply->SetBoolField(TEXT("in_pie"),
		GEditor ? GEditor->IsPlayingSessionInEditor() : false);
	return Reply;
}

TSharedPtr<FJsonObject> HandleSelectActor(const TSharedPtr<FJsonObject>& Req)
{
	FString Key, Err;
	bool bByName = false;
	if (!ResolveActorKey(Req, Key, bByName, Err))
	{
		return MakeJsonError(TEXT("missing_field"),
			FString::Printf(TEXT("select_actor: %s"), *Err));
	}
	FString ActorName;
	if (!URLabLevelOps::SelectActorSync(Key, ActorName, Err))
	{
		return MakeJsonError(TEXT("select_failed"), Err);
	}
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("select_actor_ok"));
	Reply->SetStringField(TEXT("actor_name"), ActorName);
	return Reply;
}

TSharedPtr<FJsonObject> HandleAddQuickConvert(const TSharedPtr<FJsonObject>& Req)
{
	FString Key, Err;
	bool bByName = false;
	if (!ResolveActorKey(Req, Key, bByName, Err))
	{
		return MakeJsonError(TEXT("missing_field"),
			FString::Printf(TEXT("add_quick_convert: %s"), *Err));
	}
	bool bStatic = false, bComplexMesh = false, bDrivenByUnreal = false;
	Req->TryGetBoolField(TEXT("static"), bStatic);
	Req->TryGetBoolField(TEXT("complex_mesh"), bComplexMesh);
	Req->TryGetBoolField(TEXT("driven_by_unreal"), bDrivenByUnreal);
	double CoACDThreshold = 0.05;
	Req->TryGetNumberField(TEXT("coacd_threshold"), CoACDThreshold);
	FVector Friction(1.0, 1.0, 1.0);
	ReadVec3(Req, TEXT("friction"), Friction, FVector(1, 1, 1));

	FString ActorName;
	if (!URLabLevelOps::AddQuickConvertSync(
			Key, bStatic, bComplexMesh,
			static_cast<float>(CoACDThreshold),
			bDrivenByUnreal, Friction,
			ActorName, Err))
	{
		return MakeJsonError(TEXT("add_quick_convert_failed"), Err);
	}
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("add_quick_convert_ok"));
	Reply->SetStringField(TEXT("actor_name"), ActorName);
	Reply->SetBoolField(TEXT("static"), bStatic);
	Reply->SetBoolField(TEXT("complex_mesh"), bComplexMesh);
	Reply->SetNumberField(TEXT("coacd_threshold"), CoACDThreshold);
	Reply->SetBoolField(TEXT("driven_by_unreal"), bDrivenByUnreal);
	Reply->SetBoolField(TEXT("requires_pie_restart"),
		GEditor ? GEditor->IsPlayingSessionInEditor() : false);
	return Reply;
}

TSharedPtr<FJsonObject> HandleListBlueprints(const TSharedPtr<FJsonObject>& /*Req*/)
{
	TArray<TSharedPtr<FJsonValue>> Items;
	FString Err;
	if (!URLabLevelOps::ListBlueprintsSync(Items, Err))
	{
		return MakeJsonError(TEXT("list_blueprints_failed"), Err);
	}
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("list_blueprints_ok"));
	Reply->SetArrayField(TEXT("blueprints"), Items);
	return Reply;
}

TSharedPtr<FJsonObject> HandleRemoveQuickConvert(const TSharedPtr<FJsonObject>& Req)
{
	FString Key, Err;
	bool bByName = false;
	if (!ResolveActorKey(Req, Key, bByName, Err))
	{
		return MakeJsonError(TEXT("missing_field"),
			FString::Printf(TEXT("remove_quick_convert: %s"), *Err));
	}
	FString ActorName;
	if (!URLabLevelOps::RemoveQuickConvertSync(Key, ActorName, Err))
	{
		return MakeJsonError(TEXT("remove_quick_convert_failed"), Err);
	}
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("remove_quick_convert_ok"));
	Reply->SetStringField(TEXT("actor_name"), ActorName);
	Reply->SetBoolField(TEXT("requires_pie_restart"),
		GEditor ? GEditor->IsPlayingSessionInEditor() : false);
	return Reply;
}

TSharedPtr<FJsonObject> HandleDestroyActor(const TSharedPtr<FJsonObject>& Req)
{
	FString Id, Err;
	bool bByName = false;
	if (!ResolveActorKey(Req, Id, bByName, Err))
	{
		return MakeJsonError(TEXT("missing_field"),
			FString::Printf(TEXT("destroy_actor: %s"), *Err));
	}
	if (!URLabLevelOps::DestroyActorSync(Id, Err))
	{
		return MakeJsonError(TEXT("destroy_failed"), Err);
	}
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("destroy_actor_ok"));
	Reply->SetBoolField(TEXT("requires_pie_restart"),
		GEditor ? GEditor->IsPlayingSessionInEditor() : false);
	return Reply;
}

// debug.* — DrawDebug wrappers. Positions in MJ metres, colour [0,1],
// ttl seconds (0 = single frame, -1 = persistent until clear_markers).

UWorld* PickDebugWorld()
{
	if (!GEditor)
		return nullptr;
	if (UWorld* Pie = GEditor->PlayWorld)
		return Pie;
	return GEditor->GetEditorWorldContext().World();
}

FColor ReadColor(const TSharedPtr<FJsonObject>& Req, const FColor Default = FColor::Yellow)
{
	FVector Rgb;
	if (!ReadVec3(Req, TEXT("color"), Rgb, FVector(Default.R / 255.0, Default.G / 255.0, Default.B / 255.0)))
		return Default;
	return FColor(
		(uint8)FMath::Clamp((int32)(Rgb.X * 255.0), 0, 255),
		(uint8)FMath::Clamp((int32)(Rgb.Y * 255.0), 0, 255),
		(uint8)FMath::Clamp((int32)(Rgb.Z * 255.0), 0, 255),
		255);
}

/** Translate wire ttl into (bPersistent, LifeTime) pair UE's DrawDebug*
 *  helpers expect. ttl > 0: bPersistent=true, lifetime=ttl (auto-expires).
 *  ttl == -1: bPersistent=true, lifetime=-1 (kept until clear_markers).
 *  ttl == 0 or missing: bPersistent=false, lifetime=-1 (single frame). */
void ReadTtl(const TSharedPtr<FJsonObject>& Req, bool& bOutPersistent, float& OutLifetime)
{
	double Ttl = 0.0;
	Req->TryGetNumberField(TEXT("ttl"), Ttl);
	if (Ttl > 0.0)
	{
		bOutPersistent = true;
		OutLifetime = (float)Ttl;
	}
	else if (Ttl < 0.0)
	{
		bOutPersistent = true;
		OutLifetime = -1.0f;
	}
	else
	{
		bOutPersistent = false;
		OutLifetime = -1.0f;
	}
}

TSharedPtr<FJsonObject> HandleDrawMarker(const TSharedPtr<FJsonObject>& Req)
{
	UWorld* World = PickDebugWorld();
	if (!World)
		return MakeJsonError(TEXT("no_world"), TEXT("no editor / PIE world"));

	FVector MjLoc;
	if (!ReadVec3(Req, TEXT("location"), MjLoc, FVector::ZeroVector))
		return MakeJsonError(TEXT("missing_field"), TEXT("draw_marker requires 'location'"));
	const double MjPos[3] = {MjLoc.X, MjLoc.Y, MjLoc.Z};
	const FVector UELoc = MjUtils::MjToUEPosition(MjPos);

	const FColor Color = ReadColor(Req);
	bool bPersistent;
	float Lifetime;
	ReadTtl(Req, bPersistent, Lifetime);

	// 5cm sphere — small enough not to obscure geometry, big enough to spot.
	DrawDebugSphere(World, UELoc, 5.0f, 16, Color, bPersistent, Lifetime);

	FString Label;
	if (Req->TryGetStringField(TEXT("label"), Label) && !Label.IsEmpty())
		DrawDebugString(World, UELoc, Label, nullptr, Color, Lifetime, /*bDrawShadow=*/true);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("draw_marker_ok"));
	return Reply;
}

TSharedPtr<FJsonObject> HandleDrawLine(const TSharedPtr<FJsonObject>& Req)
{
	UWorld* World = PickDebugWorld();
	if (!World)
		return MakeJsonError(TEXT("no_world"), TEXT("no editor / PIE world"));

	FVector MjFrom, MjTo;
	if (!ReadVec3(Req, TEXT("from"), MjFrom, FVector::ZeroVector)
		|| !ReadVec3(Req, TEXT("to"), MjTo, FVector::ZeroVector))
		return MakeJsonError(TEXT("missing_field"), TEXT("draw_line requires 'from' + 'to'"));
	const double MjF[3] = {MjFrom.X, MjFrom.Y, MjFrom.Z};
	const double MjT[3] = {MjTo.X, MjTo.Y, MjTo.Z};
	const FVector UEFrom = MjUtils::MjToUEPosition(MjF);
	const FVector UETo = MjUtils::MjToUEPosition(MjT);

	const FColor Color = ReadColor(Req);
	bool bPersistent;
	float Lifetime;
	ReadTtl(Req, bPersistent, Lifetime);

	double Thickness = 1.0;
	Req->TryGetNumberField(TEXT("thickness"), Thickness);

	DrawDebugLine(World, UEFrom, UETo, Color, bPersistent, Lifetime, 0, (float)Thickness);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("draw_line_ok"));
	return Reply;
}

TSharedPtr<FJsonObject> HandleDrawBox(const TSharedPtr<FJsonObject>& Req)
{
	UWorld* World = PickDebugWorld();
	if (!World)
		return MakeJsonError(TEXT("no_world"), TEXT("no editor / PIE world"));

	FVector MjCenter, MjHalf;
	if (!ReadVec3(Req, TEXT("center"), MjCenter, FVector::ZeroVector)
		|| !ReadVec3(Req, TEXT("half_extents"), MjHalf, FVector(0.1, 0.1, 0.1)))
		return MakeJsonError(TEXT("missing_field"),
			TEXT("draw_box requires 'center' + 'half_extents'"));

	const double MjC[3] = {MjCenter.X, MjCenter.Y, MjCenter.Z};
	const FVector UECenter = MjUtils::MjToUEPosition(MjC);
	// Extents are unsigned magnitudes; cm scaling is one MjToUEDist multiply.
	// MjUtils::MjToUEPosition would also flip Y, so we do extents by hand.
	const FVector UEHalf(
		FMath::Abs(MjHalf.X) * 100.0,
		FMath::Abs(MjHalf.Y) * 100.0,
		FMath::Abs(MjHalf.Z) * 100.0);

	FQuat UEQuat = FQuat::Identity;
	if (Req->HasField(TEXT("rotation_quat")) || Req->HasField(TEXT("rotation_euler")))
		ReadRotation(Req, UEQuat);

	const FColor Color = ReadColor(Req);
	bool bPersistent;
	float Lifetime;
	ReadTtl(Req, bPersistent, Lifetime);

	DrawDebugBox(World, UECenter, UEHalf, UEQuat, Color, bPersistent, Lifetime);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("draw_box_ok"));
	return Reply;
}

TSharedPtr<FJsonObject> HandleDrawArrow(const TSharedPtr<FJsonObject>& Req)
{
	UWorld* World = PickDebugWorld();
	if (!World)
		return MakeJsonError(TEXT("no_world"), TEXT("no editor / PIE world"));

	FVector MjFrom, MjTo;
	if (!ReadVec3(Req, TEXT("from"), MjFrom, FVector::ZeroVector)
		|| !ReadVec3(Req, TEXT("to"), MjTo, FVector::ZeroVector))
		return MakeJsonError(TEXT("missing_field"),
			TEXT("draw_arrow requires 'from' + 'to'"));
	const double MjF[3] = {MjFrom.X, MjFrom.Y, MjFrom.Z};
	const double MjT[3] = {MjTo.X, MjTo.Y, MjTo.Z};
	const FVector UEFrom = MjUtils::MjToUEPosition(MjF);
	const FVector UETo = MjUtils::MjToUEPosition(MjT);

	const FColor Color = ReadColor(Req);
	bool bPersistent;
	float Lifetime;
	ReadTtl(Req, bPersistent, Lifetime);

	double Thickness = 1.0;
	Req->TryGetNumberField(TEXT("thickness"), Thickness);
	// Arrow head defaults to 20% of the shaft length so the head
	// scales sensibly across short and long arrows. Caller-supplied
	// value is in MJ metres, converted to UE cm here.
	double ArrowSizeUE = (UETo - UEFrom).Length() * 0.2;
	double ArrowSizeMj = 0.0;
	if (Req->TryGetNumberField(TEXT("arrow_size"), ArrowSizeMj) && ArrowSizeMj > 0.0)
		ArrowSizeUE = ArrowSizeMj * 100.0;

	DrawDebugDirectionalArrow(World, UEFrom, UETo, (float)ArrowSizeUE,
		Color, bPersistent, Lifetime, 0, (float)Thickness);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("draw_arrow_ok"));
	return Reply;
}

TSharedPtr<FJsonObject> HandleDrawAxes(const TSharedPtr<FJsonObject>& Req)
{
	UWorld* World = PickDebugWorld();
	if (!World)
		return MakeJsonError(TEXT("no_world"), TEXT("no editor / PIE world"));

	FVector MjLoc;
	if (!ReadVec3(Req, TEXT("location"), MjLoc, FVector::ZeroVector))
		return MakeJsonError(TEXT("missing_field"), TEXT("draw_axes requires 'location'"));
	const double MjPos[3] = {MjLoc.X, MjLoc.Y, MjLoc.Z};
	const FVector UEOrigin = MjUtils::MjToUEPosition(MjPos);

	FQuat UEQuat = FQuat::Identity;
	ReadRotation(Req, UEQuat);

	double ScaleM = 0.2;
	Req->TryGetNumberField(TEXT("scale"), ScaleM);
	const float UEScale = (float)(FMath::Abs(ScaleM) * 100.0); // m -> cm

	bool bPersistent;
	float Lifetime;
	ReadTtl(Req, bPersistent, Lifetime);

	const FVector AxX = UEQuat.RotateVector(FVector(UEScale, 0.0, 0.0));
	const FVector AxY = UEQuat.RotateVector(FVector(0.0, UEScale, 0.0));
	const FVector AxZ = UEQuat.RotateVector(FVector(0.0, 0.0, UEScale));
	const float ArrowHead = UEScale * 0.2f;
	DrawDebugDirectionalArrow(World, UEOrigin, UEOrigin + AxX, ArrowHead, FColor::Red, bPersistent, Lifetime);
	DrawDebugDirectionalArrow(World, UEOrigin, UEOrigin + AxY, ArrowHead, FColor::Green, bPersistent, Lifetime);
	DrawDebugDirectionalArrow(World, UEOrigin, UEOrigin + AxZ, ArrowHead, FColor::Blue, bPersistent, Lifetime);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("draw_axes_ok"));
	return Reply;
}

TSharedPtr<FJsonObject> HandleClearMarkers(const TSharedPtr<FJsonObject>& /*Req*/)
{
	UWorld* World = PickDebugWorld();
	if (!World)
		return MakeJsonError(TEXT("no_world"), TEXT("no editor / PIE world"));

	// UE's debug drawing has no tag-filtered clear — always full wipe.
	// The `tag` field is accepted but ignored.
	FlushPersistentDebugLines(World);
	FlushDebugStrings(World);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("clear_markers_ok"));
	return Reply;
}

TSharedPtr<FJsonObject> HandleSetOverlayText(const TSharedPtr<FJsonObject>& Req)
{
	if (!GEngine)
		return MakeJsonError(TEXT("no_engine"), TEXT("GEngine null"));

	FString Text;
	Req->TryGetStringField(TEXT("text"), Text);

	// Empty text clears any URLab message — UE doesn't expose a
	// per-key clear, so we re-issue a 0-duration empty message which
	// expires next frame.
	const float Duration = Text.IsEmpty() ? 0.0f : 10.0f;
	// Stable key so successive set_overlay_text calls update instead
	// of stacking. Anchor field is parsed but not honoured (v1 uses
	// UE's fixed top-of-viewport positioning).
	GEngine->AddOnScreenDebugMessage(
		/*Key=*/(int32)0x2DEB6072, Duration, FColor::White, Text);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_overlay_text_ok"));
	return Reply;
}

// viewport.* — perspective viewport camera control. Positions in MJ metres.
// (See PickPerspectiveViewportClient for the active-viewport fallback.)

AActor* FindActorInEditorWorld(const FString& Target, bool bByName)
{
	if (!GEditor)
		return nullptr;
	UWorld* World = GEditor->GetEditorWorldContext().World();
	if (!World)
		return nullptr;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* A = *It;
		if (!A)
			continue;
		if (bByName)
		{
			if (A->GetName().Equals(Target))
				return A;
		}
		else
		{
			// AMjArticulation stores its actor_id as a UPROPERTY,
			// not a tag — the tag form (URLab.ActorId=<id>) is only
			// used for non-articulation actors like lights. Check
			// the UPROPERTY first, then fall back to the tag.
			if (AMjArticulation* Mj = Cast<AMjArticulation>(A))
			{
				if (Mj->ActorId.Equals(Target))
					return A;
			}
			for (const FName& Tag : A->Tags)
			{
				const FString TagStr = Tag.ToString();
				if (TagStr.StartsWith(TEXT("URLab.ActorId="))
					&& TagStr.RightChop(14).Equals(Target))
				{
					return A;
				}
			}
		}
	}
	return nullptr;
}

FEditorViewportClient* PickPerspectiveViewportClient()
{
	if (GCurrentLevelEditingViewportClient && GCurrentLevelEditingViewportClient->IsPerspective())
		return GCurrentLevelEditingViewportClient;
	if (!GEditor)
		return nullptr;
	for (FEditorViewportClient* C : GEditor->GetLevelViewportClients())
	{
		if (C && C->IsPerspective())
			return C;
	}
	return nullptr;
}

void ExportCameraPose(FEditorViewportClient* Client, TSharedPtr<FJsonObject>& Reply)
{
	const FVector UELoc = Client->GetViewLocation();
	const FRotator UERot = Client->GetViewRotation();
	double MjPos[3] = {0};
	MjUtils::UEToMjPosition(UELoc, MjPos);
	double MjQuat[4] = {0};
	MjUtils::UEToMjRotation(UERot.Quaternion(), MjQuat);

	TArray<TSharedPtr<FJsonValue>> Loc;
	for (int32 i = 0; i < 3; ++i)
		Loc.Add(MakeShared<FJsonValueNumber>(MjPos[i]));
	Reply->SetArrayField(TEXT("location"), Loc);

	// rotation_quat wire convention is xyzw (UE-native FQuat order), to
	// match outliner/scene rotation handling. MjQuat is wxyz, so
	// re-pack as xyzw for the wire.
	TArray<TSharedPtr<FJsonValue>> Q;
	Q.Add(MakeShared<FJsonValueNumber>(MjQuat[1]));
	Q.Add(MakeShared<FJsonValueNumber>(MjQuat[2]));
	Q.Add(MakeShared<FJsonValueNumber>(MjQuat[3]));
	Q.Add(MakeShared<FJsonValueNumber>(MjQuat[0]));
	Reply->SetArrayField(TEXT("rotation_quat"), Q);

	TArray<TSharedPtr<FJsonValue>> E;
	E.Add(MakeShared<FJsonValueNumber>(UERot.Roll));
	E.Add(MakeShared<FJsonValueNumber>(UERot.Pitch));
	E.Add(MakeShared<FJsonValueNumber>(UERot.Yaw));
	Reply->SetArrayField(TEXT("rotation_euler"), E);

	Reply->SetNumberField(TEXT("fov"), Client->ViewFOV);
}

TSharedPtr<FJsonObject> HandleViewportSetCamera(const TSharedPtr<FJsonObject>& Req)
{
	FEditorViewportClient* Client = PickPerspectiveViewportClient();
	if (!Client)
		return MakeJsonError(TEXT("no_viewport"), TEXT("no perspective viewport"));

	FVector MjLoc;
	if (!ReadVec3(Req, TEXT("location"), MjLoc, FVector::ZeroVector))
		return MakeJsonError(TEXT("missing_field"), TEXT("set_camera requires 'location'"));
	const double MjPos[3] = {MjLoc.X, MjLoc.Y, MjLoc.Z};
	Client->SetViewLocation(MjUtils::MjToUEPosition(MjPos));

	FQuat UEQuat;
	if (ReadRotation(Req, UEQuat))
		Client->SetViewRotation(UEQuat.Rotator());

	double Fov = 0.0;
	if (Req->TryGetNumberField(TEXT("fov"), Fov) && Fov > 0.0)
		Client->ViewFOV = (float)Fov;

	Client->Invalidate();

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_camera_ok"));
	ExportCameraPose(Client, Reply);
	return Reply;
}

TSharedPtr<FJsonObject> HandleViewportGetCamera(const TSharedPtr<FJsonObject>& /*Req*/)
{
	FEditorViewportClient* Client = PickPerspectiveViewportClient();
	if (!Client)
		return MakeJsonError(TEXT("no_viewport"), TEXT("no perspective viewport"));
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("get_camera_ok"));
	ExportCameraPose(Client, Reply);
	return Reply;
}

TSharedPtr<FJsonObject> HandleViewportFrameActor(const TSharedPtr<FJsonObject>& Req)
{
	FEditorViewportClient* Client = PickPerspectiveViewportClient();
	if (!Client)
		return MakeJsonError(TEXT("no_viewport"), TEXT("no perspective viewport"));

	FString Key, Err;
	bool bByName = false;
	if (!ResolveActorKey(Req, Key, bByName, Err))
	{
		return MakeJsonError(TEXT("missing_field"),
			FString::Printf(TEXT("frame_actor: %s"), *Err));
	}
	AActor* Actor = FindActorInEditorWorld(Key, bByName);
	if (!Actor)
		return MakeJsonError(TEXT("unknown_actor"), Key);

	FBox Box = Actor->GetComponentsBoundingBox(/*bNonColliding=*/true);
	if (!Box.IsValid)
		return MakeJsonError(TEXT("no_bounds"),
			FString::Printf(TEXT("Actor '%s' has no valid bounds"), *Key));

	Client->FocusViewportOnBox(Box, /*bInstant=*/true);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("frame_actor_ok"));
	ExportCameraPose(Client, Reply);
	return Reply;
}

TSharedPtr<FJsonObject> HandleViewportSetMode(const TSharedPtr<FJsonObject>& Req)
{
	FEditorViewportClient* Client = PickPerspectiveViewportClient();
	if (!Client)
		return MakeJsonError(TEXT("no_viewport"), TEXT("no perspective viewport"));

	FString ModeStr;
	Req->TryGetStringField(TEXT("mode"), ModeStr);
	if (ModeStr.IsEmpty())
		return MakeJsonError(TEXT("missing_field"), TEXT("set_mode requires 'mode'"));

	EViewModeIndex Mode = VMI_Lit;
	if (ModeStr.Equals(TEXT("lit"), ESearchCase::IgnoreCase))
		Mode = VMI_Lit;
	else if (ModeStr.Equals(TEXT("unlit"), ESearchCase::IgnoreCase))
		Mode = VMI_Unlit;
	else if (ModeStr.Equals(TEXT("wireframe"), ESearchCase::IgnoreCase))
		Mode = VMI_Wireframe;
	else if (ModeStr.Equals(TEXT("collision"), ESearchCase::IgnoreCase))
		Mode = VMI_CollisionPawn;
	else if (ModeStr.Equals(TEXT("reflections"), ESearchCase::IgnoreCase))
		Mode = VMI_ReflectionOverride;
	else
		return MakeJsonError(TEXT("invalid_mode"),
			FString::Printf(TEXT("unknown viewport mode '%s'"), *ModeStr));

	Client->SetViewMode(Mode);
	Client->Invalidate();

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_mode_ok"));
	Reply->SetStringField(TEXT("mode"), ModeStr);
	return Reply;
}

// Track-camera ticker state. Lives in module-static storage because
// it must outlive any single op invocation. One ticker at a time —
// a new track_actor call replaces the previous target. Note: state
// is NOT thread-safe and assumes all mutations happen on the game
// thread (the editor-op marshaller already guarantees that).
struct FTrackState
{
	TWeakObjectPtr<AActor> Target;
	FVector OffsetUE = FVector(-200.0, 0.0, 100.0); // ~2m behind, 1m above
	float Smoothing = 0.0f;
	FTSTicker::FDelegateHandle Handle;
};
static FTrackState& GetTrackState()
{
	static FTrackState S;
	return S;
}

void StopTrackingCameraInternal()
{
	FTrackState& S = GetTrackState();
	if (S.Handle.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(S.Handle);
		S.Handle.Reset();
	}
	S.Target.Reset();
}

TSharedPtr<FJsonObject> HandleViewportTrackActor(const TSharedPtr<FJsonObject>& Req)
{
	FEditorViewportClient* Client = PickPerspectiveViewportClient();
	if (!Client)
		return MakeJsonError(TEXT("no_viewport"), TEXT("no perspective viewport"));

	FString Key, Err;
	bool bByName = false;
	if (!ResolveActorKey(Req, Key, bByName, Err))
	{
		return MakeJsonError(TEXT("missing_field"),
			FString::Printf(TEXT("track_actor: %s"), *Err));
	}
	AActor* Actor = FindActorInEditorWorld(Key, bByName);
	if (!Actor)
		return MakeJsonError(TEXT("unknown_actor"), Key);

	FTrackState& S = GetTrackState();
	StopTrackingCameraInternal();
	S.Target = Actor;

	FVector OffsetMj;
	if (ReadVec3(Req, TEXT("offset"), OffsetMj, FVector(0.0, -2.0, 1.0)))
	{
		const double MjOff[3] = {OffsetMj.X, OffsetMj.Y, OffsetMj.Z};
		S.OffsetUE = MjUtils::MjToUEPosition(MjOff);
	}
	else
	{
		const double MjOff[3] = {0.0, -2.0, 1.0};
		S.OffsetUE = MjUtils::MjToUEPosition(MjOff);
	}

	double SmoothingD = 0.0;
	Req->TryGetNumberField(TEXT("smoothing"), SmoothingD);
	S.Smoothing = FMath::Clamp((float)SmoothingD, 0.0f, 0.99f);

	S.Handle = FTSTicker::GetCoreTicker().AddTicker(
		FTickerDelegate::CreateLambda([](float /*DeltaTime*/) -> bool {
			FTrackState& Ss = GetTrackState();
			AActor* T = Ss.Target.Get();
			FEditorViewportClient* C = PickPerspectiveViewportClient();
			if (!T || !C)
			{
				Ss.Handle.Reset();
				Ss.Target.Reset();
				return false; // remove ticker
			}
			const FVector ActorLoc = T->GetActorLocation();
			const FRotator ActorRot = T->GetActorRotation();
			const FVector DesiredLoc = ActorLoc + ActorRot.RotateVector(Ss.OffsetUE);
			const FRotator DesiredRot = (ActorLoc - DesiredLoc).Rotation();
			const FVector NewLoc = FMath::Lerp(C->GetViewLocation(), DesiredLoc, 1.0f - Ss.Smoothing);
			const FRotator NewRot = FMath::Lerp(C->GetViewRotation(), DesiredRot, 1.0f - Ss.Smoothing);
			C->SetViewLocation(NewLoc);
			C->SetViewRotation(NewRot);
			C->Invalidate();
			return true; // keep ticking
		}),
		/*InDelay=*/0.0f);

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("track_actor_ok"));
	Reply->SetStringField(TEXT("tracked_actor_path"), Actor->GetPathName());
	return Reply;
}

TSharedPtr<FJsonObject> HandleViewportUntrack(const TSharedPtr<FJsonObject>& /*Req*/)
{
	const bool bWasTracking = GetTrackState().Handle.IsValid();
	StopTrackingCameraInternal();
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("untrack_ok"));
	Reply->SetBoolField(TEXT("was_tracking"), bWasTracking);
	return Reply;
}
} // namespace

namespace URLabEditorOpHandlers
{
// Wrap a raw handler in the game-thread marshaller. The dispatcher
// runs on a ZMQ worker thread, but every editor API the body
// touches (asset registry, level subsystem, GEditor) requires the
// game thread. begin_pie / stop_pie already do their own
// game-thread hopping (they need to keep the polling loop on the
// worker), so they bypass this wrapper.
static URLabOpRegistry::FHandler GameThreadHandler(
	TSharedPtr<FJsonObject> (*Inner)(const TSharedPtr<FJsonObject>&))
{
	return [Inner](const TSharedPtr<FJsonObject>& Req) {
		return RunOnGameThreadSync([&]() { return Inner(Req); });
	};
}

/** Each editor op carries its category + namespace metadata so the
 *  bridge can synthesise method bindings from the `meta` payload. */
void RegEditor(const TCHAR* Name, const TCHAR* Ns,
	URLabOpRegistry::FHandler Body,
	std::initializer_list<const TCHAR*> ReplyFields = {},
	std::initializer_list<const TCHAR*> RequiredFields = {})
{
	URLabOpRegistry::FOpDecl Decl;
	Decl.Name = Name;
	Decl.Category = URLabOpRegistry::EOpCategory::EditorOnly;
	Decl.Namespace = Ns;
	Decl.Body = MoveTemp(Body);
	for (const TCHAR* F : ReplyFields)
		Decl.ReplyFields.Add(F);
	for (const TCHAR* F : RequiredFields)
		Decl.RequiredFields.Add(F);
	URLabOpRegistry::RegisterOp(MoveTemp(Decl));
}

void RegisterAll()
{
	// scene namespace: level / asset import + spawn ops.
	RegEditor(TEXT("import_xml"), TEXT("scene"),
		GameThreadHandler(&HandleImportXml),
		/*Reply=*/{TEXT("op:string"), TEXT("blueprint_class_path:string"), TEXT("blueprint_short_name:string"), TEXT("imported_now:bool")},
		/*Required=*/{TEXT("path")});
	RegEditor(TEXT("create_level"), TEXT("scene"),
		GameThreadHandler(&HandleCreateLevel),
		/*Reply=*/{TEXT("op:string"), TEXT("level_path:string")},
		/*Required=*/{TEXT("name")});
	RegEditor(TEXT("ensure_manager"), TEXT("scene"),
		GameThreadHandler(&HandleEnsureManager),
		{TEXT("op:string"), TEXT("actor_name:string"),
			TEXT("actor_path:string"), TEXT("was_existing:bool")});
	RegEditor(TEXT("destroy_asset"), TEXT("scene"),
		GameThreadHandler(&HandleDestroyAsset),
		/*Reply=*/{TEXT("op:string"), TEXT("asset_path:string"), TEXT("was_found:bool")},
		/*Required=*/{TEXT("asset_path")});
	RegEditor(TEXT("current_level"), TEXT("scene"),
		GameThreadHandler(&HandleCurrentLevel),
		{TEXT("op:string"), TEXT("level_path:string")});
	RegEditor(TEXT("snapshot"), TEXT("scene"),
		GameThreadHandler(&HandleSnapshotScene),
		{TEXT("op:string"), TEXT("actors:array"),
			TEXT("in_pie:bool"), TEXT("level_path:string")});
	RegEditor(TEXT("duplicate_actor"), TEXT("scene"),
		GameThreadHandler(&HandleDuplicateActor),
		/*Reply=*/{TEXT("op:string"), TEXT("actor_id:string"), TEXT("actor_name:string"), TEXT("actor_path:string"), TEXT("blueprint_class_path:string")},
		/*Required=*/{TEXT("target"), TEXT("new_actor_id")});
	RegEditor(TEXT("actor_hierarchy"), TEXT("scene"),
		GameThreadHandler(&HandleActorHierarchy),
		/*Reply=*/{TEXT("op:string"), TEXT("root:object")},
		/*Required=*/{TEXT("target")});
	RegEditor(TEXT("load_level"), TEXT("scene"),
		GameThreadHandler(&HandleLoadLevel),
		/*Reply=*/{TEXT("op:string"), TEXT("level_path:string")},
		/*Required=*/{TEXT("level_path")});
	RegEditor(TEXT("save_level"), TEXT("scene"),
		GameThreadHandler(&HandleSaveLevel),
		{TEXT("op:string"), TEXT("level_path:string")});
	RegEditor(TEXT("spawn_actor"), TEXT("scene"),
		GameThreadHandler(&HandleSpawnActor),
		/*Reply=*/{TEXT("op:string"), TEXT("actor_id:string"), TEXT("actor_name:string"), TEXT("actor_path:string"), TEXT("blueprint_class_path:string"), TEXT("location:array"), TEXT("rotation_quat:array"), TEXT("was_existing:bool"), TEXT("requires_pie_restart:bool")},
		/*Required=*/{TEXT("blueprint")});
	RegEditor(TEXT("spawn_grid"), TEXT("scene"),
		GameThreadHandler(&HandleSpawnGrid),
		/*Reply=*/{TEXT("op:string"), TEXT("count:int"), TEXT("blueprint_class_path:string"), TEXT("actors:array"), TEXT("requires_pie_restart:bool")},
		/*Required=*/{TEXT("blueprint"), TEXT("base_actor_id"), TEXT("count_x"), TEXT("count_y")});
	RegEditor(TEXT("spawn_light"), TEXT("scene"),
		GameThreadHandler(&HandleSpawnLight),
		{TEXT("op:string"), TEXT("actor_id:string"), TEXT("actor_name:string"),
			TEXT("actor_path:string"), TEXT("kind:string"),
			TEXT("intensity:float"), TEXT("color:array"),
			TEXT("location:array"), TEXT("requires_pie_restart:bool")});
	RegEditor(TEXT("destroy_actor"), TEXT("scene"),
		GameThreadHandler(&HandleDestroyActor),
		/*Reply=*/{TEXT("op:string"), TEXT("requires_pie_restart:bool")},
		/*Required=*/{TEXT("target")});
	RegEditor(TEXT("set_actor_transform"), TEXT("scene"),
		GameThreadHandler(&HandleSetActorTransform),
		/*Reply=*/{TEXT("op:string"), TEXT("target:string"), TEXT("actor_name:string"), TEXT("requires_pie_restart:bool")},
		/*Required=*/{TEXT("target")});

	// sim namespace: PIE lifecycle. Per the plan §5.1 the Python
	// wrappers expose these as client.sim.start / sim.stop /
	// sim.status; the wire op names (begin_pie / stop_pie /
	// pie_status) stay unchanged.
	RegEditor(TEXT("pie_status"), TEXT("sim"),
		GameThreadHandler(&HandlePieStatus),
		{TEXT("op:string"), TEXT("state:string"), TEXT("compile_error:string"),
			TEXT("sim_time:float?")});
	// begin_pie / stop_pie self-marshal — they need the worker
	// thread to host the post-RequestPlaySession polling loop so
	// the game thread isn't blocked while compile finishes.
	RegEditor(TEXT("begin_pie"), TEXT("sim"), &HandleBeginPie,
		{TEXT("op:string"), TEXT("state:string"), TEXT("compile_error:string"),
			TEXT("handshake_payload:object?")});
	RegEditor(TEXT("stop_pie"), TEXT("sim"), &HandleStopPie,
		{TEXT("op:string")});

	// outliner namespace: world-state introspection + edits.
	RegEditor(TEXT("find_actors"), TEXT("outliner"),
		GameThreadHandler(&HandleFindActors),
		{TEXT("op:string"), TEXT("actors:array"), TEXT("in_pie:bool")});
	RegEditor(TEXT("get_actor_bounds"), TEXT("outliner"),
		GameThreadHandler(&HandleGetActorBounds),
		/*Reply=*/{TEXT("op:string"), TEXT("actor_name:string"), TEXT("min:array"), TEXT("max:array"), TEXT("center:array"), TEXT("extents:array")},
		/*Required=*/{TEXT("target")});
	RegEditor(TEXT("list_actors"), TEXT("outliner"),
		GameThreadHandler(&HandleListActors),
		{TEXT("op:string"), TEXT("actors:array"), TEXT("in_pie:bool")});
	RegEditor(TEXT("select_actor"), TEXT("outliner"),
		GameThreadHandler(&HandleSelectActor),
		/*Reply=*/{TEXT("op:string"), TEXT("target:string")},
		/*Required=*/{TEXT("target")});
	RegEditor(TEXT("add_quick_convert"), TEXT("outliner"),
		GameThreadHandler(&HandleAddQuickConvert),
		/*Reply=*/{TEXT("op:string"), TEXT("target:string")},
		/*Required=*/{TEXT("target")});
	RegEditor(TEXT("remove_quick_convert"), TEXT("outliner"),
		GameThreadHandler(&HandleRemoveQuickConvert),
		/*Reply=*/{TEXT("op:string"), TEXT("target:string")},
		/*Required=*/{TEXT("target")});
	RegEditor(TEXT("list_blueprints"), TEXT("outliner"),
		GameThreadHandler(&HandleListBlueprints),
		{TEXT("op:string"), TEXT("blueprints:array")});

	// debug namespace: DrawDebug* primitives (editor + PIE).
	RegEditor(TEXT("draw_marker"), TEXT("debug"),
		GameThreadHandler(&HandleDrawMarker),
		/*Reply=*/{TEXT("op:string")},
		/*Required=*/{TEXT("location"), TEXT("color")});
	RegEditor(TEXT("draw_line"), TEXT("debug"),
		GameThreadHandler(&HandleDrawLine),
		/*Reply=*/{TEXT("op:string")},
		/*Required=*/{TEXT("from"), TEXT("to"), TEXT("color")});
	RegEditor(TEXT("draw_box"), TEXT("debug"),
		GameThreadHandler(&HandleDrawBox),
		/*Reply=*/{TEXT("op:string")},
		/*Required=*/{TEXT("center"), TEXT("half_extents"), TEXT("color")});
	RegEditor(TEXT("draw_axes"), TEXT("debug"),
		GameThreadHandler(&HandleDrawAxes),
		/*Reply=*/{TEXT("op:string")},
		/*Required=*/{TEXT("location")});
	RegEditor(TEXT("draw_arrow"), TEXT("debug"),
		GameThreadHandler(&HandleDrawArrow),
		/*Reply=*/{TEXT("op:string")},
		/*Required=*/{TEXT("from"), TEXT("to"), TEXT("color")});
	RegEditor(TEXT("clear_markers"), TEXT("debug"),
		GameThreadHandler(&HandleClearMarkers),
		{TEXT("op:string")});
	RegEditor(TEXT("set_overlay_text"), TEXT("debug"),
		GameThreadHandler(&HandleSetOverlayText),
		/*Reply=*/{TEXT("op:string")},
		/*Required=*/{TEXT("text")});

	// viewport namespace: perspective viewport control. Editor-only.
	RegEditor(TEXT("set_camera"), TEXT("viewport"),
		GameThreadHandler(&HandleViewportSetCamera),
		/*Reply=*/{TEXT("op:string"), TEXT("location:array"), TEXT("rotation_quat:array"), TEXT("rotation_euler:array"), TEXT("fov:float")},
		/*Required=*/{TEXT("location")});
	RegEditor(TEXT("get_camera"), TEXT("viewport"),
		GameThreadHandler(&HandleViewportGetCamera),
		{TEXT("op:string"), TEXT("location:array"),
			TEXT("rotation_quat:array"), TEXT("rotation_euler:array"),
			TEXT("fov:float")});
	RegEditor(TEXT("frame_actor"), TEXT("viewport"),
		GameThreadHandler(&HandleViewportFrameActor),
		/*Reply=*/{TEXT("op:string"), TEXT("location:array"), TEXT("rotation_quat:array"), TEXT("rotation_euler:array"), TEXT("fov:float")},
		/*Required=*/{TEXT("target")});
	RegEditor(TEXT("set_viewport_mode"), TEXT("viewport"),
		GameThreadHandler(&HandleViewportSetMode),
		/*Reply=*/{TEXT("op:string"), TEXT("mode:string")},
		/*Required=*/{TEXT("mode")});
	RegEditor(TEXT("track_actor"), TEXT("viewport"),
		GameThreadHandler(&HandleViewportTrackActor),
		/*Reply=*/{TEXT("op:string"), TEXT("tracked_actor_path:string")},
		/*Required=*/{TEXT("target")});
	RegEditor(TEXT("untrack"), TEXT("viewport"),
		GameThreadHandler(&HandleViewportUntrack),
		{TEXT("op:string"), TEXT("was_tracking:bool")});
}

void UnregisterAll()
{
	URLabOpRegistry::UnregisterHandler(TEXT("import_xml"));
	URLabOpRegistry::UnregisterHandler(TEXT("create_level"));
	URLabOpRegistry::UnregisterHandler(TEXT("ensure_manager"));
	URLabOpRegistry::UnregisterHandler(TEXT("destroy_asset"));
	URLabOpRegistry::UnregisterHandler(TEXT("current_level"));
	URLabOpRegistry::UnregisterHandler(TEXT("snapshot"));
	URLabOpRegistry::UnregisterHandler(TEXT("duplicate_actor"));
	URLabOpRegistry::UnregisterHandler(TEXT("actor_hierarchy"));
	URLabOpRegistry::UnregisterHandler(TEXT("find_actors"));
	URLabOpRegistry::UnregisterHandler(TEXT("get_actor_bounds"));
	URLabOpRegistry::UnregisterHandler(TEXT("load_level"));
	URLabOpRegistry::UnregisterHandler(TEXT("save_level"));
	URLabOpRegistry::UnregisterHandler(TEXT("spawn_actor"));
	URLabOpRegistry::UnregisterHandler(TEXT("spawn_grid"));
	URLabOpRegistry::UnregisterHandler(TEXT("spawn_light"));
	URLabOpRegistry::UnregisterHandler(TEXT("destroy_actor"));
	URLabOpRegistry::UnregisterHandler(TEXT("set_actor_transform"));
	URLabOpRegistry::UnregisterHandler(TEXT("begin_pie"));
	URLabOpRegistry::UnregisterHandler(TEXT("stop_pie"));
	URLabOpRegistry::UnregisterHandler(TEXT("pie_status"));
	URLabOpRegistry::UnregisterHandler(TEXT("list_actors"));
	URLabOpRegistry::UnregisterHandler(TEXT("select_actor"));
	URLabOpRegistry::UnregisterHandler(TEXT("add_quick_convert"));
	URLabOpRegistry::UnregisterHandler(TEXT("remove_quick_convert"));
	URLabOpRegistry::UnregisterHandler(TEXT("list_blueprints"));
	URLabOpRegistry::UnregisterHandler(TEXT("draw_marker"));
	URLabOpRegistry::UnregisterHandler(TEXT("draw_line"));
	URLabOpRegistry::UnregisterHandler(TEXT("draw_box"));
	URLabOpRegistry::UnregisterHandler(TEXT("draw_axes"));
	URLabOpRegistry::UnregisterHandler(TEXT("draw_arrow"));
	URLabOpRegistry::UnregisterHandler(TEXT("clear_markers"));
	URLabOpRegistry::UnregisterHandler(TEXT("set_overlay_text"));
	URLabOpRegistry::UnregisterHandler(TEXT("set_camera"));
	URLabOpRegistry::UnregisterHandler(TEXT("get_camera"));
	URLabOpRegistry::UnregisterHandler(TEXT("frame_actor"));
	URLabOpRegistry::UnregisterHandler(TEXT("set_viewport_mode"));
	URLabOpRegistry::UnregisterHandler(TEXT("track_actor"));
	URLabOpRegistry::UnregisterHandler(TEXT("untrack"));
	StopTrackingCameraInternal();
}
} // namespace URLabEditorOpHandlers
