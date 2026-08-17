// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. This plugin incorporates
// third-party software: MuJoCo (Apache 2.0). See ThirdPartyNotices.txt.

#include "MuJoCo/Fast/MjRendererStepMode.h"

#include "MuJoCo/Fast/MjRenderer.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "TimerManager.h"
#include "Misc/Paths.h"
#include "Utils/URLabLogging.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
// Convert a MuJoCo 3x3 orientation (row-major geom_xmat/cam_xmat) to a UE quat,
// via a wxyz quaternion.
FQuat MjMat3ToUeQuat(const double* Mat3)
{
	double Quat[4];
	mju_mat2Quat(Quat, Mat3);
	return URLabAxisConv::MjQuatToUe(Quat);
}
} // namespace

void FMjRendererStepMode::Begin(AMjRenderer& Scene)
{
	if (!Scene.Model || !Scene.Data)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjRenderer] Direct: no model/data to install"));
		return;
	}

	Scene.EnsureManager();

	// The manager compiles an (empty) scene and starts its worker in its own
	// BeginPlay; installing before that would be undone. Poll until it has begun
	// play, then install once.
	Scene.GetWorld()->GetTimerManager().SetTimer(
		InstallTimer, &Scene, &AMjRenderer::InstallIntoEngine, 0.05f, /*bLoop=*/true);
}

void FMjRendererStepMode::InstallIntoEngine(AMjRenderer& Scene)
{
	AAMjManager* Mgr = Manager.Get();
	if (!Mgr)
	{
		Scene.GetWorld()->GetTimerManager().ClearTimer(InstallTimer);
		return;
	}
	if (!Mgr->HasActorBegunPlay())
	{
		return; // keep polling until the manager's own BeginPlay has run
	}
	Scene.GetWorld()->GetTimerManager().ClearTimer(InstallTimer);

	UMjPhysicsEngine* Eng = Mgr->PhysicsEngine;
	if (!Eng || !Scene.Model || !Scene.Data)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjRenderer] Direct: engine/model unavailable at install"));
		return;
	}
	// Name the raw entity from the MJB's base name so the control/observation RPC layer (and a
	// Python client) address it by a stable prefix -- set BEFORE install so the partition the install
	// builds carries the name, with no shadow articulation.
	const FString ArtId = FPaths::GetBaseFilename(Scene.MjbFilePath);
	const FString RawName = ArtId.IsEmpty() ? TEXT("fastpath") : ArtId;
	Eng->SetRawEntityIdentity(RawName, RawName);

	if (!Eng->InstallRawModel(Scene.Model, Scene.Data))
	{
		UE_LOG(LogURLab, Error, TEXT("[MjRenderer] Direct: InstallRawModel failed"));
		return;
	}

	// Free-run the sim when no client owns the clock: a client hello promotes the
	// engine to a client-driven step mode; until then this steps at real time.
	Eng->bIsPaused = false;
	Eng->RunMujocoAsync();
	UE_LOG(LogURLab, Log,
		TEXT("[MjRenderer] Direct: installed raw model (nq=%d nv=%d nu=%d) -- engine stepping"),
		(int)Scene.Model->nq, (int)Scene.Model->nv, (int)Scene.Model->nu);
}

void FMjRendererStepMode::ApplyFromSnapshot(AMjRenderer& Scene)
{
	AAMjManager* Mgr = Manager.Get();
	if (!Mgr || !Mgr->PhysicsEngine || !Scene.Model)
	{
		return;
	}
	const int32 NGeom = static_cast<int32>(Scene.Model->ngeom);
	const int32 NCam = Scene.CameraComps.Num();

	Mgr->PhysicsEngine->WithRenderState([this, &Scene, NGeom, NCam](const FMjRenderSnapshot& Snap)
	{
		// Skip a snapshot we have already drawn (the worker publishes one per step;
		// the game thread renders at its own, usually lower, rate).
		if (Snap.FrameId == LastRenderFrameId)
		{
			return;
		}
		LastRenderFrameId = Snap.FrameId;

		if (Snap.GeomXPos.Num() < NGeom * 3 || Snap.GeomXMat.Num() < NGeom * 9)
		{
			return;
		}
		int32 NanGeoms = 0;
		for (int32 G = 0; G < Scene.GeomComps.Num(); ++G)
		{
			UPrimitiveComponent* Comp = Scene.GeomComps[G];
			if (!Comp)
			{
				continue;
			}
			const FVector Loc = URLabAxisConv::MjPositionToUe(Snap.GeomXPos.GetData() + 3 * G) + Scene.SceneOrigin;
			const FQuat Rot = MjMat3ToUeQuat(Snap.GeomXMat.GetData() + 9 * G);
			// Never push a non-finite transform into a component: it poisons the
			// renderer (distance-field matrix inversion) and hides the real cause.
			if (Loc.ContainsNaN() || Rot.ContainsNaN() || !Rot.IsNormalized())
			{
				++NanGeoms;
				continue;
			}
			Comp->SetWorldLocationAndRotation(Loc, Rot);
		}
		if (NanGeoms > 0 && !bNanLogged)
		{
			bNanLogged = true;
			UE_LOG(LogURLab, Warning,
				TEXT("[MjRenderer] Direct: %d/%d geoms non-finite at frame %llu (simTime=%.4f) -- physics diverged or bad snapshot"),
				NanGeoms, Scene.GeomComps.Num(), (unsigned long long)Snap.FrameId, Snap.SimTime);
		}

		// Cameras track the stepped state too. The snapshot carries cam_xmat as a
		// 3x3; convert to the wxyz quats ApplyCameraPoses expects.
		if (NCam > 0 && Snap.CamXPos.Num() >= NCam * 3 && Snap.CamXMat.Num() >= NCam * 9)
		{
			TArray<double> Cxpos;
			TArray<double> Cxquat;
			Cxpos.SetNumUninitialized(NCam * 3);
			Cxquat.SetNumUninitialized(NCam * 4);
			for (int32 C = 0; C < NCam; ++C)
			{
				Cxpos[3 * C + 0] = Snap.CamXPos[3 * C + 0];
				Cxpos[3 * C + 1] = Snap.CamXPos[3 * C + 1];
				Cxpos[3 * C + 2] = Snap.CamXPos[3 * C + 2];
				mju_mat2Quat(Cxquat.GetData() + 4 * C, Snap.CamXMat.GetData() + 9 * C);
			}
			Scene.ApplyCameraPoses(Cxpos.GetData(), Cxquat.GetData());
		}
	});
}

void FMjRendererStepMode::Teardown(AMjRenderer& Scene)
{
	if (Scene.GetWorld())
	{
		Scene.GetWorld()->GetTimerManager().ClearTimer(InstallTimer);
	}
	// Direct mode aliased our raw model+data into the shared engine. Stop-join the
	// physics worker and unalias BEFORE the scene frees Model/Data, so the worker is
	// never mid-step against memory about to be freed.
	if (AAMjManager* Mgr = Manager.Get())
	{
		if (Mgr->PhysicsEngine)
		{
			Mgr->PhysicsEngine->UninstallRawModel();
		}
	}
	Manager.Reset();
}

void FMjRendererStepMode::RetireForReload()
{
	// Retire the installed model + shadow but KEEP the manager + engine so the swap
	// reuses the same physics + RPC context.
	if (AAMjManager* Mgr = Manager.Get())
	{
		if (Mgr->PhysicsEngine)
		{
			Mgr->PhysicsEngine->UninstallRawModel(); // stop-join worker + unalias
		}
	}
	LastRenderFrameId = 0;
	bNanLogged = false;
}
