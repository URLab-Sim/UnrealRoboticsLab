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

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"
#include "UserChannels/MjUserChannelComponent.h"
#include "State/MjStateTypes.h"
#include "State/MjStateCollector.h"
#include "State/MjMsgpackEncoder.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "Bridge/RpcDispatcher.h"
#include "GameFramework/Actor.h"

namespace
{
using EObservationLevel = FURLabRpcDispatcher::EObservationLevel;

const FMjUserChannel* FindChannel(const TArray<FMjUserChannel>& Channels, const TCHAR* Name)
{
	const FName Target(Name);
	for (const FMjUserChannel& C : Channels)
	{
		if (C.Name == Target)
			return &C;
	}
	return nullptr;
}
} // namespace

// ============================================================================
// URLab.UserChannels.ArtScope_CollectorAndEncoder
//   A component on an articulation actor publishes a bool + a transform; the
//   collector lands them in that art's IR block with the right kinds and raw
//   MuJoCo values, and the msgpack encoder emits them under "user".
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjUserChannelArtScope,
	"URLab.UserChannels.ArtScope_CollectorAndEncoder",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjUserChannelArtScope::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		return false;
	}

	UMjUserChannelComponent* Comp = NewObject<UMjUserChannelComponent>(S.Robot, TEXT("UserChannels"));
	Comp->RegisterComponent();
	S.Manager->RegisterStateProducer(Comp);

	Comp->PublishBool(TEXT("task_done"), true);
	// (100, 200, 300) cm, identity rotation -> MuJoCo (1, -2, 3) m, wxyz (1,0,0,0).
	Comp->PublishTransform(TEXT("target"),
		FTransform(FQuat::Identity, FVector(100.0, 200.0, 300.0)), /*bConvertFromUESpace=*/true);

	// Drive the game-thread cache rebuild synchronously (BeginPlay is bypassed in
	// test worlds, so scope resolution has to be triggered explicitly).
	FMjStateCollector& Collector = S.Manager->GetStateCollector();
	Collector.Init(S.Manager);
	Collector.RebuildProducerCacheGameThread();

	mjModel* M = S.Manager->PhysicsEngine->m_model;
	mjData* D = S.Manager->PhysicsEngine->m_data;
	const FMjStateSnapshot& Snap = Collector.Collect(M, D, 0);

	if (!TestEqual(TEXT("one articulation"), Snap.Articulations.Num(), 1))
	{
		S.Cleanup();
		return false;
	}
	const FMjArticulationState& Art = Snap.Articulations[0];

	// --- Bool channel ---
	const FMjUserChannel* Done = FindChannel(Art.UserChannels, TEXT("task_done"));
	if (TestNotNull(TEXT("task_done present"), Done))
	{
		TestEqual(TEXT("task_done kind"), (int32)Done->Kind, (int32)EMjUserChannelKind::Bool);
		TestTrue(TEXT("task_done value"), Done->Values.Num() == 1 && Done->Values[0] != 0.0);
	}

	// --- Transform channel (raw MuJoCo SI) ---
	const FMjUserChannel* Target = FindChannel(Art.UserChannels, TEXT("target"));
	if (TestNotNull(TEXT("target present"), Target))
	{
		TestEqual(TEXT("target kind"), (int32)Target->Kind, (int32)EMjUserChannelKind::Transform);
		if (TestEqual(TEXT("target width"), Target->Values.Num(), 7))
		{
			TestTrue(TEXT("target pos x"), MjTestMath::NearlyEqual(Target->Values[0], 1.0));
			TestTrue(TEXT("target pos y"), MjTestMath::NearlyEqual(Target->Values[1], -2.0));
			TestTrue(TEXT("target pos z"), MjTestMath::NearlyEqual(Target->Values[2], 3.0));
			TestTrue(TEXT("target quat w"), MjTestMath::NearlyEqual(Target->Values[3], 1.0));
			TestTrue(TEXT("target quat x"), MjTestMath::NearlyEqual(Target->Values[4], 0.0));
			TestTrue(TEXT("target quat y"), MjTestMath::NearlyEqual(Target->Values[5], 0.0));
			TestTrue(TEXT("target quat z"), MjTestMath::NearlyEqual(Target->Values[6], 0.0));
		}
	}

	// --- Encoder: the channels appear under arts/<art>/user ---
	TSharedPtr<FJsonObject> Encoded = FMjMsgpackEncoder::EncodeSnapshot(Snap, EObservationLevel::Full);
	const TSharedPtr<FJsonObject>* ArtsObj = nullptr;
	if (TestTrue(TEXT("arts block"), Encoded->TryGetObjectField(TEXT("arts"), ArtsObj)))
	{
		const TSharedPtr<FJsonObject>* ArtObj = nullptr;
		if (TestTrue(TEXT("art entry"), (*ArtsObj)->TryGetObjectField(Art.Name.ToString(), ArtObj)))
		{
			const TSharedPtr<FJsonObject>* UserObj = nullptr;
			if (TestTrue(TEXT("user block"), (*ArtObj)->TryGetObjectField(TEXT("user"), UserObj)))
			{
				bool bDone = false;
				TestTrue(TEXT("encoded task_done readable"), (*UserObj)->TryGetBoolField(TEXT("task_done"), bDone));
				TestTrue(TEXT("encoded task_done true"), bDone);

				const TSharedPtr<FJsonObject>* TargetObj = nullptr;
				if (TestTrue(TEXT("encoded target object"), (*UserObj)->TryGetObjectField(TEXT("target"), TargetObj)))
				{
					const TArray<TSharedPtr<FJsonValue>>* Pos = nullptr;
					if (TestTrue(TEXT("encoded target pos"), (*TargetObj)->TryGetArrayField(TEXT("pos"), Pos))
						&& TestEqual(TEXT("encoded pos width"), Pos->Num(), 3))
					{
						TestTrue(TEXT("encoded pos x"), MjTestMath::NearlyEqual((*Pos)[0]->AsNumber(), 1.0));
						TestTrue(TEXT("encoded pos y"), MjTestMath::NearlyEqual((*Pos)[1]->AsNumber(), -2.0));
						TestTrue(TEXT("encoded pos z"), MjTestMath::NearlyEqual((*Pos)[2]->AsNumber(), 3.0));
					}
					const TArray<TSharedPtr<FJsonValue>>* Quat = nullptr;
					TestTrue(TEXT("encoded target quat"), (*TargetObj)->TryGetArrayField(TEXT("quat"), Quat));
				}
			}
		}
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.UserChannels.SceneScope_CollectorAndEncoder
//   A component on a non-articulation actor publishes into the scene scope; the
//   channel lands on the snapshot itself and encodes as a top-level "user" block.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjUserChannelSceneScope,
	"URLab.UserChannels.SceneScope_CollectorAndEncoder",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjUserChannelSceneScope::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		return false;
	}

	// A plain actor (not an AMjArticulation) => scene scope.
	AActor* SceneActor = S.World->SpawnActor<AActor>();
	if (!TestNotNull(TEXT("scene actor"), SceneActor))
	{
		S.Cleanup();
		return false;
	}

	UMjUserChannelComponent* Comp = NewObject<UMjUserChannelComponent>(SceneActor, TEXT("SceneChannels"));
	Comp->RegisterComponent();
	S.Manager->RegisterStateProducer(Comp);

	Comp->PublishInt(TEXT("episode_phase"), 2);

	FMjStateCollector& Collector = S.Manager->GetStateCollector();
	Collector.Init(S.Manager);
	Collector.RebuildProducerCacheGameThread();

	mjModel* M = S.Manager->PhysicsEngine->m_model;
	mjData* D = S.Manager->PhysicsEngine->m_data;
	const FMjStateSnapshot& Snap = Collector.Collect(M, D, 0);

	const FMjUserChannel* Phase = FindChannel(Snap.UserChannels, TEXT("episode_phase"));
	if (TestNotNull(TEXT("episode_phase present"), Phase))
	{
		TestEqual(TEXT("episode_phase kind"), (int32)Phase->Kind, (int32)EMjUserChannelKind::Int);
		TestTrue(TEXT("episode_phase value"), Phase->Values.Num() == 1 && FMath::IsNearlyEqual(Phase->Values[0], 2.0));
	}

	// No art channel leakage: the art block must not carry the scene channel.
	if (Snap.Articulations.Num() == 1)
		TestNull(TEXT("not on art"), FindChannel(Snap.Articulations[0].UserChannels, TEXT("episode_phase")));

	TSharedPtr<FJsonObject> Encoded = FMjMsgpackEncoder::EncodeSnapshot(Snap, EObservationLevel::Full);
	const TSharedPtr<FJsonObject>* UserObj = nullptr;
	if (TestTrue(TEXT("top-level user block"), Encoded->TryGetObjectField(TEXT("user"), UserObj)))
	{
		double PhaseVal = 0.0;
		TestTrue(TEXT("encoded episode_phase readable"), (*UserObj)->TryGetNumberField(TEXT("episode_phase"), PhaseVal));
		TestTrue(TEXT("encoded episode_phase value"), FMath::IsNearlyEqual(PhaseVal, 2.0));
	}

	S.Cleanup();
	return true;
}
