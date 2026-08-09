// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjSceneAssembly.h"

#include "Algo/StableSort.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

namespace
{
/**
 * A sink that wants the requests and not the bytes.
 *
 * The ship-list is a list of files, so the pass runs with `bLoadBytes` off and
 * the callbacks never fire; the interface still has to be satisfied.
 */
class FMjAssetFileCollector final : public IMjAssetSink
{
public:
	void OnMesh(const FMjAssetRequest&, const TArray<uint8>&) override {}
	void OnTexture(const FMjAssetRequest&, const TArray<uint8>&) override {}
	void OnHeightField(const FMjAssetRequest&, const TArray<uint8>&) override {}
};
}  // namespace

void FSceneAssembly::Add(const FSpecRef& Spec, const FString& Prefix, const FVector& MjPos, const FQuat& MjQuat)
{
	if (!Spec.IsValid())
	{
		return;
	}
	FMjSceneParticipant Participant;
	Participant.Spec = Spec;
	Participant.Prefix = Prefix;
	Participant.MjPos = MjPos;
	Participant.MjQuat = MjQuat;
	Participants.Add(MoveTemp(Participant));
	bSorted = false;
}

void FSceneAssembly::SetSceneRoot(const FSpecRef& Spec)
{
	SceneRoot = Spec;
}

void FSceneAssembly::SortParticipants() const
{
	if (bSorted)
	{
		return;
	}
	Algo::StableSort(Participants, [](const FMjSceneParticipant& A, const FMjSceneParticipant& B) {
		return A.Prefix.Compare(B.Prefix, ESearchCase::CaseSensitive) < 0;
	});
	bSorted = true;
}

const TArray<FMjSceneParticipant>& FSceneAssembly::GetParticipants() const
{
	SortParticipants();
	return Participants;
}

TMap<FString, FString> FSceneAssembly::CollectAssetFiles() const
{
	TMap<FString, FString> Out;
	for (const FMjSceneParticipant& Participant : GetParticipants())
	{
		FMjAssetFileCollector Collector;
		FMjAssetSink Sink(Collector);
		Sink.VfsPrefix = Participant.Prefix;
		Sink.bLoadBytes = false;
		Sink.Collect(Participant.Spec);
		for (const FMjAssetRequest& Request : Sink.GetRequests())
		{
			// Keyed by the mounted name rather than the path, because two
			// participants referencing one file mount it once each, under their
			// own prefixes, and both entries have to exist.
			if (!Request.bMissing && !Request.ResolvedPath.IsEmpty())
			{
				Out.Add(Request.VfsName, Request.ResolvedPath);
			}
		}
	}
	return Out;
}
