// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjSceneAssembly.h"

#include "Algo/StableSort.h"
#include "Internationalization/Regex.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

namespace
{
/** Bytes collected for one participant, in spec order. */
class FMjVfsCollector final : public IMjAssetSink
{
public:
	TArray<FMjVfsAsset> Assets;

	void OnMesh(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }
	void OnTexture(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }
	void OnHeightField(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }

private:
	void Take(const FMjAssetRequest& Request, const TArray<uint8>& Bytes)
	{
		if (Request.VfsName.IsEmpty() || Bytes.Num() == 0)
		{
			return;
		}
		Assets.Add(FMjVfsAsset{Request.VfsName, Bytes});
	}
};

/**
 * Re-point a participant's `file=` references at the names it is mounted under.
 *
 * `CollectAssets` mounts every asset under a participant-prefixed basename,
 * because MuJoCo's VFS falls back to a case-insensitive basename match across
 * every mount and would otherwise hand one participant another's mesh. The
 * spec text has to name the same thing, and it is the writer's output rather
 * than the spec that is rewritten, so nothing authored moves.
 */
FString PrefixAssetRefs(const FString& Xml, const FString& Prefix)
{
	if (Prefix.IsEmpty())
	{
		return Xml;
	}
	FRegexPattern Pattern(TEXT("file=\"([^\"]*?)([^/\\\\\"]+)\""));
	FRegexMatcher Matcher(Pattern, Xml);
	FString Out;
	int32 Cursor = 0;
	while (Matcher.FindNext())
	{
		Out += Xml.Mid(Cursor, Matcher.GetMatchBeginning() - Cursor);
		Out += FString::Printf(TEXT("file=\"%s%s\""), *Prefix, *Matcher.GetCaptureGroup(2));
		Cursor = Matcher.GetMatchEnding();
	}
	Out += Xml.Mid(Cursor);
	return Out;
}

FString EscapeXmlAttribute(const FString& In)
{
	FString Out = In;
	Out.ReplaceInline(TEXT("&"), TEXT("&amp;"));
	Out.ReplaceInline(TEXT("<"), TEXT("&lt;"));
	Out.ReplaceInline(TEXT(">"), TEXT("&gt;"));
	Out.ReplaceInline(TEXT("\""), TEXT("&quot;"));
	return Out;
}

/** MuJoCo's shortest-round-trip spelling is the engine's; this only has to parse back. */
FString Number(double Value)
{
	return FString::Printf(TEXT("%.17g"), Value);
}
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

TArray<FMjVfsAsset> FSceneAssembly::CollectAssets() const
{
	TArray<FMjVfsAsset> Out;
	for (const FMjSceneParticipant& Participant : GetParticipants())
	{
		FMjVfsCollector Collector;
		FMjAssetSink Sink(Collector);
		Sink.VfsPrefix = Participant.Prefix;
		Sink.Collect(Participant.Spec);
		Out.Append(MoveTemp(Collector.Assets));
	}
	return Out;
}

TMap<FString, FString> FSceneAssembly::CollectAssetFiles() const
{
	TMap<FString, FString> Out;
	for (const FMjSceneParticipant& Participant : GetParticipants())
	{
		FMjVfsCollector Collector;
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

FString FSceneAssembly::WriteSceneMjcf(TMap<FString, FString>& OutParticipantXml,
	TArray<FMjSpecDiagnostic>* OutErrors) const
{
	OutParticipantXml.Reset();

	FString SceneSections;
	if (SceneRoot.IsValid())
	{
		// The manager's sections come from its own spec, written by the same
		// writer as everything else; only the attach rows below are assembled.
		const FString RootXml = SceneRoot.WriteMjcf(OutErrors);
		int32 Open = INDEX_NONE;
		int32 Close = INDEX_NONE;
		if (RootXml.FindChar(TEXT('\n'), Open) && RootXml.FindLastChar(TEXT('<'), Close) && Close > Open)
		{
			SceneSections = RootXml.Mid(Open + 1, Close - Open - 1);
		}
	}

	FString Out;
	Out += TEXT("<mujoco model=\"scene\">\n");
	Out += SceneSections;

	Out += TEXT("  <asset>\n");
	for (const FMjSceneParticipant& Participant : GetParticipants())
	{
		const FString VfsName = Participant.Prefix + TEXT("model.xml");
		OutParticipantXml.Add(VfsName, PrefixAssetRefs(Participant.Spec.WriteMjcf(OutErrors), Participant.Prefix));
		Out += FString::Printf(TEXT("    <model name=\"%s\" file=\"%s\"/>\n"),
			*EscapeXmlAttribute(Participant.Prefix), *EscapeXmlAttribute(VfsName));
	}
	Out += TEXT("  </asset>\n");

	Out += TEXT("  <worldbody>\n");
	for (const FMjSceneParticipant& Participant : GetParticipants())
	{
		Out += FString::Printf(
			TEXT("    <frame pos=\"%s %s %s\" quat=\"%s %s %s %s\">\n"
				 "      <attach model=\"%s\" prefix=\"%s\"/>\n"
				 "    </frame>\n"),
			*Number(Participant.MjPos.X), *Number(Participant.MjPos.Y), *Number(Participant.MjPos.Z),
			*Number(Participant.MjQuat.W), *Number(Participant.MjQuat.X), *Number(Participant.MjQuat.Y),
			*Number(Participant.MjQuat.Z), *EscapeXmlAttribute(Participant.Prefix),
			*EscapeXmlAttribute(Participant.Prefix));
	}
	Out += TEXT("  </worldbody>\n");
	Out += TEXT("</mujoco>\n");
	return Out;
}
