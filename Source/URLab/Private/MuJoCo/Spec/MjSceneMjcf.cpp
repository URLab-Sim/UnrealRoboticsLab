// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjSceneMjcf.h"

#include "Internationalization/Regex.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

#if URLAB_MJ_GEN
#include "MjReservedNames.h"
#include "MjSpecNodes.h"
#endif

namespace
{
/**
 * Re-point a participant's `file=` references at the names it is mounted under.
 *
 * A scene mounts every asset under a participant-prefixed basename, because
 * MuJoCo's VFS falls back to a case-insensitive basename match across every
 * mount and would otherwise hand one participant another's mesh. The text has
 * to name the same thing, and it is the writer's output rather than the spec
 * that is rewritten, so nothing authored moves.
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

#if URLAB_MJ_GEN

/**
 * Pin the name MuJoCo would derive for a file-backed asset that authors none.
 *
 * Such an asset is not anonymous to MuJoCo: it takes the file's basename, and
 * that is the name every material or geom referring to it was written against.
 * The scene prefixes `file=` so two participants cannot collide in the VFS,
 * which moves that derived name and dangles the references. Reserving the
 * basename pins it.
 *
 * The spec is handed back exactly as it was found: the names exist for the
 * duration of one write and are then taken off again. Authoring them for real
 * would put identity nobody asked for into the user's Blueprint, and it would
 * show up in the next diff of their MJCF.
 */
class FMjAssetNames
{
public:
	explicit FMjAssetNames(const TArray<FMjSceneParticipant>& Participants)
	{
		for (const FMjSceneParticipant& Participant : Participants)
		{
			const urlab::spec::FMjSpecNodes Tree = urlab::spec::MjSpecNodesOf(Participant.Spec);
			for (UMjNodeComponent* Node : Tree.Nodes)
			{
				if (Node == nullptr || Tree.Unnamable.Contains(Node))
				{
					continue;
				}
				if (Node->MjName.IsSet() && !Node->MjName.GetValue().IsEmpty())
				{
					continue;
				}
				const FString Derived = MjAssetElementName(*Node);
				if (Derived.IsEmpty())
				{
					continue;
				}
				Node->MjName = Derived;
				Renamed.Add(Node);
			}
		}
	}

	~FMjAssetNames()
	{
		for (UMjNodeComponent* Node : Renamed)
		{
			Node->MjName.Reset();
		}
	}

	FMjAssetNames(const FMjAssetNames&) = delete;
	FMjAssetNames& operator=(const FMjAssetNames&) = delete;

private:
	TArray<UMjNodeComponent*> Renamed;
};

#endif  // URLAB_MJ_GEN

/** The document itself, with every name already on the elements it belongs to. */
FString WriteScene(const FSceneAssembly& Scene, TMap<FString, FString>& OutParticipantXml,
	TArray<FMjSpecDiagnostic>* OutErrors)
{
	OutParticipantXml.Reset();

	FString SceneSections;
	const FSpecRef SceneRoot = Scene.GetSceneRoot();
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
	for (const FMjSceneParticipant& Participant : Scene.GetParticipants())
	{
		const FString VfsName = Participant.Prefix + TEXT("model.xml");
		OutParticipantXml.Add(VfsName, PrefixAssetRefs(Participant.Spec.WriteMjcf(OutErrors), Participant.Prefix));
		Out += FString::Printf(TEXT("    <model name=\"%s\" file=\"%s\"/>\n"),
			*EscapeXmlAttribute(Participant.Prefix), *EscapeXmlAttribute(VfsName));
	}
	Out += TEXT("  </asset>\n");

	Out += TEXT("  <worldbody>\n");
	for (const FMjSceneParticipant& Participant : Scene.GetParticipants())
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
}  // namespace

FString MjWriteSceneMjcf(const FSceneAssembly& Scene, TMap<FString, FString>& OutParticipantXml,
	TArray<FMjSpecDiagnostic>* OutErrors)
{
#if URLAB_MJ_GEN
	// Both halves of the naming, held for the whole write and no longer: the
	// derived basename of a file-backed asset, and the reserved name of anything
	// else the document left unnamed.
	const FMjAssetNames AssetNames(Scene.GetParticipants());

	TArray<TUniquePtr<urlab::spec::FMjReservedNames>> Reserved;
	Reserved.Reserve(Scene.GetParticipants().Num());
	for (const FMjSceneParticipant& Participant : Scene.GetParticipants())
	{
		Reserved.Add(MakeUnique<urlab::spec::FMjReservedNames>(Participant.Spec));
	}
#endif
	return WriteScene(Scene, OutParticipantXml, OutErrors);
}
