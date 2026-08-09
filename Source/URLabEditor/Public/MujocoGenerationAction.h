// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// MJCF into a Blueprint, plus the Unreal assets that come with it.
//
// The reading is not done here. `MjParseIntoBlueprint` turns MJCF text into the
// Blueprint's construction-script templates, and that tree IS the spec --
// includes expanded, default classes in place, every section read. What is left
// for the editor is the part a spec does not carry: a UStaticMesh for a
// <mesh> and a UTexture2D for a <texture>, so the components panel has something
// to show.
//
// Those assets are a pass over the parsed spec rather than a step of the
// parse, because an asset path is only decidable once the whole spec is in
// the tree: a <compiler meshdir> can arrive from an included file, and each
// element resolves against the file it was itself read from.
//
// It is an asset action so that one implementation serves the right-click menu,
// the drag-and-drop import factory, and the new-articulation factory.

#include "CoreMinimal.h"

#include "AssetActionUtility.h"

#include "MuJoCo/Spec/MjSpecRef.h"

#include "MujocoGenerationAction.generated.h"

class UBlueprint;

UCLASS()
class URLABEDITOR_API UMujocoGenerationAction : public UAssetActionUtility
{
	GENERATED_BODY()

public:
	UMujocoGenerationAction();

	/**
	 * Re-read every selected articulation Blueprint from the MJCF it names.
	 *
	 * The path comes from each Blueprint's own `MuJoCoXMLFile`, so this is the
	 * "the XML changed, pick it up again" action rather than a fresh import.
	 */
	UFUNCTION(CallInEditor, Category = "MuJoCo")
	void GenerateMuJoCoComponents();

	/**
	 * Replace `Blueprint`'s spec with the one in `XmlPath`, assets and all.
	 *
	 * Replace, not merge: whatever spec the Blueprint already held is taken
	 * out first, because a construction script holding two spec roots
	 * answers every later question from whichever one it happens to reach first.
	 */
	bool GenerateForBlueprint(UBlueprint* Blueprint, const FString& XmlPath,
		const FMjDocParseOptions& Options = {});

	/**
	 * As above, for MJCF text already in hand.
	 *
	 * `Filename` is never opened. It is what `<include>` and every asset path
	 * resolve against, and it names the content folder the imported assets land
	 * in, so a caller holding text from somewhere else still has to say where
	 * that text would have lived.
	 */
	bool GenerateFromXml(UBlueprint* Blueprint, const FString& Xml, const FString& Filename,
		const FMjDocParseOptions& Options = {});

	/** Give a fresh articulation Blueprint an empty spec to author into. */
	void SetupEmptyArticulation(UBlueprint* Blueprint);
};
