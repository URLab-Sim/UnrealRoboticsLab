// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Asset bytes out of a parsed spec and into Unreal's importers.
//
// A <mesh>, <texture>, <hfield> or <skin> is an ordinary generated element like
// any other, and the spec is complete without any Unreal asset existing. What
// the editor additionally wants is a UStaticMesh or a UTexture2D to preview
// with, and that is what this produces: it walks the spec's asset section,
// resolves each element's `file` against the directories the spec itself
// declares, and hands the bytes to a sink.
//
// It is a pass over the spec rather than a reader callback because the
// resolution it performs -- meshdir and texturedir from <compiler>, relative to
// the file the element came from -- is only decidable once the whole spec is
// in the tree, includes expanded and compiler blocks folded. That is also why it
// can run against a spec nobody parsed: an edited tree collects assets the
// same way.
//
// The importer itself is untouched. This produces (element, name, path, bytes)
// and stops; whether that becomes a UStaticMesh, a VFS entry for a compile, or
// the handshake's ship-list is the sink's business.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

class UMjNodeComponent;
struct FSpecRef;

/** One asset an element references. */
struct URLAB_API FMjAssetRequest
{
	/** The element that declared it. */
	UMjNodeComponent* Element = nullptr;

	/** Its MJCF name, or the file's base name when the element is unnamed. */
	FString Name;

	/**
	 * The path the spec referenced, resolved against the element's source
	 * directory and the spec's meshdir / texturedir. Empty for an inline
	 * asset (vertex/face data authored in the MJCF itself).
	 */
	FString ResolvedPath;

	/**
	 * The directory `file` was resolved against.
	 *
	 * The element's own source directory with the spec's meshdir or
	 * texturedir folded in -- the same rule MuJoCo applies. Empty for a spec
	 * parsed from text that came from no file. The export pass needs it because
	 * a file it writes has to land where this rule will find it again.
	 */
	FString BaseDirectory;

	/** The file's basename, prefixed when the request came from a scene assembly. */
	FString VfsName;

	/**
	 * True when the path could not be resolved to a readable file.
	 *
	 * Answered by the resolution, so it is as true of a pass that reads no
	 * bytes as of one that does.
	 */
	bool bMissing = false;
};

/**
 * Where resolved asset bytes go.
 *
 * Implemented in URLabEditor over the existing mesh, texture and material
 * importers, and in the compile path over the in-memory VFS. Both see the same
 * requests in the same order, which is what keeps the editor's imported assets
 * and the compiled model's assets the same set.
 */
class URLAB_API IMjAssetSink
{
public:
	virtual ~IMjAssetSink() = default;

	/** A mesh asset. `Bytes` is empty when the element carries inline geometry. */
	virtual void OnMesh(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) = 0;

	/** A texture asset. */
	virtual void OnTexture(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) = 0;

	/** A height field. */
	virtual void OnHeightField(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) = 0;

	/**
	 * A skin's `.skn` file.
	 *
	 * Defaulted to ignoring it, unlike the three above, because a skin is a
	 * file the COMPILER needs and not an asset Unreal has anything to make of:
	 * there is no UStaticMesh or UTexture2D at the end of it. A sink that
	 * mounts bytes for a compile overrides this; a sink that imports Unreal
	 * assets, or writes files back out beside a model, correctly does not.
	 */
	virtual void OnSkin(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) {}

	/** An asset the spec names but no file backs. */
	virtual void OnMissing(const FMjAssetRequest& Request) {}
};

/**
 * The name every reference to an asset element uses.
 *
 * Its MJCF `name`, or -- when it has none -- the base filename of its `file`,
 * which is the name MuJoCo's own compiler derives. A card deck writes
 * `<texture type="2d" file="2_of_clubs.png"/>` and then `<material
 * texture="2_of_clubs">`, so the derived spelling is not a corner case.
 *
 * Empty for an unnamed element that names no file either, which is a builtin
 * texture nothing can refer to.
 */
URLAB_API FString MjAssetElementName(const UMjNodeComponent& Element);

/**
 * Where a file an element references resolves to.
 *
 * The element's own source directory with `AssetDir` folded in, and the file
 * taken as it stands when it is already absolute -- the rule MuJoCo applies.
 * `AssetDir` is the `<compiler>` directory for the asset's kind; a `<model>`
 * asset passes none, because the reader resolves a nested model against the
 * model file's own directory and through no asset directory at all
 * (`src/xml/xml_native_reader.cc:2303`). Empty when `File` is.
 */
URLAB_API FString MjResolveAssetPath(const UMjNodeComponent& Element, const FString& AssetDir, const FString& File);

/**
 * The spec's asset pass.
 *
 * `VfsPrefix` prefixes every emitted VfsName. It is not cosmetic: MuJoCo's VFS
 * falls back to a case-insensitive BASENAME match across all mounts, so two
 * articulations that each reference their own `base.obj` silently share whichever
 * one was mounted -- a missing asset becoming the wrong asset, with no warning.
 * Subdirectories do not help, because the fallback ignores them. A scene assembly
 * therefore mounts participant-prefixed basenames.
 */
struct URLAB_API FMjAssetSink
{
	explicit FMjAssetSink(IMjAssetSink& InSink) : Sink(&InSink) {}

	/** Prefix applied to every VfsName. Empty for a single-spec import. */
	FString VfsPrefix;

	/**
	 * Read the bytes as well as resolving the path.
	 *
	 * Off for a pass that only wants the paths: whether an asset is missing is
	 * reported either way, so nothing has to read a mesh to find out it exists.
	 */
	bool bLoadBytes = true;

	/** Walk `Spec`'s asset section and emit one request per asset element. */
	void Collect(const FSpecRef& Spec);

	/** The requests the last Collect emitted, in spec order. */
	const TArray<FMjAssetRequest>& GetRequests() const { return Requests; }

private:
	IMjAssetSink* Sink = nullptr;
	TArray<FMjAssetRequest> Requests;
};
