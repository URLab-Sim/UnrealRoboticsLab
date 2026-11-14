#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "MJSpecHandle.h"
#include "MJAssetLibrary.generated.h"

class UStaticMesh;
class UTexture2D;

USTRUCT(BlueprintType)
struct URLAB_API FMJRawAssetBuffer
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Assets")
	TArray<uint8> Bytes;
};

/**
 * Stores UE mesh/texture assets and raw binary buffers fed into MuJoCo 4 mjVFS.
 * Supports MuJoCo 4 by-reference textures (tex_adr == -1 / mj_loadTexture / mj_embedAssets).
 */
UCLASS(BlueprintType)
class URLAB_API UMJAssetLibrary : public UDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Assets")
	TMap<FName, TObjectPtr<UStaticMesh>> Meshes;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Assets")
	TMap<FName, TObjectPtr<UTexture2D>> Textures;

	/** Raw file payloads (PNG, OBJ, STL, XML, etc.) keyed by VFS filename. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Assets")
	TMap<FString, FMJRawAssetBuffer> RawBuffers;

	/** Adds all RawBuffers into Vfs via mj_addBufferVFS. Returns number of buffers added. */
	int32 PopulateVFS(mjVFS* Vfs) const;

	/** Returns true if texture TexId in M is a MuJoCo 4 by-reference texture (tex_adr == -1). */
	static bool IsTextureByReference(const mjModel* M, int32 TexId);

	/** Decodes a texture (including by-reference textures) via mj_loadTexture. */
	static bool LoadTextureFromModel(
		const mjModel* M,
		int32 TexId,
		TArray<uint8>& OutPixels,
		int32& OutWidth,
		int32& OutHeight,
		int32& OutColorSpace,
		FString& OutError);

	/** Creates a new mjModel with all by-reference assets embedded via mj_embedAssets. */
	static bool EmbedModelAssets(const mjModel* M, FMJModelHandle& OutEmbedded, FString& OutError);
};
