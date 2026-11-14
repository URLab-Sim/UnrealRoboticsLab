#include "MJAssetLibrary.h"
#include "Utils/URLabLogging.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include <cstdlib>

int32 UMJAssetLibrary::PopulateVFS(mjVFS* Vfs) const
{
	if (!Vfs)
	{
		return 0;
	}

	int32 Added = 0;
	for (const TPair<FString, FMJRawAssetBuffer>& Pair : RawBuffers)
	{
		if (Pair.Key.IsEmpty() || Pair.Value.Bytes.Num() == 0)
		{
			continue;
		}
		FTCHARToUTF8 NameUtf8(*Pair.Key);
		const int Rc = mj_addBufferVFS(Vfs, NameUtf8.Get(), Pair.Value.Bytes.GetData(), Pair.Value.Bytes.Num());
		if (Rc == 0)
		{
			++Added;
		}
		else
		{
			UE_LOG(LogURLab, Warning, TEXT("mj_addBufferVFS failed for '%s' (rc=%d)"), *Pair.Key, Rc);
		}
	}
	return Added;
}

bool UMJAssetLibrary::IsTextureByReference(const mjModel* M, int32 TexId)
{
	if (!M || TexId < 0 || TexId >= M->ntex)
	{
		return false;
	}
	return M->tex_adr[TexId] == -1;
}

bool UMJAssetLibrary::LoadTextureFromModel(
	const mjModel* M,
	int32 TexId,
	TArray<uint8>& OutPixels,
	int32& OutWidth,
	int32& OutHeight,
	int32& OutColorSpace,
	FString& OutError)
{
	OutPixels.Reset();
	OutWidth = 0;
	OutHeight = 0;
	OutColorSpace = 0;

	if (!M || TexId < 0 || TexId >= M->ntex)
	{
		OutError = TEXT("Invalid mjModel or TexId");
		return false;
	}

	mjtByte* Data = nullptr;
	char ErrBuf[1024] = {};
	const int Rc = mj_loadTexture(M, TexId, &Data, &OutWidth, &OutHeight, &OutColorSpace, ErrBuf, sizeof(ErrBuf));
	if (Rc != 0 || !Data)
	{
		const char* Msg = ErrBuf[0] ? ErrBuf : "mj_loadTexture failed";
		OutError = UTF8_TO_TCHAR(Msg);
		return false;
	}

	const int32 Channels = (TexId >= 0 && TexId < M->ntex) ? M->tex_nchannel[TexId] : 3;
	const int32 NBytes = OutWidth * OutHeight * Channels;
	OutPixels.Append(Data, NBytes);
	mju_free(Data);
	return true;
}

bool UMJAssetLibrary::EmbedModelAssets(const mjModel* M, FMJModelHandle& OutEmbedded, FString& OutError)
{
	if (!M)
	{
		OutError = TEXT("Null mjModel");
		return false;
	}
	char ErrBuf[1024] = {};
	mjModel* Embedded = mj_embedAssets(M, ErrBuf, sizeof(ErrBuf));
	if (!Embedded)
	{
		const char* Msg = ErrBuf[0] ? ErrBuf : "mj_embedAssets failed";
		OutError = UTF8_TO_TCHAR(Msg);
		return false;
	}
	OutEmbedded.Reset(Embedded);
	return true;
}
