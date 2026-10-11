#include "Render/RenderSceneResources.h"

#include "Misc/SecureHash.h"

namespace
{
bool IsFinite(const FLinearColor& Value)
{
	return FMath::IsFinite(Value.R) && FMath::IsFinite(Value.G)
		&& FMath::IsFinite(Value.B) && FMath::IsFinite(Value.A);
}

bool IsFinite(const FVector2D& Value)
{
	return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y);
}

bool ValidateBinding(const FRenderSurfaceBinding& Binding, int32 NumMaterials, FString& OutError)
{
	if (Binding.MaterialIndex != INDEX_NONE
		&& (Binding.MaterialIndex < 0 || Binding.MaterialIndex >= NumMaterials))
	{
		OutError = TEXT("surface binding references a missing material");
		return false;
	}
	if (!IsFinite(Binding.Rgba) || !IsFinite(Binding.TexUniformScale))
	{
		OutError = TEXT("surface binding contains a non-finite value");
		return false;
	}
	return true;
}

struct FStableResourceHash
{
	FSHA1 Hash;

	void Byte(uint8 Value) { Hash.Update(&Value, 1); }
	void Bytes(const TArray<uint8>& Values)
	{
		if (Values.Num() > 0) Hash.Update(Values.GetData(), static_cast<uint32>(Values.Num()));
	}
	void U32(uint32 Value)
	{
		uint8 Bytes[4];
		for (int32 Index = 0; Index < 4; ++Index) Bytes[Index] = static_cast<uint8>(Value >> (Index * 8));
		Hash.Update(Bytes, UE_ARRAY_COUNT(Bytes));
	}
	void I32(int32 Value) { U32(static_cast<uint32>(Value)); }
	void U64(uint64 Value)
	{
		uint8 Bytes[8];
		for (int32 Index = 0; Index < 8; ++Index) Bytes[Index] = static_cast<uint8>(Value >> (Index * 8));
		Hash.Update(Bytes, UE_ARRAY_COUNT(Bytes));
	}
	void Float(float Value)
	{
		uint32 Bits;
		FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
		U32(Bits);
	}
	void Double(double Value)
	{
		uint64 Bits;
		FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
		U64(Bits);
	}
	void Color(const FLinearColor& Value)
	{
		Float(Value.R); Float(Value.G); Float(Value.B); Float(Value.A);
	}
	void Vec2(const FVector2D& Value) { Double(Value.X); Double(Value.Y); }
	void Vec3(const FVector& Value) { Double(Value.X); Double(Value.Y); Double(Value.Z); }
	FString Finish()
	{
		Hash.Final();
		uint8 Digest[FSHA1::DigestSize];
		Hash.GetHash(Digest);
		return BytesToHex(Digest, UE_ARRAY_COUNT(Digest));
	}
};
}

bool FRenderSceneResources::IsValid(FString& OutError) const
{
	OutError.Reset();
	if (SceneRevision.IsEmpty())
	{
		OutError = TEXT("scene resource revision is empty");
		return false;
	}

	for (const FRenderTextureResource& Texture : Textures)
	{
		if (Texture.Width <= 0 || Texture.Height <= 0)
		{
			OutError = TEXT("texture dimensions must be positive");
			return false;
		}
		const int64 Pixels = static_cast<int64>(Texture.Width) * static_cast<int64>(Texture.Height);
		if (Pixels > MAX_int32 / 4 || Texture.Rgba8.Num() != Pixels * 4)
		{
			OutError = TEXT("RGBA8 texture dimensions or payload size are invalid");
			return false;
		}
	}

	for (const FRenderMeshResource& Mesh : Meshes)
	{
		if (Mesh.Positions.Num() < 3 || Mesh.Normals.Num() != Mesh.Positions.Num()
			|| Mesh.UVs.Num() != Mesh.Positions.Num() || Mesh.Triangles.Num() < 3
			|| Mesh.Triangles.Num() % 3 != 0)
		{
			OutError = TEXT("mesh arrays or triangle count are invalid");
			return false;
		}
		for (int32 Index : Mesh.Triangles)
		{
			if (!Mesh.Positions.IsValidIndex(Index))
			{
				OutError = TEXT("mesh triangle index is out of range");
				return false;
			}
		}
		for (int32 Index = 0; Index < Mesh.Positions.Num(); ++Index)
		{
			if (Mesh.Positions[Index].ContainsNaN() || Mesh.Normals[Index].ContainsNaN()
				|| !IsFinite(Mesh.UVs[Index]))
			{
				OutError = TEXT("mesh contains a non-finite value");
				return false;
			}
		}
	}

	for (const FRenderMaterialResource& Material : Materials)
	{
		if (!IsFinite(Material.Rgba) || !FMath::IsFinite(Material.Emission)
			|| !FMath::IsFinite(Material.Specular) || !FMath::IsFinite(Material.Shininess)
			|| !FMath::IsFinite(Material.Reflectance) || !FMath::IsFinite(Material.Metallic)
			|| !FMath::IsFinite(Material.Roughness) || !IsFinite(Material.TexRepeat))
		{
			OutError = TEXT("material contains a non-finite value");
			return false;
		}
		for (int32 RoleIndex = 0; RoleIndex < static_cast<int32>(ERenderMaterialRole::Count); ++RoleIndex)
		{
			const FRenderTextureBinding& Binding = Material.Textures[RoleIndex];
			if (Binding.TextureIndex != INDEX_NONE && !Textures.IsValidIndex(Binding.TextureIndex))
			{
				OutError = TEXT("material references a missing texture");
				return false;
			}
			const bool bExpectedSRGB = RoleIndex == static_cast<int32>(ERenderMaterialRole::Rgb)
				|| RoleIndex == static_cast<int32>(ERenderMaterialRole::Emissive)
				|| RoleIndex == static_cast<int32>(ERenderMaterialRole::Rgba);
			const bool bExpectedNormal = RoleIndex == static_cast<int32>(ERenderMaterialRole::Normal);
			if (Binding.TextureIndex != INDEX_NONE
				&& (Binding.bSRGB != bExpectedSRGB || Binding.bNormal != bExpectedNormal))
			{
				OutError = TEXT("material texture sampling does not match its role");
				return false;
			}
		}
	}

	for (const FRenderSurfaceBinding& Geom : Geoms)
	{
		if (!ValidateBinding(Geom, Materials.Num(), OutError)) return false;
	}
	for (const FRenderSurfaceBinding& Deformable : Deformables)
	{
		if (!ValidateBinding(Deformable, Materials.Num(), OutError)) return false;
	}
	return true;
}

FString FRenderSceneResources::ContentHash() const
{
	FStableResourceHash Stable;
	Stable.U32(2);
	Stable.I32(Meshes.Num());
	for (const FRenderMeshResource& Mesh : Meshes)
	{
		Stable.I32(Mesh.Positions.Num());
		for (const FVector& Value : Mesh.Positions) Stable.Vec3(Value);
		Stable.I32(Mesh.Normals.Num());
		for (const FVector& Value : Mesh.Normals) Stable.Vec3(Value);
		Stable.I32(Mesh.UVs.Num());
		for (const FVector2D& Value : Mesh.UVs) Stable.Vec2(Value);
		Stable.I32(Mesh.Triangles.Num());
		for (int32 Value : Mesh.Triangles) Stable.I32(Value);
	}
	Stable.I32(Textures.Num());
	for (const FRenderTextureResource& Texture : Textures)
	{
		Stable.I32(Texture.Width); Stable.I32(Texture.Height); Stable.I32(Texture.Rgba8.Num());
		Stable.Bytes(Texture.Rgba8);
	}
	Stable.I32(Materials.Num());
	for (const FRenderMaterialResource& Material : Materials)
	{
		Stable.Color(Material.Rgba);
		Stable.Float(Material.Emission); Stable.Float(Material.Specular);
		Stable.Float(Material.Shininess); Stable.Float(Material.Reflectance);
		Stable.Float(Material.Metallic); Stable.Float(Material.Roughness);
		Stable.Vec2(Material.TexRepeat); Stable.Byte(Material.bTexUniform ? 1 : 0);
		for (const FRenderTextureBinding& Binding : Material.Textures)
		{
			Stable.I32(Binding.TextureIndex); Stable.Byte(Binding.bSRGB ? 1 : 0);
			Stable.Byte(Binding.bNormal ? 1 : 0);
		}
	}
	auto AddSurfaces = [&Stable](const TArray<FRenderSurfaceBinding>& Surfaces)
	{
		Stable.I32(Surfaces.Num());
		for (const FRenderSurfaceBinding& Surface : Surfaces)
		{
			Stable.I32(Surface.MaterialIndex); Stable.Color(Surface.Rgba);
			Stable.Vec2(Surface.TexUniformScale);
		}
	};
	AddSurfaces(Geoms);
	AddSurfaces(Deformables);
	return Stable.Finish();
}
