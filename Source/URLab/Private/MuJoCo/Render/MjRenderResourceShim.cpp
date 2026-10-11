#include "MuJoCo/Render/MjRenderResourceShim.h"

#include "Utils/CoordinateConversion.h"
#include "Render/RenderMaterial.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
constexpr double CreaseDot = 0.8;
constexpr double InfinitePlaneHalfM = 25.0;
// The classic MuJoCo visualizer emits three distinct vertices for each flat-shaded
// triangle. Bound the expanded resource before allocating its arrays.
constexpr int64 MaxHfieldOutputVertices = 3000000;

bool ValidCount(mjtSize Count)
{
	return Count >= 0 && Count <= MAX_int32;
}

bool Require(bool Condition, const TCHAR* Message, FString& OutError)
{
	if (Condition) return true;
	OutError = Message;
	return false;
}

bool ValidateModelPools(const mjModel& Model, FString& OutError)
{
	if (!Require(ValidCount(Model.nmesh) && ValidCount(Model.nmeshvert)
		&& ValidCount(Model.nmeshnormal) && ValidCount(Model.nmeshtexcoord)
		&& ValidCount(Model.nmeshface) && ValidCount(Model.ntex)
		&& ValidCount(Model.ntexdata) && ValidCount(Model.nmat)
		&& ValidCount(Model.ngeom) && ValidCount(Model.nflex)
		&& ValidCount(Model.nskin) && ValidCount(Model.nhfield)
		&& ValidCount(Model.nhfielddata)
		&& Model.nmesh <= MAX_int32 - Model.nhfield,
		TEXT("MuJoCo render pool count exceeds renderer limits"), OutError)) return false;
	if (!Require(Model.nflex <= MAX_int32 - Model.nskin,
		TEXT("MuJoCo deformable count exceeds renderer limits"), OutError)) return false;

	if (Model.nmesh > 0 && !Require(Model.mesh_vertadr && Model.mesh_vertnum
		&& Model.mesh_normaladr && Model.mesh_texcoordadr && Model.mesh_faceadr
		&& Model.mesh_facenum && Model.mesh_vert && Model.mesh_normal && Model.mesh_face
		&& Model.mesh_facenormal && Model.mesh_pos && Model.mesh_quat,
		TEXT("MuJoCo mesh pool is incomplete"), OutError)) return false;
	if (Model.nhfield > 0 && !Require(Model.hfield_size && Model.hfield_nrow
		&& Model.hfield_ncol && Model.hfield_adr
		&& (Model.nhfielddata == 0 || Model.hfield_data),
		TEXT("MuJoCo heightfield pool is incomplete"), OutError)) return false;
	if (Model.nmeshtexcoord > 0 && !Require(Model.mesh_texcoord && Model.mesh_facetexcoord,
		TEXT("MuJoCo mesh texcoord pool is incomplete"), OutError)) return false;
	if (Model.ntex > 0 && !Require(Model.tex_width && Model.tex_height && Model.tex_nchannel
		&& Model.tex_adr && Model.tex_data,
		TEXT("MuJoCo texture pool is incomplete"), OutError)) return false;
	if (Model.nmat > 0 && !Require(Model.mat_rgba && Model.mat_emission && Model.mat_specular
		&& Model.mat_shininess && Model.mat_reflectance && Model.mat_metallic
		&& Model.mat_roughness && Model.mat_texrepeat && Model.mat_texuniform && Model.mat_texid,
		TEXT("MuJoCo material pool is incomplete"), OutError)) return false;
	if (Model.ngeom > 0 && !Require(Model.geom_matid && Model.geom_dataid && Model.geom_rgba && Model.geom_size
		&& Model.geom_type, TEXT("MuJoCo geom appearance pool is incomplete"), OutError)) return false;
	if (Model.nflex > 0 && !Require(Model.flex_matid && Model.flex_rgba
		&& Model.flex_dim && Model.flex_vertadr && Model.flex_vertnum
		&& Model.flex_elemnum && Model.flex_elemdataadr
		&& (Model.nflexelemdata == 0 || Model.flex_elem)
		&& Model.flex_shellnum && Model.flex_shelldataadr
		&& (Model.nflexshelldata == 0 || Model.flex_shell),
		TEXT("MuJoCo flex appearance pool is incomplete"), OutError)) return false;
	if (Model.nskin > 0 && !Require(Model.skin_matid && Model.skin_rgba
		&& Model.skin_vertadr && Model.skin_vertnum && Model.skin_faceadr
		&& Model.skin_facenum && (Model.nskinface == 0 || Model.skin_face)
		&& (Model.nskinvert == 0 || Model.skin_vert)
		&& Model.skin_boneadr && Model.skin_bonenum
		&& (Model.nskinbone == 0 || (Model.skin_bonebodyid
			&& Model.skin_bonebindpos && Model.skin_bonebindquat
			&& Model.skin_bonevertadr && Model.skin_bonevertnum))
		&& (Model.nskinbonevert == 0 || Model.skin_bonevertid)
		&& (Model.nskinbonevert == 0 || Model.skin_bonevertweight),
		TEXT("MuJoCo skin render pool is incomplete"), OutError)) return false;
	return true;
}

bool ValidateMeshSlice(const mjModel& Model, int32 MeshId, FString& OutError)
{
	const int64 VertAdr = Model.mesh_vertadr[MeshId];
	const int64 VertNum = Model.mesh_vertnum[MeshId];
	const int64 NormalAdr = Model.mesh_normaladr[MeshId];
	const int64 FaceAdr = Model.mesh_faceadr[MeshId];
	const int64 FaceNum = Model.mesh_facenum[MeshId];
	const int64 TexAdr = Model.mesh_texcoordadr[MeshId];
	if (!Require(VertAdr >= 0 && VertNum >= 3 && VertAdr + VertNum <= Model.nmeshvert,
		TEXT("MuJoCo mesh vertex slice is out of bounds"), OutError)) return false;
	if (!Require(NormalAdr >= 0 && NormalAdr < Model.nmeshnormal,
		TEXT("MuJoCo mesh normal slice is out of bounds"), OutError)) return false;
	if (!Require(FaceAdr >= 0 && FaceNum > 0 && FaceNum <= MAX_int32 / 3
		&& FaceAdr + FaceNum <= Model.nmeshface,
		TEXT("MuJoCo mesh face slice is empty or out of bounds"), OutError)) return false;
	if (!Require(TexAdr == -1 || (TexAdr >= 0 && TexAdr < Model.nmeshtexcoord),
		TEXT("MuJoCo mesh texcoord slice is out of bounds"), OutError)) return false;
	const double* SourcePosition = Model.mesh_pos + 3 * MeshId;
	const double* SourceQuaternion = Model.mesh_quat + 4 * MeshId;
	double QuatNormSquared = 0.0;
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		if (!Require(FMath::IsFinite(SourcePosition[Axis]),
			TEXT("MuJoCo mesh source position is non-finite"), OutError)) return false;
	}
	for (int32 Axis = 0; Axis < 4; ++Axis)
	{
		if (!Require(FMath::IsFinite(SourceQuaternion[Axis]),
			TEXT("MuJoCo mesh source quaternion is non-finite"), OutError)) return false;
		QuatNormSquared += SourceQuaternion[Axis] * SourceQuaternion[Axis];
	}
	if (!Require(FMath::IsNearlyEqual(QuatNormSquared, 1.0, 1.e-4),
		TEXT("MuJoCo mesh source quaternion is not unit length"), OutError)) return false;

	for (int64 Face = FaceAdr; Face < FaceAdr + FaceNum; ++Face)
	{
		const int32* FaceVerts = Model.mesh_face + 3 * Face;
		const int32* FaceNormals = Model.mesh_facenormal + 3 * Face;
		const int32* FaceTexcoords = TexAdr >= 0 ? Model.mesh_facetexcoord + 3 * Face : nullptr;
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			if (!Require(FaceVerts[Corner] >= 0 && FaceVerts[Corner] < VertNum,
				TEXT("MuJoCo mesh vertex index is out of bounds"), OutError)) return false;
			if (!Require(FaceNormals[Corner] >= 0
				&& NormalAdr + FaceNormals[Corner] < Model.nmeshnormal,
				TEXT("MuJoCo mesh normal index is out of bounds"), OutError)) return false;
			if (FaceTexcoords && !Require(FaceTexcoords[Corner] >= 0
				&& TexAdr + FaceTexcoords[Corner] < Model.nmeshtexcoord,
				TEXT("MuJoCo mesh texcoord index is out of bounds"), OutError)) return false;
		}
	}
	return true;
}

void BuildMeshArrays(const mjModel& Model, int32 MeshId, FRenderMeshResource& Out)
{
	const int32 FaceAdr = Model.mesh_faceadr[MeshId];
	const int32 FaceNum = Model.mesh_facenum[MeshId];
	const int32 VertAdr = Model.mesh_vertadr[MeshId];
	const int32 VertNum = Model.mesh_vertnum[MeshId];
	const int32 NormalAdr = Model.mesh_normaladr[MeshId];
	const int32 TexAdr = Model.mesh_texcoordadr[MeshId];
	const bool bHasUV = TexAdr >= 0;

	TArray<FVector> FaceGeoNormals;
	FaceGeoNormals.SetNumUninitialized(FaceNum);
	int32 OrientVote = 0;
	for (int32 Face = 0; Face < FaceNum; ++Face)
	{
		const int32* FaceVerts = Model.mesh_face + 3 * (FaceAdr + Face);
		const FVector P0 = URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(Model.mesh_vert + 3 * (FaceVerts[0] + VertAdr));
		const FVector P1 = URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(Model.mesh_vert + 3 * (FaceVerts[1] + VertAdr));
		const FVector P2 = URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(Model.mesh_vert + 3 * (FaceVerts[2] + VertAdr));
		FVector Normal = FVector::CrossProduct(P1 - P0, P2 - P0).GetSafeNormal();
		const int32 NormalIndex = Model.mesh_facenormal[3 * (FaceAdr + Face)] + NormalAdr;
		const double MjNormal[3] = {Model.mesh_normal[3 * NormalIndex],
			Model.mesh_normal[3 * NormalIndex + 1], Model.mesh_normal[3 * NormalIndex + 2]};
		const FVector Reference = URLabCoordinateConversion::RightHandedDirectionToUnreal(MjNormal).GetSafeNormal();
		if (Normal.IsNearlyZero()) Normal = Reference;
		else OrientVote += FVector::DotProduct(Normal, Reference) >= 0.0 ? 1 : -1;
		FaceGeoNormals[Face] = Normal;
	}
	if (OrientVote < 0)
	{
		for (FVector& Normal : FaceGeoNormals) Normal = -Normal;
	}

	TArray<TArray<int32, TInlineAllocator<8>>> IncidentFaces;
	IncidentFaces.SetNum(VertNum);
	for (int32 Face = 0; Face < FaceNum; ++Face)
	{
		const int32* FaceVerts = Model.mesh_face + 3 * (FaceAdr + Face);
		for (int32 Corner = 0; Corner < 3; ++Corner) IncidentFaces[FaceVerts[Corner]].Add(Face);
	}

	TArray<FVector> CornerNormals;
	CornerNormals.Init(FVector::ZeroVector, FaceNum * 3);
	for (int32 Vertex = 0; Vertex < VertNum; ++Vertex)
	{
		const TArray<int32, TInlineAllocator<8>>& Faces = IncidentFaces[Vertex];
		TArray<FVector, TInlineAllocator<8>> GroupSums;
		TArray<int32, TInlineAllocator<16>> FaceGroups;
		FaceGroups.SetNumUninitialized(Faces.Num());
		for (int32 Index = 0; Index < Faces.Num(); ++Index)
		{
			const FVector FaceNormal = FaceGeoNormals[Faces[Index]];
			int32 Group = INDEX_NONE;
			for (int32 Candidate = 0; Candidate < GroupSums.Num(); ++Candidate)
			{
				if (FVector::DotProduct(FaceNormal, GroupSums[Candidate].GetSafeNormal()) >= CreaseDot)
				{
					GroupSums[Candidate] += FaceNormal;
					Group = Candidate;
					break;
				}
			}
			FaceGroups[Index] = Group != INDEX_NONE ? Group : GroupSums.Add(FaceNormal);
		}
		for (int32 Index = 0; Index < Faces.Num(); ++Index)
		{
			const int32 Face = Faces[Index];
			const int32* FaceVerts = Model.mesh_face + 3 * (FaceAdr + Face);
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				if (FaceVerts[Corner] == Vertex)
					CornerNormals[3 * Face + Corner] = GroupSums[FaceGroups[Index]].GetSafeNormal();
			}
		}
	}

	Out.Positions.Reserve(FaceNum * 3);
	Out.Normals.Reserve(FaceNum * 3);
	Out.UVs.Reserve(FaceNum * 3);
	Out.Triangles.Reserve(FaceNum * 3);
	for (int32 Face = 0; Face < FaceNum; ++Face)
	{
		const int32* FaceVerts = Model.mesh_face + 3 * (FaceAdr + Face);
		const int32* FaceTexcoords = bHasUV ? Model.mesh_facetexcoord + 3 * (FaceAdr + Face) : nullptr;
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			Out.Positions.Add(URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(
				Model.mesh_vert + 3 * (FaceVerts[Corner] + VertAdr)));
			Out.Normals.Add(CornerNormals[3 * Face + Corner].GetSafeNormal());
			if (FaceTexcoords)
			{
				const int32 Index = FaceTexcoords[Corner] + TexAdr;
				Out.UVs.Add(FVector2D(Model.mesh_texcoord[2 * Index], Model.mesh_texcoord[2 * Index + 1]));
			}
			else Out.UVs.Add(FVector2D::ZeroVector);
			Out.Triangles.Add(Out.Positions.Num() - 1);
		}
	}

}

bool BuildHeightfieldArrays(const mjModel& Model, int32 HfieldId,
	FRenderMeshResource& Out, FString& OutError)
{
	const int64 Rows = Model.hfield_nrow[HfieldId];
	const int64 Cols = Model.hfield_ncol[HfieldId];
	const int64 Address = Model.hfield_adr[HfieldId];
	const int64 BaseQuads = Model.vis.quality.numquads;
	if (Rows < 2 || Cols < 2 || Rows > MAX_int32 / Cols)
	{
		const char* RawName = Model.names ? mj_id2name(&Model, mjOBJ_HFIELD, HfieldId) : nullptr;
		const FString Name = RawName ? UTF8_TO_TCHAR(RawName) : FString::Printf(TEXT("id %d"), HfieldId);
		OutError = FString::Printf(
			TEXT("MuJoCo heightfield '%s' grid is %lldx%lld; renderer requires at least 2 rows and 2 columns"),
			*Name, static_cast<long long>(Rows), static_cast<long long>(Cols));
		return false;
	}
	const int64 Samples = Rows * Cols;
	if (!Require(Address >= 0 && Samples <= Model.nhfielddata
		&& Address <= Model.nhfielddata - Samples,
		TEXT("MuJoCo heightfield sample slice is out of bounds"), OutError)) return false;
	if (!Require(BaseQuads >= 1 && BaseQuads <= 4096,
		TEXT("MuJoCo heightfield bottom subdivision is invalid"), OutError)) return false;
	const int64 TopTris = 2 * (Rows - 1) * (Cols - 1);
	const int64 SideTris = 4 * ((Rows - 1) + (Cols - 1));
	const int64 BottomTris = 2 * BaseQuads * BaseQuads;
	const int64 VertexCount = 3 * (TopTris + SideTris + BottomTris);
	if (!Require(VertexCount <= MaxHfieldOutputVertices && VertexCount <= MAX_int32,
		TEXT("MuJoCo heightfield visual mesh exceeds renderer vertex budget"), OutError)) return false;
	const mjtNum* Size = Model.hfield_size + 4 * HfieldId;
	for (int32 Axis = 0; Axis < 4; ++Axis)
	{
		if (!Require(FMath::IsFinite(Size[Axis]) && Size[Axis] >= 0.0
			&& (Axis >= 2 || Size[Axis] > 0.0),
			TEXT("MuJoCo heightfield size is invalid"), OutError)) return false;
	}
	const float* Data = Model.hfield_data + Address;
	for (int64 Index = 0; Index < Samples; ++Index)
	{
		if (!Require(FMath::IsFinite(Data[Index]) && Data[Index] >= 0.0f
			&& Data[Index] <= 1.0f,
			TEXT("MuJoCo heightfield sample is invalid"), OutError)) return false;
	}
	Out.Positions.Reserve(static_cast<int32>(VertexCount));
	Out.Normals.Reserve(static_cast<int32>(VertexCount));
	Out.UVs.Reserve(static_cast<int32>(VertexCount));
	Out.Triangles.Reserve(static_cast<int32>(VertexCount));
	// MuJoCo's classic mjr_uploadHField uses the compiled sample order directly.
	// XML elevation rows have already been reordered by the MuJoCo compiler.
	auto Grid = [&](int64 Row, int64 Col) -> FVector
	{
		return FVector(Size[0] * (2.0 * Col / (Cols - 1) - 1.0),
			Size[1] * (2.0 * Row / (Rows - 1) - 1.0),
			Size[2] * Data[Row * Cols + Col]);
	};
	auto AddTriangle = [&](const FVector& A, const FVector& B, const FVector& C,
		const FVector& FallbackNormal)
	{
		FVector SourceNormal = FVector::CrossProduct(B - A, C - A).GetSafeNormal();
		if (SourceNormal.IsNearlyZero()) SourceNormal = FallbackNormal;
		for (const FVector& Source : {A, B, C})
		{
			const double Point[3] = {Source.X, Source.Y, Source.Z};
			const double Normal[3] = {SourceNormal.X, SourceNormal.Y, SourceNormal.Z};
			Out.Positions.Add(URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(Point));
			Out.Normals.Add(URLabCoordinateConversion::RightHandedDirectionToUnreal(Normal));
			// Classic regular-texture texgen projects local X/Y. Store an equivalent
			// normalized planar basis; the neutral material applies repeat separately.
			Out.UVs.Add(FVector2D(0.5 * (Source.X / Size[0] + 1.0),
				0.5 * (1.0 - Source.Y / Size[1])));
			Out.Triangles.Add(Out.Positions.Num() - 1);
		}
	};
	auto AddQuad = [&](const FVector& A, const FVector& B, const FVector& C,
		const FVector& D, const FVector& FallbackNormal)
	{
		AddTriangle(A, B, C, FallbackNormal);
		AddTriangle(A, C, D, FallbackNormal);
	};
	for (int64 Row = 0; Row < Rows - 1; ++Row)
	{
		// Alternating strip corners from classic render_context.c:addVert.
		for (int64 Col = 0; Col < Cols - 1; ++Col)
		{
			const FVector A = Grid(Row + 1, Col);
			const FVector B = Grid(Row, Col);
			const FVector C = Grid(Row + 1, Col + 1);
			const FVector D = Grid(Row, Col + 1);
			AddTriangle(A, B, C, FVector::UpVector);
			AddTriangle(B, D, C, FVector::UpVector);
		}
	}
	const double Bottom = -Size[3];
	for (int64 Row = 0; Row < Rows - 1; ++Row)
	{
		const FVector Left0 = Grid(Row, 0), Left1 = Grid(Row + 1, 0);
		AddQuad(FVector(Left1.X, Left1.Y, Bottom),
			FVector(Left0.X, Left0.Y, Bottom), Left0, Left1,
			-FVector::ForwardVector);
		const FVector Right0 = Grid(Row, Cols - 1), Right1 = Grid(Row + 1, Cols - 1);
		AddQuad(FVector(Right0.X, Right0.Y, Bottom),
			FVector(Right1.X, Right1.Y, Bottom), Right1, Right0,
			FVector::ForwardVector);
	}
	for (int64 Col = 0; Col < Cols - 1; ++Col)
	{
		const FVector Front0 = Grid(0, Col), Front1 = Grid(0, Col + 1);
		AddQuad(FVector(Front0.X, Front0.Y, Bottom),
			FVector(Front1.X, Front1.Y, Bottom), Front1, Front0,
			-FVector::RightVector);
		const FVector Back0 = Grid(Rows - 1, Col), Back1 = Grid(Rows - 1, Col + 1);
		AddQuad(FVector(Back1.X, Back1.Y, Bottom),
			FVector(Back0.X, Back0.Y, Bottom), Back0, Back1,
			FVector::RightVector);
	}
	for (int64 Row = 0; Row < BaseQuads; ++Row)
	{
		for (int64 Col = 0; Col < BaseQuads; ++Col)
		{
			const double X0 = Size[0] * (2.0 * Col / BaseQuads - 1.0);
			const double X1 = Size[0] * (2.0 * (Col + 1) / BaseQuads - 1.0);
			const double Y0 = Size[1] * (2.0 * Row / BaseQuads - 1.0);
			const double Y1 = Size[1] * (2.0 * (Row + 1) / BaseQuads - 1.0);
			AddQuad(FVector(X0, Y0, Bottom), FVector(X0, Y1, Bottom),
				FVector(X1, Y1, Bottom), FVector(X1, Y0, Bottom),
				-FVector::UpVector);
		}
	}
	return true;
}

FRenderSurfaceBinding SurfaceBinding(int32 MaterialIndex, const float* Rgba,
	const FVector2D& UniformExtent)
{
	FRenderSurfaceBinding Out;
	Out.MaterialIndex = MaterialIndex;
	Out.Rgba = FLinearColor(Rgba[0], Rgba[1], Rgba[2], Rgba[3]);
	Out.TexUniformScale = UniformExtent;
	return Out;
}
}

bool MjRenderResourceShim::Fill(FRenderSceneResources& Out, const mjModel_* ModelPtr,
	const FString& SceneRevision, FString& OutError)
{
	OutError.Reset();
	const mjModel* Model = ModelPtr;
	if (!Require(Model != nullptr, TEXT("null MuJoCo model"), OutError)
		|| !ValidateModelPools(*Model, OutError)) return false;

	FRenderSceneResources Candidate;
	Candidate.SceneRevision = SceneRevision;
	Candidate.Meshes.SetNum(static_cast<int32>(Model->nmesh + Model->nhfield));
	for (int32 MeshId = 0; MeshId < Model->nmesh; ++MeshId)
	{
		if (!ValidateMeshSlice(*Model, MeshId, OutError)) return false;
		BuildMeshArrays(*Model, MeshId, Candidate.Meshes[MeshId]);
	}
	for (int32 HfieldId = 0; HfieldId < Model->nhfield; ++HfieldId)
	{
		if (!BuildHeightfieldArrays(*Model, HfieldId,
			Candidate.Meshes[Model->nmesh + HfieldId], OutError)) return false;
	}

	Candidate.Textures.SetNum(static_cast<int32>(Model->ntex));
	for (int32 TextureId = 0; TextureId < Candidate.Textures.Num(); ++TextureId)
	{
		FRenderTextureResource& Texture = Candidate.Textures[TextureId];
		Texture.Width = Model->tex_width[TextureId];
		Texture.Height = Model->tex_height[TextureId];
		const int32 Channels = Model->tex_nchannel[TextureId];
		if (!Require(Texture.Width > 0 && Texture.Height > 0 && Channels >= 1 && Channels <= 4,
			TEXT("MuJoCo texture dimensions or channel count are invalid"), OutError)) return false;
		const int64 Pixels = static_cast<int64>(Texture.Width) * Texture.Height;
		if (!Require(Pixels <= MAX_int32 / 4,
			TEXT("MuJoCo texture RGBA8 payload exceeds renderer limits"), OutError)) return false;
		const int64 SourceBytes = Pixels * Channels;
		const int64 SourceAddress = Model->tex_adr[TextureId];
		if (!Require(SourceAddress >= 0 && SourceBytes <= Model->ntexdata
			&& SourceAddress <= Model->ntexdata - SourceBytes,
			TEXT("MuJoCo texture source is out of bounds"), OutError)) return false;

		Texture.Rgba8.SetNumUninitialized(static_cast<int32>(Pixels * 4));
		const uint8* Source = Model->tex_data + SourceAddress;
		for (int32 Pixel = 0; Pixel < static_cast<int32>(Pixels); ++Pixel)
		{
			const uint8* Src = Source + Pixel * Channels;
			uint8* Dst = Texture.Rgba8.GetData() + Pixel * 4;
			if (Channels >= 3)
			{
				Dst[0] = Src[0]; Dst[1] = Src[1]; Dst[2] = Src[2];
				Dst[3] = Channels >= 4 ? Src[3] : 255;
			}
			else
			{
				Dst[0] = Dst[1] = Dst[2] = Src[0]; Dst[3] = 255;
			}
		}
	}

	Candidate.Materials.SetNum(static_cast<int32>(Model->nmat));
	for (int32 MaterialId = 0; MaterialId < Candidate.Materials.Num(); ++MaterialId)
	{
		FRenderMaterialResource& Material = Candidate.Materials[MaterialId];
		const float* Rgba = Model->mat_rgba + 4 * MaterialId;
		Material.Rgba = FLinearColor(Rgba[0], Rgba[1], Rgba[2], Rgba[3]);
		Material.Emission = Model->mat_emission[MaterialId];
		Material.Specular = Model->mat_specular[MaterialId];
		Material.Shininess = Model->mat_shininess[MaterialId];
		Material.Reflectance = Model->mat_reflectance[MaterialId];
		Material.TexRepeat = FVector2D(Model->mat_texrepeat[2 * MaterialId],
			Model->mat_texrepeat[2 * MaterialId + 1]);
		Material.bTexUniform = Model->mat_texuniform[MaterialId] != 0;
		for (int32 RoleIndex = 0; RoleIndex < static_cast<int32>(ERenderMaterialRole::Count); ++RoleIndex)
		{
			const int32 TextureId = Model->mat_texid[MaterialId * mjNTEXROLE + RoleIndex + 1];
			if (!Require(TextureId == INDEX_NONE || Candidate.Textures.IsValidIndex(TextureId),
				TEXT("MuJoCo material texture id is out of bounds"), OutError)) return false;
			if (TextureId != INDEX_NONE)
			{
				FRenderTextureBinding& Binding = Material.Textures[RoleIndex];
				Binding.TextureIndex = TextureId;
				RenderMaterialRoleSampling(static_cast<ERenderMaterialRole>(RoleIndex),
					Binding.bSRGB, Binding.bNormal);
			}
		}
		const bool bHasOrm = Material.Textures[static_cast<int32>(ERenderMaterialRole::Orm)].TextureIndex != INDEX_NONE;
		const bool bHasRoughness = bHasOrm || Material.Textures[static_cast<int32>(ERenderMaterialRole::Roughness)].TextureIndex != INDEX_NONE;
		const bool bHasMetallic = bHasOrm || Material.Textures[static_cast<int32>(ERenderMaterialRole::Metallic)].TextureIndex != INDEX_NONE;
		const bool bHasEmissive = Material.Textures[static_cast<int32>(ERenderMaterialRole::Emissive)].TextureIndex != INDEX_NONE;
		Material.Roughness = Model->mat_roughness[MaterialId] >= 0.0f
			? FMath::Clamp(Model->mat_roughness[MaterialId], 0.0f, 1.0f)
			: (bHasRoughness ? 1.0f : FMath::Clamp(1.0f - Material.Shininess, 0.0f, 1.0f));
		Material.Metallic = Model->mat_metallic[MaterialId] >= 0.0f
			? FMath::Clamp(Model->mat_metallic[MaterialId], 0.0f, 1.0f)
			: (bHasMetallic ? 1.0f : 0.0f);
		Material.Emission = bHasEmissive && Material.Emission <= 0.0f
			? 1.0f : FMath::Max(Material.Emission, 0.0f);
	}

	Candidate.Geoms.SetNum(static_cast<int32>(Model->ngeom));
	for (int32 GeomId = 0; GeomId < Candidate.Geoms.Num(); ++GeomId)
	{
		const int32 MaterialId = Model->geom_matid[GeomId];
		if (!Require(MaterialId == INDEX_NONE || Candidate.Materials.IsValidIndex(MaterialId),
			TEXT("MuJoCo geom material id is out of bounds"), OutError)) return false;
		const float* Rgba = MaterialId == INDEX_NONE
			? Model->geom_rgba + 4 * GeomId : Model->mat_rgba + 4 * MaterialId;
		const bool bPlane = Model->geom_type[GeomId] == mjGEOM_PLANE;
		const double SizeX = Model->geom_size[3 * GeomId] > 0.0 ? Model->geom_size[3 * GeomId]
			: (bPlane ? InfinitePlaneHalfM : 0.0);
		const double SizeY = Model->geom_size[3 * GeomId + 1] > 0.0 ? Model->geom_size[3 * GeomId + 1]
			: (bPlane ? InfinitePlaneHalfM : 0.0);
		Candidate.Geoms[GeomId] = SurfaceBinding(MaterialId, Rgba, FVector2D(SizeX, SizeY));
		if (Model->geom_type[GeomId] == mjGEOM_MESH)
		{
			const int32 MeshId = Model->geom_dataid[GeomId];
			if (!Require(MeshId >= 0 && MeshId < Model->nmesh,
				TEXT("MuJoCo mesh geom references a missing mesh"), OutError)) return false;
		}
		if (Model->geom_type[GeomId] == mjGEOM_HFIELD)
		{
			const int32 HfieldId = Model->geom_dataid[GeomId];
			if (!Require(HfieldId >= 0 && HfieldId < Model->nhfield,
				TEXT("MuJoCo heightfield geom references a missing heightfield"), OutError)) return false;
		}
	}

	Candidate.Deformables.SetNum(static_cast<int32>(Model->nflex + Model->nskin));
	for (int32 FlexId = 0; FlexId < Model->nflex; ++FlexId)
	{
		const int32 MaterialId = Model->flex_matid[FlexId];
		if (!Require(MaterialId == INDEX_NONE || Candidate.Materials.IsValidIndex(MaterialId),
			TEXT("MuJoCo flex material id is out of bounds"), OutError)) return false;
		const float* Rgba = MaterialId == INDEX_NONE
			? Model->flex_rgba + 4 * FlexId : Model->mat_rgba + 4 * MaterialId;
		Candidate.Deformables[FlexId] = SurfaceBinding(MaterialId, Rgba, FVector2D::ZeroVector);
	}
	for (int32 SkinId = 0; SkinId < Model->nskin; ++SkinId)
	{
		const int32 MaterialId = Model->skin_matid[SkinId];
		if (!Require(MaterialId == INDEX_NONE || Candidate.Materials.IsValidIndex(MaterialId),
			TEXT("MuJoCo skin material id is out of bounds"), OutError)) return false;
		const float* Rgba = MaterialId == INDEX_NONE
			? Model->skin_rgba + 4 * SkinId : Model->mat_rgba + 4 * MaterialId;
		Candidate.Deformables[Model->nflex + SkinId] =
			SurfaceBinding(MaterialId, Rgba, FVector2D::ZeroVector);
	}

	if (!Candidate.IsValid(OutError)) return false;
	Out = MoveTemp(Candidate);
	return true;
}
