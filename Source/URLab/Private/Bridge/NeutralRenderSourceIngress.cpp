// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Bridge/NeutralRenderSourceIngress.h"

#include "Misc/ByteSwap.h"
#include "Containers/ContainerAllocationPolicies.h"
#include "Utils/AssetCache.h"
#include "Utils/CoordinateConversion.h"

namespace
{
using V = FURLabWireValue;

bool Fail(FString& Error, const FString& Message) { Error = Message; return false; }
const V* Field(const V& Map, const TCHAR* Name) { return Map.FindMapValue(Name); }

bool CheckedMul(int64 A, int64 B, int64& Out)
{
	if (A < 0 || B < 0 || (A && B > MAX_int64 / A)) return false;
	Out = A * B; return true;
}

bool CheckedAdd(int64 Value,int64& Total)
{
	if(Value<0||Total>MAX_int64-Value)return false;Total+=Value;return true;
}

template<typename T> bool ArrayBytes(int64 Count,int64& Total)
{
	if(Count<0||Count>MAX_int32)return false;
	const int32 Capacity=DefaultCalculateSlackReserve<int32>(static_cast<int32>(Count),sizeof(T),true,alignof(T));
	int64 Bytes=0;return Capacity>=Count&&CheckedMul(Capacity,sizeof(T),Bytes)&&CheckedAdd(Bytes,Total);
}

bool NameValid(const FString& Name, bool bAllowEmpty)
{
	FTCHARToUTF8 Utf8(*Name);
	return (bAllowEmpty || !Name.IsEmpty()) && Utf8.Length() <= 1024;
}

bool FloatArray(const V* Value, int32 Count, double* Out, FString& Error)
{
	if (!Value || Value->GetArray().Num() != Count)
		return Fail(Error, TEXT("neutral numeric array has the wrong length"));
	for (int32 I = 0; I < Count; ++I)
	{
		Out[I] = Value->GetArray()[I].GetFloat();
		if (!FMath::IsFinite(Out[I])) return Fail(Error, TEXT("neutral numeric value is non-finite"));
	}
	return true;
}

bool Pose(const V& Value, bool bOptical, FTransform& Out, FString& Error)
{
	double P[3], Q[4];
	if (!FloatArray(Field(Value, TEXT("position_m")), 3, P, Error)
		|| !FloatArray(Field(Value, TEXT("rotation_wxyz")), 4, Q, Error)) return false;
	const double N = FMath::Sqrt(Q[0]*Q[0]+Q[1]*Q[1]+Q[2]*Q[2]+Q[3]*Q[3]);
	if (!FMath::IsFinite(N) || FMath::Abs(N - 1.0) > 1.e-6)
		return Fail(Error, TEXT("neutral quaternion is not near unit length"));
	for (double& X : Q) X /= N;
	Out = bOptical ? URLabCoordinateConversion::RightHandedOpticalPoseToUnreal(P, Q)
		: FTransform(URLabCoordinateConversion::RightHandedWxyzQuaternionToUnreal(Q),
			URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(P));
	const FVector Translation = Out.GetTranslation();
	return Out.GetRotation().IsNormalized()
		&& FMath::IsFinite(Translation.X) && FMath::IsFinite(Translation.Y)
		&& FMath::IsFinite(Translation.Z)
		? true : Fail(Error, TEXT("neutral pose is not representable"));
}

struct FPinRecord { FString BlobId, Sha; FURLabPayloadPin Pin; int64 Length = 0; };
struct FView { int32 PinIndex = INDEX_NONE; int64 Offset = 0, Length = 0; };

class FInputs
{
public:
	FInputs(FURLabPayloadStore& InStore, const FURLabPayloadOwner& InOwner)
		: Store(InStore), Owner(InOwner) {}

	bool View(const V& Wire, int64 Expected, FView& Out, FString& Error)
	{
		const V* Payload = Field(Wire, TEXT("payload"));
		const V* Offset = Field(Wire, TEXT("byte_offset"));
		const V* Length = Field(Wire, TEXT("byte_length"));
		if (!Payload || !Offset || !Length) return Fail(Error, TEXT("neutral buffer view is incomplete"));
		const V* Blob = Field(*Payload, TEXT("blob_id"));
		const V* Sha = Field(*Payload, TEXT("sha256"));
		if (!Blob || !Sha || Blob->GetString().IsEmpty()
			|| !FURLabAssetCache::IsValidSha256Hex(Sha->GetString()))
			return Fail(Error, TEXT("neutral buffer payload identity is invalid"));
		Out.Offset = Offset->GetInt(); Out.Length = Length->GetInt();
		if (Out.Offset < 0 || Out.Length <= 0 || Out.Length != Expected)
			return Fail(Error, TEXT("neutral buffer view length is inexact"));
		Out.PinIndex = INDEX_NONE;
		for (int32 I=0; I<Pins.Num(); ++I) if (Pins[I].BlobId == Blob->GetString()) { Out.PinIndex=I; break; }
		if (Out.PinIndex == INDEX_NONE)
		{
			if (Pins.Num() >= 63) return Fail(Error, TEXT("neutral source has too many distinct payloads"));
			FPinRecord R; R.BlobId=Blob->GetString(); R.Sha=Sha->GetString();
			FURLabPayloadStoreStatus S=Store.Pin(Owner,R.BlobId,R.Pin);
			if (!S) return Fail(Error,S.Message);
			R.Length=R.Pin->Num();
			FURLabStoredPayloadDescriptor Descriptor;
			S=Store.GetDescriptor(Owner,R.BlobId,Descriptor);
			if(!S)return Fail(Error,S.Message);
			if(R.Pin->GetSha256()!=R.Sha||Descriptor.BlobId!=R.BlobId
				||Descriptor.Sha256!=R.Sha||Descriptor.ByteLength!=R.Length
				||Descriptor.BootId!=Owner.BootId||Descriptor.SessionId!=Owner.SessionId)
				return Fail(Error,TEXT("neutral payload identity, owner, hash, or length mismatch"));
			Out.PinIndex=Pins.Add(MoveTemp(R));
		}
		else if (Pins[Out.PinIndex].Sha != Sha->GetString())
			return Fail(Error, TEXT("one neutral blob id names conflicting hashes"));
		const int64 N=Pins[Out.PinIndex].Length;
		if (Out.Offset > N || Out.Length > N-Out.Offset) return Fail(Error,TEXT("neutral buffer view is out of bounds"));
		return true;
	}

	bool Read(const FView& View, int64 Relative, void* Dest, int64 Count, FString& Error)
	{
		if (!Pins.IsValidIndex(View.PinIndex) || Relative < 0 || Count < 0
			|| Relative > View.Length || Count > View.Length-Relative)
			return Fail(Error,TEXT("neutral buffer read is out of bounds"));
		const FPinRecord& R=Pins[View.PinIndex]; TConstArrayView<uint8> Network;
		if (R.Pin->TryGetNetworkBytes(Network))
		{
			FMemory::Memcpy(Dest,Network.GetData()+View.Offset+Relative,Count); return true;
		}
		int64 Done=0;
		while(Done<Count)
		{
			const int64 Part=FMath::Min<int64>(Store.GetLimits().MaxChunkBytes,Count-Done);
			FURLabPayloadCopyPin Copy; const auto S=Store.CopyPinnedRange(R.Pin,View.Offset+Relative+Done,Part,Copy);
			if(!S) return Fail(Error,S.Message);
			if(!Copy || Copy->Num()!=Part) return Fail(Error,TEXT("neutral SHM range length changed"));
			FMemory::Memcpy(static_cast<uint8*>(Dest)+Done,Copy->GetBytes().GetData(),Part); Done+=Part;
		}
		return true;
	}

private:
	FURLabPayloadStore& Store; FURLabPayloadOwner Owner; TArray<FPinRecord> Pins;
};

bool ReadDouble(FInputs& Inputs,const FView& View,int32 Index,double& Out,FString& Error)
{
	uint64 Bits=0; if(!Inputs.Read(View,static_cast<int64>(Index)*8,&Bits,8,Error)) return false;
	Bits=INTEL_ORDER64(Bits); FMemory::Memcpy(&Out,&Bits,8); return FMath::IsFinite(Out);
}

bool ReadFloat(FInputs& Inputs,const FView& View,int64 Index,float& Out,FString& Error)
{
	uint32 Bits=0;if(!Inputs.Read(View,Index*4,&Bits,4,Error))return false;
	Bits=INTEL_ORDER32(Bits);FMemory::Memcpy(&Out,&Bits,4);
	return FMath::IsFinite(Out)?true:Fail(Error,TEXT("neutral float buffer contains non-finite data"));
}

bool ReadUInt(FInputs& Inputs,const FView& View,int64 Index,uint32& Out,FString& Error)
{
	if(!Inputs.Read(View,Index*4,&Out,4,Error))return false;Out=INTEL_ORDER32(Out);return true;
}

class FNeutralLease final : public IRenderSceneResourceLifetimeLease
{
public:
	explicit FNeutralLease(FURLabPayloadPhysicalReservationPtr&& InReservation)
		: Reservation(MoveTemp(InReservation)){}
private:
	FURLabPayloadPhysicalReservationPtr Reservation;
};

class FDynamicFrameLease final : public IRenderCameraCaptureLifetimeLease
{
public:
	explicit FDynamicFrameLease(FURLabPayloadPhysicalReservationPtr&& InReservation)
		: Reservation(MoveTemp(InReservation)) {}
private:
	FURLabPayloadPhysicalReservationPtr Reservation;
};

bool FrameFromView(FInputs& Inputs,const V& Wire,int32 Bodies,int32 Cameras,bool bUser,int32 Lights,
	const FString& Revision,FRenderFrame& Out,FString& Error)
{
	Out.Identity.SceneRevision=Revision;
	Out.Identity.FrameId=static_cast<uint64>(Field(Wire,TEXT("frame_id"))->GetInt());
	Out.Identity.ClockEpoch=static_cast<uint64>(Field(Wire,TEXT("clock_epoch"))->GetInt());
	Out.Identity.bHasSourceTime=Field(Wire,TEXT("has_source_time"))->GetBool();
	const V* Time=Field(Wire,TEXT("source_time"));
	if (Out.Identity.bHasSourceTime != (Time!=nullptr)) return Fail(Error,TEXT("neutral source-time presence is inconsistent"));
	if(Time) Out.Identity.SourceTime=Time->GetFloat();
	int64 Records64=0;if(!CheckedAdd(Bodies,Records64)||!CheckedAdd(Cameras,Records64)
		||(bUser&&!CheckedAdd(1,Records64))||!CheckedAdd(Lights,Records64)||Records64>MAX_int32)
		return Fail(Error,TEXT("neutral pose count overflows"));
	const int32 Records=static_cast<int32>(Records64); const V* Poses=Field(Wire,TEXT("poses"));
	if(Records==0) return Poses?Fail(Error,TEXT("empty neutral frame must omit poses")):true;
	int64 Bytes=0; if(!Poses || !CheckedMul(Records*7,8,Bytes)) return Fail(Error,TEXT("neutral pose byte count overflows"));
	FView View; if(!Inputs.View(*Poses,Bytes,View,Error)) return false;
	int32 Scalar=0;
	auto One=[&](bool Optical,FTransform& Transform)
	{
		double X[7]; for(double& V:X) if(!ReadDouble(Inputs,View,Scalar++,V,Error)) return false;
		const double N=FMath::Sqrt(X[3]*X[3]+X[4]*X[4]+X[5]*X[5]+X[6]*X[6]);
		if(!FMath::IsFinite(N)||FMath::Abs(N-1.0)>1.e-6) return Fail(Error,TEXT("neutral initial quaternion is not near unit"));
		for(int32 I=3;I<7;++I)X[I]/=N;
		Transform=Optical?URLabCoordinateConversion::RightHandedOpticalPoseToUnreal(X,X+3)
			:FTransform(URLabCoordinateConversion::RightHandedWxyzQuaternionToUnreal(X+3),URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(X));
		return true;
	};
	Out.BodySceneTransforms.Reserve(Bodies); Out.CameraSceneTransforms.Reserve(Cameras);
	Out.LightSceneTransforms.Reserve(Lights);
	for(int32 I=0;I<Bodies;++I){FTransform T;if(!One(false,T))return false;Out.BodySceneTransforms.Add(T);}
	for(int32 I=0;I<Cameras;++I){FTransform T;if(!One(true,T))return false;Out.CameraSceneTransforms.Add(T);}
	if(bUser){FTransform T;if(!One(true,T))return false;Out.UserCameraSceneTransform=T;}
	for(int32 I=0;I<Lights;++I){FTransform T;if(!One(false,T))return false;Out.LightSceneTransforms.Add(T);}
	return true;
}

bool FrameVisualization(FInputs& Inputs,const V* Wire,const TArray<FRenderDeformableDesc>& Deformables,FRenderFrame& Frame,FString& Error)
{
	if(!Wire)return Deformables.IsEmpty()?true:Fail(Error,TEXT("frame is missing deformable data"));
	const V* Values=Field(*Wire,TEXT("deformables"));
	if(Deformables.IsEmpty()&&Values&&Values->GetArray().Num())return Fail(Error,TEXT("frame contains uninstalled deformables"));
	if(Deformables.IsEmpty())Values=nullptr;
	if(Deformables.Num()&&(!Values||Values->GetArray().Num()!=Deformables.Num()))return Fail(Error,TEXT("frame must contain every installed deformable"));
	for(int32 I=0;I<Deformables.Num();++I)
	{
		const FRenderDeformableDesc& Desc=Deformables[I];const V& Row=Values->GetArray()[I];
		if(Field(Row,TEXT("name"))->GetString()!=Desc.Name)return Fail(Error,TEXT("initial deformable order/name differs from scene"));
		int64 FloatCount=0,ByteCount=0;if(!CheckedMul(Desc.VertexCount,3,FloatCount)||!CheckedMul(FloatCount,4,ByteCount))return Fail(Error,TEXT("deformable position length overflows"));
		FView PositionView;if(!Inputs.View(*Field(Row,TEXT("vertex_positions_cm")),ByteCount,PositionView,Error))return false;
		FRenderFrame::FDeformable& Dynamic=Frame.Deformables.AddDefaulted_GetRef();Dynamic.Name=Desc.Name;Dynamic.Positions.Reserve(Desc.VertexCount);
		for(int64 Vx=0;Vx<Desc.VertexCount;++Vx){float X,Y,Z;if(!ReadFloat(Inputs,PositionView,Vx*3,X,Error)||!ReadFloat(Inputs,PositionView,Vx*3+1,Y,Error)||!ReadFloat(Inputs,PositionView,Vx*3+2,Z,Error))return false;Dynamic.Positions.Emplace(X,-Y,Z);}
		if(const V* Normals=Field(Row,TEXT("vertex_normals")))
		{
			FView NormalView;if(!Inputs.View(*Normals,ByteCount,NormalView,Error))return false;Dynamic.Normals.Reserve(Desc.VertexCount);
			for(int64 Vx=0;Vx<Desc.VertexCount;++Vx){float X,Y,Z;if(!ReadFloat(Inputs,NormalView,Vx*3,X,Error)||!ReadFloat(Inputs,NormalView,Vx*3+1,Y,Error)||!ReadFloat(Inputs,NormalView,Vx*3+2,Z,Error))return false;const FVector N(X,-Y,Z);const double Length=N.Size();if(!FMath::IsFinite(Length)||Length<1.e-8)return Fail(Error,TEXT("deformable normal is invalid"));Dynamic.Normals.Add(N/Length);}
		}
	}
	if(const V* Debug=Field(*Wire,TEXT("debug_meshes")))
	{
		if(Debug->GetArray().Num()>4096)return Fail(Error,TEXT("initial debug mesh count exceeds limit"));
		for(const V& Row:Debug->GetArray())
		{
			FRenderFrame::FDebugMesh& Mesh=Frame.DebugMeshes.AddDefaulted_GetRef();Mesh.Name=Field(Row,TEXT("name"))->GetString();
			double Color[4];if(!NameValid(Mesh.Name,false)||!FloatArray(Field(Row,TEXT("rgba")),4,Color,Error))return Fail(Error,TEXT("debug mesh identity or color is invalid"));
			for(double C:Color)if(C<0||C>1)return Fail(Error,TEXT("debug mesh color is out of range"));Mesh.Color=FLinearColor(Color[0],Color[1],Color[2],Color[3]);
			const V& PWire=*Field(Row,TEXT("positions_cm"));const int64 PBytes=Field(PWire,TEXT("byte_length"))->GetInt();const V& IWire=*Field(Row,TEXT("triangles"));const int64 IBytes=Field(IWire,TEXT("byte_length"))->GetInt();
			if(PBytes<36||PBytes%12||PBytes>12'000'000||IBytes<12||IBytes%12||IBytes>12'000'000)return Fail(Error,TEXT("debug mesh buffer sizes exceed limits"));
			const int64 Vertices=PBytes/12,Indices=IBytes/4;FView PV,IV;if(!Inputs.View(PWire,PBytes,PV,Error)||!Inputs.View(IWire,IBytes,IV,Error))return false;
			Mesh.Positions.Reserve(Vertices);Mesh.Triangles.Reserve(Indices);
			for(int64 Vx=0;Vx<Vertices;++Vx){float X,Y,Z;if(!ReadFloat(Inputs,PV,Vx*3,X,Error)||!ReadFloat(Inputs,PV,Vx*3+1,Y,Error)||!ReadFloat(Inputs,PV,Vx*3+2,Z,Error))return false;Mesh.Positions.Emplace(X,-Y,Z);}
			// UE's rendered front face uses -cross(P1-P0, P2-P0). Reflecting Y
			// already reverses the RH geometric cross, so keep source corner order.
			for(int64 T=0;T<Indices;T+=3){uint32 A,B,C;if(!ReadUInt(Inputs,IV,T,A,Error)||!ReadUInt(Inputs,IV,T+1,B,Error)||!ReadUInt(Inputs,IV,T+2,C,Error)||A>=Vertices||B>=Vertices||C>=Vertices)return Fail(Error,TEXT("debug mesh index is invalid"));Mesh.Triangles.Add(A);Mesh.Triangles.Add(B);Mesh.Triangles.Add(C);}
		}
	}
	return true;
}

bool BodiesAndGeoms(const V& Source,FRenderScene& Scene,FRenderSceneResources& Resources,FString& Error)
{
	const V* Bodies=Field(Source,TEXT("bodies")); const V* Geoms=Field(Source,TEXT("geoms"));
	if(!Bodies||!Geoms) return Fail(Error,TEXT("neutral bodies/geoms are required"));
	Scene.Bodies.Reserve(Bodies->GetArray().Num());
	for(const V& B:Bodies->GetArray())
	{
		const FString Name=Field(B,TEXT("name"))->GetString();
		if(!NameValid(Name,true)) return Fail(Error,TEXT("neutral body name exceeds its UTF-8 bound"));
		Scene.Bodies.AddDefaulted_GetRef().Name=Name;
	}
	Scene.Geoms.Reserve(Geoms->GetArray().Num()); Resources.Geoms.Reserve(Geoms->GetArray().Num());
	for(const V& G:Geoms->GetArray())
	{
		FRenderGeomDesc D; D.BodyIndex=static_cast<int32>(Field(G,TEXT("body_index"))->GetInt());
		D.Name=Field(G,TEXT("name"))->GetString(); D.Group=static_cast<int32>(Field(G,TEXT("group"))->GetInt());
		if(!Scene.Bodies.IsValidIndex(D.BodyIndex)||D.Group<0||D.Group>30||!NameValid(D.Name,true))
			return Fail(Error,TEXT("neutral geom identity is invalid"));
		if(!Pose(*Field(G,TEXT("body_local_pose")),false,D.BodyLocalTransform,Error)) return false;
		const FString Kind=Field(G,TEXT("kind"))->GetString(); int32 SizeCount=0;
		if(Kind==TEXT("plane")){D.Kind=ERenderGeomKind::Plane;SizeCount=2;}
		else if(Kind==TEXT("sphere")){D.Kind=ERenderGeomKind::Sphere;SizeCount=1;}
		else if(Kind==TEXT("ellipsoid")){D.Kind=ERenderGeomKind::Ellipsoid;SizeCount=3;}
		else if(Kind==TEXT("cylinder")){D.Kind=ERenderGeomKind::Cylinder;SizeCount=2;}
		else if(Kind==TEXT("capsule")){D.Kind=ERenderGeomKind::Capsule;SizeCount=2;}
		else if(Kind==TEXT("box")){D.Kind=ERenderGeomKind::Box;SizeCount=3;}
		else if(Kind==TEXT("mesh")){D.Kind=ERenderGeomKind::Mesh;SizeCount=0;}
		else return Fail(Error,TEXT("neutral geom kind is unsupported"));
		double S[3]={0,0,0}; if(!FloatArray(Field(G,TEXT("size")),SizeCount,S,Error)) return false;
		for(int32 I=0;I<SizeCount;++I) if(S[I]<=0||!FMath::IsFinite(static_cast<float>(S[I]*100.0)))
			return Fail(Error,TEXT("neutral geom size is invalid or unrepresentable"));
		D.SizeCm=FVector(S[0]*100.0,S[SizeCount>1?1:0]*100.0,S[SizeCount>2?2:(SizeCount>1?1:0)]*100.0);
		const V* Mesh=Field(G,TEXT("mesh_index"));
		if((D.Kind==ERenderGeomKind::Mesh)!=(Mesh!=nullptr)) return Fail(Error,TEXT("neutral geom mesh index presence is invalid"));
		if(Mesh) D.MeshResourceIndex=static_cast<int32>(Mesh->GetInt());
		const V& W=*Field(G,TEXT("surface")); FRenderSurfaceBinding Surface;
		if(const V* M=Field(W,TEXT("material_index"))) Surface.MaterialIndex=static_cast<int32>(M->GetInt());
		double C[4]={1,1,1,1},U[2]={0,0}; if((Field(W,TEXT("rgba"))&&!FloatArray(Field(W,TEXT("rgba")),4,C,Error))||(Field(W,TEXT("texuniform_scale"))&&!FloatArray(Field(W,TEXT("texuniform_scale")),2,U,Error)))return false;
		for(double X:C)if(X<0||X>1)return Fail(Error,TEXT("neutral surface rgba is out of range"));
		if(U[0]<0||U[1]<0)return Fail(Error,TEXT("neutral texuniform scale is negative"));
		Surface.Rgba=FLinearColor(C[0],C[1],C[2],C[3]);Surface.TexUniformScale=FVector2D(U[0],U[1]);
		Scene.Geoms.Add(MoveTemp(D));Resources.Geoms.Add(MoveTemp(Surface));
	}
	return true;
}

bool Materials(const V& Source,FRenderSceneResources& Resources,FString& Error)
{
	const V* Values=Field(Source,TEXT("materials"));if(!Values)return Fail(Error,TEXT("neutral materials are required"));
	static const TCHAR* Roles[]={TEXT("rgb"),TEXT("occlusion"),TEXT("roughness"),TEXT("metallic"),TEXT("normal"),TEXT("opacity"),TEXT("emissive"),TEXT("rgba"),TEXT("orm")};
	Resources.Materials.Reserve(Values->GetArray().Num());
	for(const V& W:Values->GetArray())
	{
		FRenderMaterialResource M;double Rgba[4]={1,1,1,1},Repeat[2]={1,1};
		if((Field(W,TEXT("rgba"))&&!FloatArray(Field(W,TEXT("rgba")),4,Rgba,Error))||(Field(W,TEXT("texrepeat"))&&!FloatArray(Field(W,TEXT("texrepeat")),2,Repeat,Error)))return false;
		auto Scalar=[&](const TCHAR* N,float& Out,double Min,double Max){const V* Value=Field(W,N);if(!Value)return true;double X=Value->GetFloat();if(!FMath::IsFinite(X)||X<Min||X>Max)return Fail(Error,FString::Printf(TEXT("neutral material %s is out of range"),N));Out=static_cast<float>(X);return true;};
		for(double X:Rgba)if(X<0||X>1)return Fail(Error,TEXT("neutral material rgba is out of range"));
		M.Rgba=FLinearColor(Rgba[0],Rgba[1],Rgba[2],Rgba[3]);
		if(!Scalar(TEXT("emission"),M.Emission,0,TNumericLimits<double>::Max())||!Scalar(TEXT("specular"),M.Specular,0,1)
			||!Scalar(TEXT("shininess"),M.Shininess,0,1)||!Scalar(TEXT("reflectance"),M.Reflectance,0,1)
			||!Scalar(TEXT("metallic"),M.Metallic,0,1)||!Scalar(TEXT("roughness"),M.Roughness,0,1)
			||Repeat[0]<=0||Repeat[1]<=0)return false;
		M.TexRepeat=FVector2D(Repeat[0],Repeat[1]);if(const V* Uniform=Field(W,TEXT("texuniform")))M.bTexUniform=Uniform->GetBool();
		const V* Bind=Field(W,TEXT("texture_roles"));
		for(int32 I=0;Bind&&I<UE_ARRAY_COUNT(Roles);++I)if(const V* X=Field(*Bind,Roles[I]))
		{
			M.Textures[I].TextureIndex=static_cast<int32>(X->GetInt());
			M.Textures[I].bNormal=I==static_cast<int32>(ERenderMaterialRole::Normal);
			M.Textures[I].bSRGB=I==static_cast<int32>(ERenderMaterialRole::Rgb)||I==static_cast<int32>(ERenderMaterialRole::Emissive)||I==static_cast<int32>(ERenderMaterialRole::Rgba);
		}
		Resources.Materials.Add(MoveTemp(M));
	}
	return true;
}

bool Textures(FInputs& Inputs,const V& Source,FRenderSceneResources& Resources,FString& Error)
{
	const V* Values=Field(Source,TEXT("textures"));if(!Values)return Fail(Error,TEXT("neutral textures are required"));
	Resources.Textures.Reserve(Values->GetArray().Num());
	for(const V& W:Values->GetArray())
	{
		const int64 Width=Field(W,TEXT("width"))->GetInt(),Height=Field(W,TEXT("height"))->GetInt();int64 Pixels=0,Bytes=0;
		if(Width<=0||Height<=0||Width>MAX_int32||Height>MAX_int32||!CheckedMul(Width,Height,Pixels)||!CheckedMul(Pixels,4,Bytes)||Bytes>MAX_int32)
			return Fail(Error,TEXT("neutral texture dimensions overflow"));
		FView View;if(!Inputs.View(*Field(W,TEXT("rgba8")),Bytes,View,Error))return false;
		FRenderTextureResource T;T.Width=Width;T.Height=Height;T.Rgba8.SetNumUninitialized(Bytes,EAllowShrinking::No);
		if(!Inputs.Read(View,0,T.Rgba8.GetData(),Bytes,Error))return false;Resources.Textures.Add(MoveTemp(T));
	}
	return true;
}

bool Meshes(FInputs& Inputs,const V& Source,FRenderSceneResources& Resources,FString& Error)
{
	const V* Values=Field(Source,TEXT("meshes"));if(!Values)return Fail(Error,TEXT("neutral meshes are required"));
	Resources.Meshes.Reserve(Values->GetArray().Num());
	for(const V& W:Values->GetArray())
	{
		const int64 Vertices=Field(W,TEXT("vertex_count"))->GetInt(),Indices=Field(W,TEXT("triangle_index_count"))->GetInt();
		int64 PBytes=0,NBytes=0,UBytes=0,IBytes=0;
		if(Vertices<3||Vertices>MAX_int32||Indices<=0||Indices>MAX_int32||Indices%3||!CheckedMul(Vertices*3,4,PBytes)||!CheckedMul(Vertices*3,4,NBytes)||!CheckedMul(Vertices*2,4,UBytes)||!CheckedMul(Indices,4,IBytes))return Fail(Error,TEXT("neutral mesh counts are invalid"));
		FView P,N,U,I;if(!Inputs.View(*Field(W,TEXT("positions")),PBytes,P,Error)||!Inputs.View(*Field(W,TEXT("normals")),NBytes,N,Error)||!Inputs.View(*Field(W,TEXT("uvs")),UBytes,U,Error)||!Inputs.View(*Field(W,TEXT("triangles")),IBytes,I,Error))return false;
		FRenderMeshResource M;M.Positions.Reserve(Vertices);M.Normals.Reserve(Vertices);M.UVs.Reserve(Vertices);M.Triangles.Reserve(Indices);
		for(int64 Vx=0;Vx<Vertices;++Vx)
		{
			float X,Y,Z,NX,NY,NZ,Ux,Uy;if(!ReadFloat(Inputs,P,Vx*3,X,Error)||!ReadFloat(Inputs,P,Vx*3+1,Y,Error)||!ReadFloat(Inputs,P,Vx*3+2,Z,Error)||!ReadFloat(Inputs,N,Vx*3,NX,Error)||!ReadFloat(Inputs,N,Vx*3+1,NY,Error)||!ReadFloat(Inputs,N,Vx*3+2,NZ,Error)||!ReadFloat(Inputs,U,Vx*2,Ux,Error)||!ReadFloat(Inputs,U,Vx*2+1,Uy,Error))return false;
			const FVector Pos(static_cast<double>(X)*100.0,-static_cast<double>(Y)*100.0,static_cast<double>(Z)*100.0);const FVector3f Narrow(Pos);
			if(!FMath::IsFinite(Narrow.X)||!FMath::IsFinite(Narrow.Y)||!FMath::IsFinite(Narrow.Z))return Fail(Error,TEXT("neutral mesh position is not float32 representable after scaling"));
			FVector Normal(NX,-NY,NZ);const double Len=Normal.Size();if(!FMath::IsFinite(Len)||FMath::Abs(Len-1.0)>1.e-4)return Fail(Error,TEXT("neutral mesh normal is not unit length"));
			M.Positions.Add(Pos);M.Normals.Add(Normal/Len);M.UVs.Add(FVector2D(Ux,Uy));
		}
		// UE's front face is opposite the conventional cross product. The Y
		// reflection above supplies that reversal without reordering indices.
		for(int64 T=0;T<Indices;T+=3){uint32 A,B,C;if(!ReadUInt(Inputs,I,T,A,Error)||!ReadUInt(Inputs,I,T+1,B,Error)||!ReadUInt(Inputs,I,T+2,C,Error))return false;if(A>=Vertices||B>=Vertices||C>=Vertices)return Fail(Error,TEXT("neutral mesh triangle index is out of range"));M.Triangles.Add(A);M.Triangles.Add(B);M.Triangles.Add(C);}
		Resources.Meshes.Add(MoveTemp(M));
	}
	return true;
}

bool Camera(const V& W,FRenderCameraDesc& C,FString& Error)
{
	C.CanonicalName=Field(W,TEXT("canonical_name"))->GetString();if(!NameValid(C.CanonicalName,false))return Fail(Error,TEXT("neutral camera name is invalid"));
	if(const V* ModeValue=Field(W,TEXT("mode"))){const FString Mode=ModeValue->GetString();if(Mode==TEXT("real"))C.Capture.Mode=ERenderCameraMode::Real;else if(Mode==TEXT("depth"))C.Capture.Mode=ERenderCameraMode::Depth;else if(Mode==TEXT("semantic"))C.Capture.Mode=ERenderCameraMode::SemanticSegmentation;else if(Mode==TEXT("instance"))C.Capture.Mode=ERenderCameraMode::InstanceSegmentation;else return Fail(Error,TEXT("neutral camera mode is invalid"));}
	if(const V* R=Field(W,TEXT("resolution"))){const int64 Width=Field(*R,TEXT("width"))->GetInt(),Height=Field(*R,TEXT("height"))->GetInt();if(Width<=0||Height<=0||Width>MAX_int32||Height>MAX_int32)return Fail(Error,TEXT("neutral camera resolution is invalid"));C.Optics.Resolution=FIntPoint(Width,Height);}
	if(const V* ProjectionValue=Field(W,TEXT("projection"))){const FString Projection=ProjectionValue->GetString();if(Projection==TEXT("perspective"))C.Optics.Projection=ERenderCameraProjection::Perspective;else if(Projection==TEXT("orthographic"))C.Optics.Projection=ERenderCameraProjection::Orthographic;else return Fail(Error,TEXT("neutral camera projection is invalid"));}
	auto F=[&](const TCHAR* N,float& Out,double Scale=1.0){const V* Value=Field(W,N);if(!Value)return true;const double X=Value->GetFloat()*Scale;if(!FMath::IsFinite(X)||!FMath::IsFinite(static_cast<float>(X)))return Fail(Error,FString::Printf(TEXT("neutral camera %s is unrepresentable"),N));Out=static_cast<float>(X);return true;};
	if(!F(TEXT("vertical_fov_degrees"),C.Optics.VerticalFovDegrees)||!F(TEXT("orthographic_width_m"),C.Optics.OrthographicWidthCm,100)||!F(TEXT("clip_near_m"),C.Optics.NearClipCm,100)||!F(TEXT("clip_far_m"),C.Optics.FarClipCm,100)||!F(TEXT("depth_near_m"),C.Capture.DepthNearCm,100)||!F(TEXT("depth_far_m"),C.Capture.DepthFarCm,100))return false;
	if(C.Optics.VerticalFovDegrees<=0||C.Optics.VerticalFovDegrees>=180||C.Optics.NearClipCm<=0||C.Optics.FarClipCm<=C.Optics.NearClipCm||C.Capture.DepthNearCm<=0||C.Capture.DepthFarCm<=C.Capture.DepthNearCm||(C.Optics.Projection==ERenderCameraProjection::Orthographic&&C.Optics.OrthographicWidthCm<=0))return Fail(Error,TEXT("neutral camera optics are out of range"));
	if(const V* I=Field(W,TEXT("intrinsics"))){C.Optics.bHasIntrinsics=true;auto IF=[&](const TCHAR*N,float& Out){double X=Field(*I,N)->GetFloat();if(!FMath::IsFinite(X)||!FMath::IsFinite(static_cast<float>(X)))return false;Out=X;return true;};if(!IF(TEXT("fx"),C.Optics.FocalPixelsX)||!IF(TEXT("fy"),C.Optics.FocalPixelsY)||!IF(TEXT("cx"),C.Optics.PrincipalPixelsX)||!IF(TEXT("cy"),C.Optics.PrincipalPixelsY)||C.Optics.FocalPixelsX<=0||C.Optics.FocalPixelsY<=0)return Fail(Error,TEXT("neutral camera intrinsics are invalid"));}
	C.Capture.bManualCaptureOnly=true;return true;
}

bool Cameras(const V& Source,FRenderScene& Scene,FString& Error)
{
	const V* Values=Field(Source,TEXT("model_cameras"));if(!Values)return Fail(Error,TEXT("neutral model cameras are required"));TSet<FString> Names;
	Scene.Cameras.Reserve(Values->GetArray().Num());for(const V& W:Values->GetArray()){FRenderCameraDesc C;if(!Camera(W,C,Error)||Names.Contains(C.CanonicalName))return Error.IsEmpty()?Fail(Error,TEXT("neutral camera name is duplicate")):false;Names.Add(C.CanonicalName);Scene.Cameras.Add(MoveTemp(C));}
	if(const V* W=Field(Source,TEXT("user_camera"))){FRenderCameraDesc C;if(!Camera(*W,C,Error)||Names.Contains(C.CanonicalName))return Error.IsEmpty()?Fail(Error,TEXT("neutral user camera name is duplicate")):false;Scene.UserCamera=MoveTemp(C);}return true;
}

bool Lights(const V& Source,FRenderScene& Scene,FString& Error)
{
	const V* Values=Field(Source,TEXT("lights"));
	if(!Values)return Fail(Error,TEXT("neutral lights are required"));
	Scene.Lights.Reserve(Values->GetArray().Num());
	for(const V& W:Values->GetArray())
	{
		FRenderLightDesc D;D.Name=Field(W,TEXT("name"))->GetString();
		if(!NameValid(D.Name,true))return Fail(Error,TEXT("neutral light name exceeds its UTF-8 bound"));
		const FString Kind=Field(W,TEXT("kind"))->GetString();
		if(Kind==TEXT("directional"))D.Kind=ERenderLightKind::Directional;
		else if(Kind==TEXT("point"))D.Kind=ERenderLightKind::Point;
		else if(Kind==TEXT("spot"))D.Kind=ERenderLightKind::Spot;
		else return Fail(Error,TEXT("neutral light kind is unsupported"));
		D.bEnabled=Field(W,TEXT("enabled"))->GetBool();
		D.bCastShadows=Field(W,TEXT("cast_shadows"))->GetBool();
		double Color[3];if(!FloatArray(Field(W,TEXT("color_rgb")),3,Color,Error))return false;
		for(double X:Color)if(X<0.0||X>1.0)return Fail(Error,TEXT("neutral light color is out of range"));
		D.Color=FLinearColor(Color[0],Color[1],Color[2],1.0f);
		auto Scalar=[&](const TCHAR* Name,float& Out,double Scale=1.0){const double X=Field(W,Name)->GetFloat()*Scale;if(!FMath::IsFinite(X)||!FMath::IsFinite(static_cast<float>(X)))return Fail(Error,FString::Printf(TEXT("neutral light %s is unrepresentable"),Name));Out=static_cast<float>(X);return true;};
		if(!Scalar(TEXT("intensity"),D.Intensity)||!Scalar(TEXT("range_m"),D.RangeCm,100.0)
			||!Scalar(TEXT("source_radius_m"),D.SourceRadiusCm,100.0)
			||!Scalar(TEXT("inner_cone_degrees"),D.InnerConeDegrees)
			||!Scalar(TEXT("outer_cone_degrees"),D.OuterConeDegrees))return false;
		Scene.Lights.Add(MoveTemp(D));
	}
	return true;
}

bool EnvironmentAndVisualization(const V& Source,FRenderScene& Scene,FString& Error)
{
	if(const V* E=Field(Source,TEXT("environment")))
	{
		double C[4];
		if(!FloatArray(Field(*E,TEXT("ambient_color_rgba")),4,C,Error))return false;
		Scene.Environment.AmbientColor=FLinearColor(C[0],C[1],C[2],C[3]);
		Scene.Environment.AmbientIntensity=Field(*E,TEXT("ambient_intensity"))->GetFloat();
		if(!FloatArray(Field(*E,TEXT("sky_top_rgba")),4,C,Error))return false;
		Scene.Environment.SkyTop=FLinearColor(C[0],C[1],C[2],C[3]);
		if(!FloatArray(Field(*E,TEXT("sky_bottom_rgba")),4,C,Error))return false;
		Scene.Environment.SkyBottom=FLinearColor(C[0],C[1],C[2],C[3]);
		if(!FloatArray(Field(*E,TEXT("headlight_color_rgba")),4,C,Error))return false;
		Scene.Environment.HeadlightColor=FLinearColor(C[0],C[1],C[2],C[3]);
		Scene.Environment.HeadlightIntensity=Field(*E,TEXT("headlight_intensity"))->GetFloat();
		Scene.Environment.bSkyboxEnabled=Field(*E,TEXT("skybox_enabled"))->GetBool();
		Scene.Environment.bHeadlightEnabled=Field(*E,TEXT("headlight_enabled"))->GetBool();
		for(float X:{Scene.Environment.AmbientIntensity,Scene.Environment.HeadlightIntensity})
			if(!FMath::IsFinite(X)||X<0)return Fail(Error,TEXT("neutral environment intensity is invalid"));
		for(const FLinearColor& Color:{Scene.Environment.AmbientColor,Scene.Environment.SkyTop,Scene.Environment.SkyBottom,Scene.Environment.HeadlightColor})
			if(!FMath::IsFinite(Color.R)||!FMath::IsFinite(Color.G)||!FMath::IsFinite(Color.B)||!FMath::IsFinite(Color.A)||Color.R<0||Color.G<0||Color.B<0||Color.A<0||Color.R>1||Color.G>1||Color.B>1||Color.A>1)
				return Fail(Error,TEXT("neutral environment color is out of range"));
		if(const V* Faces=Field(*E,TEXT("skybox_face_texture_indices")))
		{
			if(Faces->GetArray().Num()!=6)return Fail(Error,TEXT("skybox requires six texture indices"));
			for(const V& Face:Faces->GetArray())Scene.Environment.SkyboxFaceTextureIndices.Add(static_cast<int32>(Face.GetInt()));
		}
	}
	if(const V* Config=Field(Source,TEXT("visualization")))
	{
		TSet<FString> Seen;
		for(const V& Feature:Field(*Config,TEXT("features"))->GetArray())
		{
			const FString Name=Feature.GetString();
			if(Name.IsEmpty()||Seen.Contains(Name))return Fail(Error,TEXT("neutral visualization feature is empty or duplicate"));
			Seen.Add(Name);Scene.Visualization.Features.Add(Name);
		}
	}
	return true;
}

bool Deformables(FInputs& Inputs,const V& Source,FRenderScene& Scene,FRenderSceneResources& Resources,FString& Error)
{
	if(const V* Surfaces=Field(Source,TEXT("deformable_surfaces")))
	{
		if(Surfaces->GetArray().Num()>4096)return Fail(Error,TEXT("neutral deformable surface count exceeds limit"));
		for(const V& Wire:Surfaces->GetArray())
		{
			FRenderSurfaceBinding& Surface=Resources.Deformables.AddDefaulted_GetRef();
			if(const V* M=Field(Wire,TEXT("material_index")))
			{
				if(M->GetInt()>MAX_int32)return Fail(Error,TEXT("neutral deformable material index is too large"));
				Surface.MaterialIndex=static_cast<int32>(M->GetInt());
			}
			if(const V* R=Field(Wire,TEXT("rgba")))
			{
				double C[4];if(!FloatArray(R,4,C,Error))return false;
				Surface.Rgba=FLinearColor(C[0],C[1],C[2],C[3]);
			}
			if(const V* S=Field(Wire,TEXT("texuniform_scale")))
			{
				double U[2];if(!FloatArray(S,2,U,Error))return false;
				Surface.TexUniformScale=FVector2D(U[0],U[1]);
			}
		}
	}
	const V* Values=Field(Source,TEXT("deformables"));if(!Values)return true;
	if(Values->GetArray().Num()>4096)return Fail(Error,TEXT("neutral deformable count exceeds limit"));
	TSet<FString> Names;
	for(const V& W:Values->GetArray())
	{
		FRenderDeformableDesc D;D.Name=Field(W,TEXT("name"))->GetString();
		const int64 Count=Field(W,TEXT("vertex_count"))->GetInt(),Surface=Field(W,TEXT("surface_index"))->GetInt();
		const V& TriangleWire=*Field(W,TEXT("triangles"));const int64 Bytes=Field(TriangleWire,TEXT("byte_length"))->GetInt();
		if(!NameValid(D.Name,false)||Names.Contains(D.Name)||Count<3||Count>2'000'000||Surface<0||Surface>4095||Bytes<12||Bytes%12||Bytes>48'000'000)
			return Fail(Error,TEXT("neutral deformable identity or topology size is invalid"));
		FView View;if(!Inputs.View(TriangleWire,Bytes,View,Error))return false;
		if(!Resources.Deformables.IsValidIndex(static_cast<int32>(Surface)))return Fail(Error,TEXT("neutral deformable surface index is out of range"));
		D.VertexCount=Count;D.SurfaceIndex=Surface;D.Triangles.Reserve(Bytes/4);
		for(int64 I=0;I<Bytes/4;++I){uint32 Index=0;if(!ReadUInt(Inputs,View,I,Index,Error)||Index>=Count)return Fail(Error,TEXT("neutral deformable triangle index is invalid"));D.Triangles.Add(static_cast<int32>(Index));}
		if(const V* UVWire=Field(W,TEXT("uvs")))
		{
			int64 UVBytes=0;if(!CheckedMul(Count,8,UVBytes))return Fail(Error,TEXT("neutral deformable UV length overflows"));
			FView UVView;if(!Inputs.View(*UVWire,UVBytes,UVView,Error))return false;
			D.UVs.Reserve(Count);
			for(int64 Vertex=0;Vertex<Count;++Vertex){float U,V;if(!ReadFloat(Inputs,UVView,2*Vertex,U,Error)||!ReadFloat(Inputs,UVView,2*Vertex+1,V,Error))return false;D.UVs.Emplace(U,V);}
		}
		Names.Add(D.Name);Scene.Deformables.Add(MoveTemp(D));
	}
	return true;
}

bool PlanBytes(const V& Source,int64& Total,FString& Error)
{
	Total=0;const V* B=Field(Source,TEXT("bodies"));const V* G=Field(Source,TEXT("geoms"));const V* Me=Field(Source,TEXT("meshes"));const V* T=Field(Source,TEXT("textures"));const V* M=Field(Source,TEXT("materials"));const V* C=Field(Source,TEXT("model_cameras"));const V* L=Field(Source,TEXT("lights"));
	if(!B||!G||!Me||!T||!M||!C||!L||!ArrayBytes<FRenderBodyDesc>(B->GetArray().Num(),Total)||!ArrayBytes<FRenderGeomDesc>(G->GetArray().Num(),Total)||!ArrayBytes<FRenderSurfaceBinding>(G->GetArray().Num(),Total)||!ArrayBytes<FRenderMeshResource>(Me->GetArray().Num(),Total)||!ArrayBytes<FRenderTextureResource>(T->GetArray().Num(),Total)||!ArrayBytes<FRenderMaterialResource>(M->GetArray().Num(),Total)||!ArrayBytes<FRenderCameraDesc>(C->GetArray().Num(),Total)||!ArrayBytes<FRenderLightDesc>(L->GetArray().Num(),Total)||!ArrayBytes<FTransform>(B->GetArray().Num(),Total)||!ArrayBytes<FTransform>(C->GetArray().Num(),Total)||!ArrayBytes<FTransform>(L->GetArray().Num(),Total))return Fail(Error,TEXT("neutral decoded metadata capacity overflows"));
	if(Field(Source,TEXT("user_camera"))&&!CheckedAdd(sizeof(FRenderCameraDesc)+sizeof(FTransform),Total))return Fail(Error,TEXT("neutral user camera capacity overflows"));
	if(const V* Surfaces=Field(Source,TEXT("deformable_surfaces")))
		if(Surfaces->GetArray().Num()>4096||!ArrayBytes<FRenderSurfaceBinding>(Surfaces->GetArray().Num(),Total))return Fail(Error,TEXT("neutral deformable surface capacity exceeds limit"));
	for(const V& W:Me->GetArray()){const int64 Vc=Field(W,TEXT("vertex_count"))->GetInt(),Ic=Field(W,TEXT("triangle_index_count"))->GetInt();if(!ArrayBytes<FVector>(Vc,Total)||!ArrayBytes<FVector>(Vc,Total)||!ArrayBytes<FVector2D>(Vc,Total)||!ArrayBytes<int32>(Ic,Total))return Fail(Error,TEXT("neutral mesh capacity overflows"));}
	for(const V& W:T->GetArray()){int64 P=0;if(!CheckedMul(Field(W,TEXT("width"))->GetInt(),Field(W,TEXT("height"))->GetInt(),P)||!CheckedMul(P,4,P)||!ArrayBytes<uint8>(P,Total))return Fail(Error,TEXT("neutral texture capacity overflows"));}
	if(const V* Ds=Field(Source,TEXT("deformables")))
	{
		if(Ds->GetArray().Num()>4096||!ArrayBytes<FRenderDeformableDesc>(Ds->GetArray().Num(),Total))return Fail(Error,TEXT("neutral deformable metadata capacity exceeds limit"));
		int64 DynamicVertices=0,DynamicBytes=0;
		for(const V& D:Ds->GetArray())
		{
			const FString& Name=Field(D,TEXT("name"))->GetString();int64 NameBytes=0;
			if(!NameValid(Name,false)||!CheckedMul(Name.Len()+1,sizeof(TCHAR),NameBytes)
				||!CheckedAdd(NameBytes,Total))return Fail(Error,TEXT("neutral deformable name exceeds limit"));
			const int64 Count=Field(D,TEXT("vertex_count"))->GetInt(),Bytes=Field(*Field(D,TEXT("triangles")),TEXT("byte_length"))->GetInt();
			if(Count<3||Count>2'000'000||Bytes<12||Bytes%12||Bytes>48'000'000||!ArrayBytes<int32>(Bytes/4,Total)||!ArrayBytes<FVector>(Count,Total)||!ArrayBytes<FVector>(Count,Total)||(Field(D,TEXT("uvs"))&&!ArrayBytes<FVector2D>(Count,Total))||!CheckedAdd(Count,DynamicVertices))return Fail(Error,TEXT("neutral deformable allocation exceeds limit"));
		}
		if(DynamicVertices>1'000'000||!CheckedMul(DynamicVertices,24,DynamicBytes)||DynamicBytes>128ll*1024*1024||!CheckedAdd(DynamicBytes,Total))return Fail(Error,TEXT("neutral deformable aggregate allocation exceeds limit"));
	}
	if(const V* Initial=Field(Source,TEXT("initial_frame")))if(const V* Vis=Field(*Initial,TEXT("visualization")))if(const V* Meshes=Field(*Vis,TEXT("debug_meshes")))
	{
		if(Meshes->GetArray().Num()>4096||!ArrayBytes<FRenderFrame::FDebugMesh>(Meshes->GetArray().Num(),Total))return Fail(Error,TEXT("neutral debug mesh count exceeds limit"));
		int64 Vertices=0,Indices=0;
		for(const V& Mesh:Meshes->GetArray())
		{
			const FString& Name=Field(Mesh,TEXT("name"))->GetString();int64 NameBytes=0;
			if(!NameValid(Name,false)||!CheckedMul(Name.Len()+1,sizeof(TCHAR),NameBytes)
				||!CheckedAdd(NameBytes,Total))return Fail(Error,TEXT("neutral initial debug mesh name exceeds limit"));
			const int64 PB=Field(*Field(Mesh,TEXT("positions_cm")),TEXT("byte_length"))->GetInt(),IB=Field(*Field(Mesh,TEXT("triangles")),TEXT("byte_length"))->GetInt();
			if(PB<36||PB%12||IB<12||IB%12||!CheckedAdd(PB/12,Vertices)||!CheckedAdd(IB/4,Indices)||!ArrayBytes<FVector>(PB/12,Total)||!ArrayBytes<int32>(IB/4,Total))return Fail(Error,TEXT("neutral debug mesh allocation is invalid"));
		}
		int64 Dynamic=0;if(Vertices>1'000'000||Indices>3'000'000||!CheckedMul(Vertices,24,Dynamic)||!CheckedAdd(Indices*4,Dynamic)||Dynamic>128ll*1024*1024||!CheckedAdd(Dynamic,Total))return Fail(Error,TEXT("neutral debug mesh aggregate allocation exceeds limit"));
	}
	return true;
}
}

bool FURLabNeutralRenderSourceIngress::Prepare(const TSharedPtr<FURLabPayloadStore,ESPMode::ThreadSafe>& Store,const FURLabPayloadOwner& Owner,const FURLabWireValue& Source,const FString& ServerSceneRevision,FRenderSceneInstallPayload& OutPayload,FString& OutError)
{
	OutError.Reset();if(!Store||!Owner.Session||Owner.BootId.IsEmpty()||Owner.SessionId.IsEmpty()||ServerSceneRevision.IsEmpty()||!Field(Source,TEXT("format"))||Field(Source,TEXT("format"))->GetString()!=TEXT("neutral"))return Fail(OutError,TEXT("neutral ingress requires exact store owner, revision, and format"));
	int64 Bytes=0;if(!PlanBytes(Source,Bytes,OutError))return false;FURLabPayloadPhysicalReservationPtr Reservation;if(Bytes>0){const auto S=Store->ReservePhysicalCapacity(Owner,Bytes,Reservation);if(!S)return Fail(OutError,S.Message);}
	FInputs Inputs(*Store,Owner);FRenderScene Scene;FRenderSceneResources Resources;FRenderFrame Frame;Scene.SceneRevision=ServerSceneRevision;Resources.SceneRevision=ServerSceneRevision;
	if(!BodiesAndGeoms(Source,Scene,Resources,OutError)||!Materials(Source,Resources,OutError)||!Textures(Inputs,Source,Resources,OutError)||!Meshes(Inputs,Source,Resources,OutError)||!Cameras(Source,Scene,OutError)||!Lights(Source,Scene,OutError)||!EnvironmentAndVisualization(Source,Scene,OutError)||!Deformables(Inputs,Source,Scene,Resources,OutError))return false;
	const V* Initial=Field(Source,TEXT("initial_frame"));if(!Initial||!FrameFromView(Inputs,*Initial,Scene.Bodies.Num(),Scene.Cameras.Num(),Scene.UserCamera.IsSet(),Scene.Lights.Num(),ServerSceneRevision,Frame,OutError)||!FrameVisualization(Inputs,Field(*Initial,TEXT("visualization")),Scene.Deformables,Frame,OutError))return false;
	for(FRenderGeomDesc& G:Scene.Geoms)if(G.Kind==ERenderGeomKind::Mesh&&!Resources.Meshes.IsValidIndex(G.MeshResourceIndex))return Fail(OutError,TEXT("neutral geom mesh index is out of range"));
	for(const FRenderSurfaceBinding& G:Resources.Geoms)if(G.MaterialIndex!=INDEX_NONE&&!Resources.Materials.IsValidIndex(G.MaterialIndex))return Fail(OutError,TEXT("neutral geom material index is out of range"));
	for(const FRenderSurfaceBinding& D:Resources.Deformables)if(D.MaterialIndex!=INDEX_NONE&&!Resources.Materials.IsValidIndex(D.MaterialIndex))return Fail(OutError,TEXT("neutral deformable material index is out of range"));
	for(const FRenderMaterialResource& M:Resources.Materials)for(const FRenderTextureBinding& B:M.Textures)if(B.TextureIndex!=INDEX_NONE&&!Resources.Textures.IsValidIndex(B.TextureIndex))return Fail(OutError,TEXT("neutral material texture index is out of range"));
	for(int32 TextureIndex:Scene.Environment.SkyboxFaceTextureIndices)if(!Resources.Textures.IsValidIndex(TextureIndex))return Fail(OutError,TEXT("neutral skybox texture index is out of range"));
	FString Validation;if(!Resources.IsValid(Validation)||!Scene.IsValid(Resources,MAX_int32,Validation)||!Frame.IsValid(Scene.Bodies.Num(),Scene.Cameras.Num(),Scene.UserCamera.IsSet(),Scene.Lights.Num(),ServerSceneRevision,Validation))return Fail(OutError,Validation);
	auto Lease=MakeShared<FNeutralLease,ESPMode::ThreadSafe>(MoveTemp(Reservation));OutPayload=FRenderSceneInstallPayload(MoveTemp(Scene),MoveTemp(Resources),MoveTemp(Frame),MoveTemp(Lease));return true;
}

bool FURLabNeutralRenderSourceIngress::DecodeFrameVisualization(
	const TSharedPtr<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
	const FURLabPayloadOwner& Owner, const FURLabWireValue& Visualization,
	const TArray<FRenderDeformableDesc>& Deformables, FRenderFrame& InOutFrame,
	FRenderCameraCaptureLifetimeLeasePtr& OutLease, FString& OutError)
{
	OutError.Reset(); OutLease.Reset();
	if(!Store||!Owner.Session||Owner.BootId.IsEmpty()||Owner.SessionId.IsEmpty())
		return Fail(OutError,TEXT("dynamic frame requires exact payload owner"));
	const V* Rows=Field(Visualization,TEXT("deformables"));
	if(Deformables.Num()!=(Rows?Rows->GetArray().Num():0))
		return Fail(OutError,TEXT("dynamic frame must include every installed deformable"));
	int64 Total=0, VertexCount=0, TriangleCount=0;
	if(!ArrayBytes<FRenderFrame::FDeformable>(Deformables.Num(),Total))
		return Fail(OutError,TEXT("dynamic deformable allocation overflows"));
	for(const FRenderDeformableDesc& D:Deformables)
	{
		int64 NameBytes=0;
		if(!NameValid(D.Name,false)||!CheckedMul(D.Name.Len()+1,sizeof(TCHAR),NameBytes)
			||!CheckedAdd(NameBytes,Total)||D.VertexCount<3||D.VertexCount>2'000'000||!CheckedAdd(D.VertexCount,VertexCount)
			||!ArrayBytes<FVector>(D.VertexCount,Total)||!ArrayBytes<FVector>(D.VertexCount,Total))
			return Fail(OutError,TEXT("dynamic deformable allocation exceeds limit"));
	}
	if(VertexCount>1'000'000)return Fail(OutError,TEXT("dynamic deformable vertex budget exceeded"));
	if(const V* Debug=Field(Visualization,TEXT("debug_meshes")))
	{
		if(Debug->GetArray().Num()>4096||!ArrayBytes<FRenderFrame::FDebugMesh>(Debug->GetArray().Num(),Total))
			return Fail(OutError,TEXT("dynamic debug mesh count exceeds limit"));
		for(const V& Mesh:Debug->GetArray())
		{
			const V* Name=Field(Mesh,TEXT("name"));int64 NameBytes=0;
			if(!Name||!NameValid(Name->GetString(),false)
				||!CheckedMul(Name->GetString().Len()+1,sizeof(TCHAR),NameBytes)
				||!CheckedAdd(NameBytes,Total))
				return Fail(OutError,TEXT("dynamic debug mesh name exceeds limit"));
			const V* Positions=Field(Mesh,TEXT("positions_cm"));
			const V* Triangles=Field(Mesh,TEXT("triangles"));
			if(!Positions||!Triangles)return Fail(OutError,TEXT("dynamic debug mesh views are missing"));
			const int64 P=Field(*Positions,TEXT("byte_length"))->GetInt();
			const int64 I=Field(*Triangles,TEXT("byte_length"))->GetInt();
			if(P<36||P%12||P>12'000'000||I<12||I%12||I>12'000'000
				||!CheckedAdd(P/12,VertexCount)||!CheckedAdd(I/4,TriangleCount)
				||!ArrayBytes<FVector>(P/12,Total)||!ArrayBytes<int32>(I/4,Total))
				return Fail(OutError,TEXT("dynamic debug mesh allocation exceeds limit"));
		}
	}
	if(VertexCount>2'000'000||TriangleCount>3'000'000||Total>128ll*1024*1024)
		return Fail(OutError,TEXT("dynamic visualization aggregate budget exceeded"));
	FURLabPayloadPhysicalReservationPtr Reservation;
	if(Total>0)
	{
		const auto Status=Store->ReservePhysicalCapacity(Owner,Total,Reservation);
		if(!Status)return Fail(OutError,Status.Message);
	}
	FInputs Inputs(*Store,Owner);
	if(!FrameVisualization(Inputs,&Visualization,Deformables,InOutFrame,OutError))return false;
	OutLease=MakeShared<FDynamicFrameLease,ESPMode::ThreadSafe>(MoveTemp(Reservation));
	return true;
}
