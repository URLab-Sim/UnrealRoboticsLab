// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Bridge/NeutralRenderSourceIngress.h"
#include "Misc/ByteSwap.h"
#include "Utils/AssetCache.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace URLabNeutralRenderSourceIngressTest
{
using V=FURLabWireValue;
V NeutralMap(std::initializer_list<TPair<FString,V>> Fields){TArray<FString>K;TArray<V>X;for(const auto& F:Fields){K.Add(F.Key);X.Add(F.Value);}return V::MakeMap(MoveTemp(K),MoveTemp(X));}
V Empty(){return V::MakeArray(TArray<V>());}
V Initial(){return NeutralMap({{TEXT("frame_id"),V::MakeInt(0)},{TEXT("clock_epoch"),V::MakeInt(0)},{TEXT("has_source_time"),V::MakeBool(false)}});}
V Source(){return NeutralMap({{TEXT("format"),V::MakeString(TEXT("neutral"))},{TEXT("bodies"),Empty()},{TEXT("geoms"),Empty()},{TEXT("meshes"),Empty()},{TEXT("textures"),Empty()},{TEXT("materials"),Empty()},{TEXT("model_cameras"),Empty()},{TEXT("lights"),Empty()},{TEXT("initial_frame"),Initial()}});}
FURLabPayloadStoreLimits Limits(){FURLabPayloadStoreLimits L;L.MaxBlobBytes=1024*1024;L.MaxChunkBytes=1024;L.MaxSessionBytes=4*1024*1024;L.MaxGlobalBytes=4*1024*1024;L.MaxBlobCount=8;L.MaxSessionBlobCount=8;L.MaxSessionPhysicalObjectCount=8;L.MaxActiveSessions=1;return L;}
bool Owner(FAutomationTestBase& T,FURLabPayloadStore& Store,FURLabPayloadOwner& Out){FURLabPayloadSessionPtr S;const auto R=Store.OpenSession(TEXT("neutral-test"),TEXT("session"),S);if(!T.TestTrue(TEXT("neutral session opens"),R.IsOk()))return false;Out=FURLabPayloadOwner::FromSession(S);return true;}
void F32(TArray<uint8>& B,float X){uint32 U=0;FMemory::Memcpy(&U,&X,4);U=INTEL_ORDER32(U);B.Append(reinterpret_cast<uint8*>(&U),4);}
void F64(TArray<uint8>& B,double X){uint64 U=0;FMemory::Memcpy(&U,&X,8);U=INTEL_ORDER64(U);B.Append(reinterpret_cast<uint8*>(&U),8);}
void U32(TArray<uint8>& B,uint32 X){X=INTEL_ORDER32(X);B.Append(reinterpret_cast<uint8*>(&X),4);}
V View(const FString& Blob,const FString& Sha,int64 O,int64 N){return NeutralMap({{TEXT("payload"),NeutralMap({{TEXT("blob_id"),V::MakeString(Blob)},{TEXT("sha256"),V::MakeString(Sha)}})},{TEXT("byte_offset"),V::MakeInt(O)},{TEXT("byte_length"),V::MakeInt(N)}});}
bool Put(FURLabPayloadStore& S,const FURLabPayloadOwner& O,const TArray<uint8>& B,FString& Blob,FString& Sha){Sha=FURLabAssetCache::Sha256Hex(B);if(!S.BeginWrite(O,B.Num(),Sha,Blob)||!S.WriteChunk(O,Blob,0,B))return false;FURLabStoredPayloadDescriptor D;FURLabPayloadPin P;return S.Commit(O,Blob,D,P).IsOk();}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabNeutralRenderSourceEmptyAndAtomicTest,
	"URLab.NeutralRenderSourceIngress.EmptyAndAtomicFailure",
	EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)

bool FURLabNeutralRenderSourceEmptyAndAtomicTest::RunTest(const FString&)
{
	using namespace URLabNeutralRenderSourceIngressTest;
	auto Store=MakeShared<FURLabPayloadStore,ESPMode::ThreadSafe>(TEXT("neutral-test"),Limits());FURLabPayloadOwner O;if(!Owner(*this,*Store,O))return false;
	FRenderSceneInstallPayload Payload;FString Error;
	TestTrue(TEXT("empty portable scene prepares"),FURLabNeutralRenderSourceIngress::Prepare(Store,O,Source(),TEXT("neutral-rev"),Payload,Error));
	TestEqual(TEXT("server revision owns scene"),Payload.Scene.SceneRevision,FString(TEXT("neutral-rev")));
	TestEqual(TEXT("server revision owns initial frame"),Payload.InitialFrame.Identity.SceneRevision,FString(TEXT("neutral-rev")));
	const FString Before=Payload.Scene.SceneRevision;V Bad=Source();
	V BadGeom=NeutralMap({{TEXT("body_index"),V::MakeInt(0)},{TEXT("name"),V::MakeString(TEXT("bad"))},{TEXT("group"),V::MakeInt(0)},{TEXT("kind"),V::MakeString(TEXT("sphere"))},
		{TEXT("body_local_pose"),NeutralMap({{TEXT("position_m"),V::MakeArray({V::MakeFloat(0),V::MakeFloat(0),V::MakeFloat(0)})},{TEXT("rotation_wxyz"),V::MakeArray({V::MakeFloat(1),V::MakeFloat(0),V::MakeFloat(0),V::MakeFloat(0)})}})},
		{TEXT("size"),V::MakeArray({V::MakeFloat(1)})},{TEXT("surface"),NeutralMap({{TEXT("rgba"),V::MakeArray({V::MakeFloat(1),V::MakeFloat(1),V::MakeFloat(1),V::MakeFloat(1)})},{TEXT("texuniform_scale"),V::MakeArray({V::MakeFloat(0),V::MakeFloat(0)})}})}});
	Bad=NeutralMap({{TEXT("format"),V::MakeString(TEXT("neutral"))},{TEXT("bodies"),Empty()},
		{TEXT("geoms"),V::MakeArray({MoveTemp(BadGeom)})},{TEXT("meshes"),Empty()},
		{TEXT("textures"),Empty()},{TEXT("materials"),Empty()},
		{TEXT("model_cameras"),Empty()},{TEXT("lights"),Empty()},{TEXT("initial_frame"),Initial()}});
	TestFalse(TEXT("invalid body reference rejects"),FURLabNeutralRenderSourceIngress::Prepare(Store,O,Bad,TEXT("replacement"),Payload,Error));
	TestEqual(TEXT("failed replacement preserves output"),Payload.Scene.SceneRevision,Before);
	TestFalse(TEXT("failure explains rejection"),Error.IsEmpty());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabNeutralRenderSourcePackedTest,
	"URLab.NeutralRenderSourceIngress.PackedMeshTextureCamera",
	EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FURLabNeutralRenderSourcePackedTest::RunTest(const FString&)
{
	using namespace URLabNeutralRenderSourceIngressTest;auto Store=MakeShared<FURLabPayloadStore,ESPMode::ThreadSafe>(TEXT("neutral-test"),Limits());FURLabPayloadOwner O;if(!Owner(*this,*Store,O))return false;
	TArray<uint8>B;const int64 P=B.Num();for(float X:{0.f,0.f,0.f,1.f,0.f,0.f,0.f,1.f,0.f})F32(B,X);const int64 N=B.Num();for(int I=0;I<3;++I){F32(B,0);F32(B,0);F32(B,1);}const int64 U=B.Num();for(float X:{0.f,0.f,1.f,0.f,0.f,1.f})F32(B,X);const int64 T=B.Num();U32(B,0);U32(B,1);U32(B,2);const int64 R=B.Num();B.Append({255,0,0,255});const int64 Pose=B.Num();for(double X:{0.,0.,0.,1.,0.,0.,0.,0.,0.,1.,1.,0.,0.,0.,2.,3.,4.,1.,0.,0.,0.})F64(B,X);
	FString Blob,Sha;if(!TestTrue(TEXT("packed source uploads"),Put(*Store,O,B,Blob,Sha)))return false;
	const V IdentityPose=NeutralMap({{TEXT("position_m"),V::MakeArray({V::MakeFloat(0),V::MakeFloat(0),V::MakeFloat(0)})},{TEXT("rotation_wxyz"),V::MakeArray({V::MakeFloat(1),V::MakeFloat(0),V::MakeFloat(0),V::MakeFloat(0)})}});
	V S=NeutralMap({{TEXT("format"),V::MakeString(TEXT("neutral"))},{TEXT("bodies"),V::MakeArray({NeutralMap({{TEXT("name"),V::MakeString(TEXT("body"))}})})},
		{TEXT("geoms"),V::MakeArray({NeutralMap({{TEXT("body_index"),V::MakeInt(0)},{TEXT("name"),V::MakeString(TEXT("mesh"))},{TEXT("group"),V::MakeInt(0)},{TEXT("kind"),V::MakeString(TEXT("mesh"))},{TEXT("body_local_pose"),IdentityPose},{TEXT("size"),Empty()},{TEXT("mesh_index"),V::MakeInt(0)},{TEXT("surface"),NeutralMap({{TEXT("material_index"),V::MakeInt(0)}})}})})},
		{TEXT("meshes"),V::MakeArray({NeutralMap({{TEXT("name"),V::MakeString(TEXT("tri"))},{TEXT("vertex_count"),V::MakeInt(3)},{TEXT("triangle_index_count"),V::MakeInt(3)},{TEXT("positions"),View(Blob,Sha,P,36)},{TEXT("normals"),View(Blob,Sha,N,36)},{TEXT("uvs"),View(Blob,Sha,U,24)},{TEXT("triangles"),View(Blob,Sha,T,12)}})})},
		{TEXT("textures"),V::MakeArray({NeutralMap({{TEXT("name"),V::MakeString(TEXT("red"))},{TEXT("width"),V::MakeInt(1)},{TEXT("height"),V::MakeInt(1)},{TEXT("rgba8"),View(Blob,Sha,R,4)}})})},
		{TEXT("materials"),V::MakeArray({NeutralMap({{TEXT("texture_roles"),NeutralMap({{TEXT("rgb"),V::MakeInt(0)}})}})})},
		{TEXT("model_cameras"),V::MakeArray({NeutralMap({{TEXT("canonical_name"),V::MakeString(TEXT("camera"))}})})},
		{TEXT("lights"),V::MakeArray({NeutralMap({{TEXT("name"),V::MakeString(TEXT("key"))},{TEXT("kind"),V::MakeString(TEXT("point"))},{TEXT("enabled"),V::MakeBool(true)},{TEXT("color_rgb"),V::MakeArray({V::MakeFloat(1),V::MakeFloat(0.5),V::MakeFloat(0.25)})},{TEXT("intensity"),V::MakeFloat(900)},{TEXT("cast_shadows"),V::MakeBool(true)},{TEXT("range_m"),V::MakeFloat(5)},{TEXT("source_radius_m"),V::MakeFloat(0.1)},{TEXT("inner_cone_degrees"),V::MakeFloat(0)},{TEXT("outer_cone_degrees"),V::MakeFloat(0)}})})},
		{TEXT("initial_frame"),NeutralMap({{TEXT("frame_id"),V::MakeInt(1)},{TEXT("clock_epoch"),V::MakeInt(2)},{TEXT("has_source_time"),V::MakeBool(false)},{TEXT("poses"),View(Blob,Sha,Pose,168)},
			{TEXT("visualization"),NeutralMap({{TEXT("debug_meshes"),V::MakeArray({NeutralMap({{TEXT("name"),V::MakeString(TEXT("debug_tri"))},{TEXT("rgba"),V::MakeArray({V::MakeFloat(1),V::MakeFloat(1),V::MakeFloat(1),V::MakeFloat(1)})},{TEXT("positions_cm"),View(Blob,Sha,P,36)},{TEXT("triangles"),View(Blob,Sha,T,12)}})})}})}})}});
	FRenderSceneInstallPayload Out;FString Error;if(!TestTrue(TEXT("packed neutral source prepares"),FURLabNeutralRenderSourceIngress::Prepare(Store,O,S,TEXT("packed-rev"),Out,Error))){AddError(Error);return false;}
	TestEqual(TEXT("UE mesh keeps RH source corner order"),Out.Resources.Meshes[0].Triangles,TArray<int32>({0,1,2}));
	const FRenderMeshResource& Mesh=Out.Resources.Meshes[0];
	const FVector UeFront=-FVector::CrossProduct(Mesh.Positions[1]-Mesh.Positions[0],Mesh.Positions[2]-Mesh.Positions[0]).GetSafeNormal();
	TestTrue(TEXT("independent RH +Z triangle faces +Z in UE"),UeFront.Equals(FVector::UpVector,1.e-5)&&Mesh.Normals[0].Equals(UeFront,1.e-5));
	if (TestEqual(TEXT("one debug mesh decoded"),Out.InitialFrame.DebugMeshes.Num(),1))
		TestEqual(TEXT("UE debug mesh keeps RH source corner order"),Out.InitialFrame.DebugMeshes[0].Triangles,TArray<int32>({0,1,2}));
	TestEqual(TEXT("texture bytes retained"),Out.Resources.Textures[0].Rgba8,TArray<uint8>({255,0,0,255}));TestEqual(TEXT("camera pose converted"),Out.InitialFrame.CameraSceneTransforms.Num(),1);TestEqual(TEXT("light descriptor retained"),Out.Scene.Lights.Num(),1);TestEqual(TEXT("light transform follows dense camera order"),Out.InitialFrame.LightSceneTransforms.Num(),1);TestEqual(TEXT("light position converted to centimetres"),Out.InitialFrame.LightSceneTransforms[0].GetTranslation(),FVector(200,-300,400));return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabNeutralDeformableUVTest,
	"URLab.NeutralRenderSourceIngress.DeformableUVs",
	EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FURLabNeutralDeformableUVTest::RunTest(const FString&)
{
	using namespace URLabNeutralRenderSourceIngressTest;
	auto Store=MakeShared<FURLabPayloadStore,ESPMode::ThreadSafe>(TEXT("neutral-test"),Limits());
	FURLabPayloadOwner OwnerValue;if(!Owner(*this,*Store,OwnerValue))return false;
	TArray<uint8> Bytes;
	const int64 Tri=Bytes.Num();U32(Bytes,0);U32(Bytes,1);U32(Bytes,2);
	const int64 UV=Bytes.Num();for(float X:{0.125f,0.25f,0.875f,0.25f,0.125f,0.75f})F32(Bytes,X);
	const int64 Pos=Bytes.Num();for(float X:{0.f,0.f,0.f,10.f,0.f,0.f,0.f,10.f,0.f})F32(Bytes,X);
	FString Blob,Sha;if(!TestTrue(TEXT("UV fixture uploads"),Put(*Store,OwnerValue,Bytes,Blob,Sha)))return false;
	const V UVView=View(Blob,Sha,UV,24);
	const V Frame=NeutralMap({{TEXT("frame_id"),V::MakeInt(0)},
		{TEXT("clock_epoch"),V::MakeInt(0)},{TEXT("has_source_time"),V::MakeBool(false)},
		{TEXT("visualization"),NeutralMap({{TEXT("deformables"),V::MakeArray({NeutralMap({
			{TEXT("name"),V::MakeString(TEXT("cloth"))},
			{TEXT("vertex_positions_cm"),View(Blob,Sha,Pos,36)}})})}})}});
	auto MakeSource=[&](const V& UVs){return NeutralMap({
		{TEXT("format"),V::MakeString(TEXT("neutral"))},{TEXT("bodies"),Empty()},
		{TEXT("geoms"),Empty()},{TEXT("meshes"),Empty()},{TEXT("textures"),Empty()},
		{TEXT("materials"),Empty()},{TEXT("model_cameras"),Empty()},{TEXT("lights"),Empty()},
		{TEXT("deformable_surfaces"),V::MakeArray({NeutralMap({})})},
		{TEXT("deformables"),V::MakeArray({NeutralMap({
			{TEXT("name"),V::MakeString(TEXT("cloth"))},{TEXT("vertex_count"),V::MakeInt(3)},
			{TEXT("triangles"),View(Blob,Sha,Tri,12)},{TEXT("uvs"),UVs},
			{TEXT("surface_index"),V::MakeInt(0)}})})},{TEXT("initial_frame"),Frame}});};
	FRenderSceneInstallPayload Out;FString Error;
	if(!TestTrue(TEXT("static deformable UV decodes"),
		FURLabNeutralRenderSourceIngress::Prepare(Store,OwnerValue,MakeSource(UVView),TEXT("uv-rev"),Out,Error)))
	{AddError(Error);return false;}
	TestEqual(TEXT("UV count"),Out.Scene.Deformables[0].UVs.Num(),3);
	TestTrue(TEXT("asymmetric UV survives wire"),Out.Scene.Deformables[0].UVs[1].Equals(FVector2D(0.875,0.25)));
	TestFalse(TEXT("wrong UV byte length rejected"),
		FURLabNeutralRenderSourceIngress::Prepare(Store,OwnerValue,
			MakeSource(View(Blob,Sha,UV,16)),TEXT("bad-uv"),Out,Error));
	return !HasAnyErrors();
}
#endif
