#include "Bridge/RpcDispatcher.h"

#include "Async/Async.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/NeutralRenderSourceIngress.h"
#include "Bridge/RenderSourceIngress.h"
#include "Bridge/RenderPoseIngress.h"
#include "Bridge/RenderServiceBinding.h"
#include "HAL/PlatformTime.h"
#include "Bridge/RetainedCompletion.h"
#include "Bridge/RenderScenePatchWire.h"
#include "MuJoCo/Render/MjRenderKinematics.h"
#include "Utils/CoordinateConversion.h"
#include "Wire/UrlabWire.gen.h"
#include "Workflows/RenderServerWorldSubsystem.h"
#include "Workflows/RenderScenePatch.h"

namespace
{
using V = FURLabWireValue;

V Map(std::initializer_list<TPair<FString, V>> Fields)
{
	TArray<FString> Keys;
	TArray<V> Values;
	for (const TPair<FString, V>& Field : Fields)
	{
		Keys.Add(Field.Key);
		Values.Add(Field.Value);
	}
	return V::MakeMap(MoveTemp(Keys), MoveTemp(Values));
}

V OwnerStatus(const FURLabRenderOwnerStatus& Status)
{
	return Map({
		{TEXT("available"), V::MakeBool(Status.bAvailable)},
		{TEXT("active"), V::MakeBool(Status.bActive)},
		{TEXT("stale"), V::MakeBool(Status.bStale)},
		{TEXT("retiring"), V::MakeBool(Status.bRetiring)},
		{TEXT("generation"), V::MakeInt(static_cast<int64>(Status.AttachmentGeneration))},
		{TEXT("seconds_remaining"), V::MakeFloat(Status.SecondsRemaining)}});
}

const TCHAR* OwnerError(EURLabRenderOwnerAuthorityError Error)
{
	return Error == EURLabRenderOwnerAuthorityError::Unavailable
		|| Error == EURLabRenderOwnerAuthorityError::ShuttingDown ? TEXT("service_unavailable")
		: Error == EURLabRenderOwnerAuthorityError::Occupied ? TEXT("owner_unavailable")
		: Error == EURLabRenderOwnerAuthorityError::InvalidTtl ? TEXT("bad_request")
		: TEXT("owner_invalid");
}

bool ParseBlob(const V& Value, FURLabRenderSourceBlobRef& Out)
{
	const V* Blob = Value.FindMapValue(TEXT("blob_id"));
	const V* Sha = Value.FindMapValue(TEXT("sha256"));
	if (!Blob || !Sha) return false;
	Out.BlobId = Blob->GetString();
	Out.Sha256 = Sha->GetString();
	return true;
}

bool ParseSource(const V& Value, FURLabRenderSourceRequest& Out, FString& OutError)
{
	const FString Format = Value.FindMapValue(TEXT("format"))->GetString();
	Out.Kind = Format == TEXT("mjb") ? EMjModelSourceKind::Mjb
		: Format == TEXT("mjz") ? EMjModelSourceKind::Mjz
		: EMjModelSourceKind::XmlWithAssets;
	if (!ParseBlob(*Value.FindMapValue(TEXT("primary")), Out.Primary))
	{
		OutError = TEXT("source primary payload reference is invalid");
		return false;
	}
	if (const V* Label = Value.FindMapValue(TEXT("label"))) Out.Label = Label->GetString();
	if (Out.Kind == EMjModelSourceKind::XmlWithAssets)
	{
		for (const V& Entry : Value.FindMapValue(TEXT("assets"))->GetArray())
		{
			FURLabRenderSourceAssetRef& Asset = Out.Assets.AddDefaulted_GetRef();
			Asset.Name = Entry.FindMapValue(TEXT("name"))->GetString();
			if (!ParseBlob(*Entry.FindMapValue(TEXT("payload")), Asset.Payload))
			{
				OutError = TEXT("source asset payload reference is invalid");
				return false;
			}
		}
	}
	return true;
}


const TCHAR* CameraMode(ERenderCameraMode Mode)
{
	switch (Mode)
	{
	case ERenderCameraMode::Real: return TEXT("real");
	case ERenderCameraMode::Depth: return TEXT("depth");
	case ERenderCameraMode::SemanticSegmentation: return TEXT("semantic");
	default: return TEXT("instance");
	}
}
bool ParseCameraMode(const FString& Value, ERenderCameraMode& Out)
{
	if (Value == TEXT("real")) Out = ERenderCameraMode::Real;
	else if (Value == TEXT("depth")) Out = ERenderCameraMode::Depth;
	else if (Value == TEXT("semantic")) Out = ERenderCameraMode::SemanticSegmentation;
	else if (Value == TEXT("instance")) Out = ERenderCameraMode::InstanceSegmentation;
	else return false;
	return true;
}
V OpticsValue(const FRenderCameraOptics& O)
{
	TArray<FString> K{TEXT("resolution"),TEXT("projection"),TEXT("vertical_fov_degrees"),
		TEXT("orthographic_width_m"),TEXT("clip_near_m"),TEXT("clip_far_m")};
	TArray<V> X{Map({{TEXT("width"),V::MakeInt(O.Resolution.X)},{TEXT("height"),V::MakeInt(O.Resolution.Y)}}),
		V::MakeString(O.Projection == ERenderCameraProjection::Perspective ? TEXT("perspective") : TEXT("orthographic")),
		V::MakeFloat(O.VerticalFovDegrees),V::MakeFloat(O.OrthographicWidthCm / 100.0),
		V::MakeFloat(O.NearClipCm / 100.0),V::MakeFloat(O.FarClipCm / 100.0)};
	if (O.bHasIntrinsics) { K.Add(TEXT("intrinsics")); X.Add(Map({{TEXT("fx"),V::MakeFloat(O.FocalPixelsX)},
		{TEXT("fy"),V::MakeFloat(O.FocalPixelsY)},{TEXT("cx"),V::MakeFloat(O.PrincipalPixelsX)},
		{TEXT("cy"),V::MakeFloat(O.PrincipalPixelsY)}})); }
	return V::MakeMap(MoveTemp(K),MoveTemp(X));
}
V FrameValue(const FRenderFrameIdentity& F)
{
	TArray<FString> K{TEXT("scene_revision"), TEXT("frame_id"), TEXT("clock_epoch"),
		TEXT("has_source_time")};
	TArray<V> X{V::MakeString(F.SceneRevision), V::MakeInt(static_cast<int64>(F.FrameId)),
		V::MakeInt(static_cast<int64>(F.ClockEpoch)), V::MakeBool(F.bHasSourceTime)};
	if (F.bHasSourceTime)
	{
		K.Add(TEXT("source_time"));
		X.Add(V::MakeFloat(F.SourceTime));
	}
	return V::MakeMap(MoveTemp(K),MoveTemp(X));
}
V CameraValue(const FRenderCameraEffectiveConfiguration& C)
{
	return Map({{TEXT("name"), V::MakeString(C.CanonicalName)},
		{TEXT("mode"), V::MakeString(CameraMode(C.Mode))},
		{TEXT("optics"), OpticsValue(C.Optics)},
		{TEXT("depth_near_m"),V::MakeFloat(C.DepthNearCm / 100.0)},
		{TEXT("depth_far_m"),V::MakeFloat(C.DepthFarCm / 100.0)},
		{TEXT("baseline_configuration_revision"),
			V::MakeInt(static_cast<int64>(C.BaselineConfigurationRevision))}});
}
V DescriptorValue(const FURLabStoredPayloadDescriptor& D)
{
	const bool bShm = D.BackingKind == EURLabPayloadBackingKind::ShmSealedFile;
	TArray<FString> K{TEXT("transfer"), TEXT("blob_id"), TEXT("boot_id"),
		TEXT("session_id"), TEXT("byte_length"), TEXT("sha256"), TEXT("expires_in_s")};
	TArray<V> X{V::MakeString(bShm ? TEXT("shm") : TEXT("network")),
		V::MakeString(D.BlobId), V::MakeString(D.BootId), V::MakeString(D.SessionId),
		V::MakeInt(D.ByteLength), V::MakeString(D.Sha256), V::MakeFloat(D.ExpiresInSeconds)};
	if (bShm)
	{
		K.Add(TEXT("region_id")); X.Add(V::MakeString(D.RegionId));
		K.Add(TEXT("region_generation"));
		X.Add(V::MakeInt(static_cast<int64>(D.RegionGeneration)));
		K.Add(TEXT("object_name")); X.Add(V::MakeString(D.ObjectName));
	}
	return V::MakeMap(MoveTemp(K),MoveTemp(X));
}
bool ParseResolution(const V& Value, FIntPoint& Out, FString& Error)
{
	const int64 W=Value.FindMapValue(TEXT("width"))->GetInt(), H=Value.FindMapValue(TEXT("height"))->GetInt();
	if (W<=0 || H<=0 || W>MAX_int32 || H>MAX_int32) { Error=TEXT("camera resolution is invalid"); return false; }
	Out=FIntPoint(static_cast<int32>(W),static_cast<int32>(H)); return true;
}
bool ParseSelections(const V& Value, TArray<FRenderCameraCaptureSelection>& Out, FString& Error)
{
	for (const V& Item : Value.GetArray())
	{
		FRenderCameraCaptureSelection& S=Out.AddDefaulted_GetRef();
		S.CameraName=Item.FindMapValue(TEXT("name"))->GetString();
		const V* O=Item.FindMapValue(TEXT("override")); if (!O) continue;
		FRenderCameraCaptureOverride R;
		if (const V* M=O->FindMapValue(TEXT("mode"))) { ERenderCameraMode Mode; if(!ParseCameraMode(M->GetString(),Mode)){Error=TEXT("camera override mode is invalid");return false;} R.Mode=Mode; }
		const V* Resolution=O->FindMapValue(TEXT("resolution")); const V* Optics=O->FindMapValue(TEXT("optics"));
		if (Resolution && Optics) { Error=TEXT("camera override resolution and optics are mutually exclusive"); return false; }
		if (Resolution) { FIntPoint P; if(!ParseResolution(*Resolution,P,Error))return false; R.Resolution=P; }
		if (Optics)
		{
			FRenderCameraOptics X; if(!ParseResolution(*Optics->FindMapValue(TEXT("resolution")),X.Resolution,Error))return false;
			const FString Projection=Optics->FindMapValue(TEXT("projection"))->GetString();
			if(Projection==TEXT("perspective")) X.Projection=ERenderCameraProjection::Perspective;
			else if(Projection==TEXT("orthographic")) X.Projection=ERenderCameraProjection::Orthographic;
			else {Error=TEXT("camera projection is invalid");return false;}
			X.VerticalFovDegrees=Optics->FindMapValue(TEXT("vertical_fov_degrees"))->GetFloat();
			X.OrthographicWidthCm=Optics->FindMapValue(TEXT("orthographic_width_m"))->GetFloat()*100.0f;
			X.NearClipCm=Optics->FindMapValue(TEXT("clip_near_m"))->GetFloat()*100.0f;
			X.FarClipCm=Optics->FindMapValue(TEXT("clip_far_m"))->GetFloat()*100.0f;
			if(const V* I=Optics->FindMapValue(TEXT("intrinsics"))) { X.bHasIntrinsics=true; X.FocalPixelsX=I->FindMapValue(TEXT("fx"))->GetFloat(); X.FocalPixelsY=I->FindMapValue(TEXT("fy"))->GetFloat(); X.PrincipalPixelsX=I->FindMapValue(TEXT("cx"))->GetFloat(); X.PrincipalPixelsY=I->FindMapValue(TEXT("cy"))->GetFloat(); }
			if (X.NearClipCm <= 0.0f || X.FarClipCm <= X.NearClipCm
				|| (X.Projection == ERenderCameraProjection::Perspective
					&& (X.VerticalFovDegrees <= 0.0f || X.OrthographicWidthCm != 0.0f))
				|| (X.Projection == ERenderCameraProjection::Orthographic
					&& (X.OrthographicWidthCm <= 0.0f || X.VerticalFovDegrees != 0.0f || X.bHasIntrinsics)))
			{ Error=TEXT("camera optics contain contradictory or invalid projection fields"); return false; }
			R.CompleteOptics=X;
		}
		const V* Near=O->FindMapValue(TEXT("depth_near_m")); const V* Far=O->FindMapValue(TEXT("depth_far_m"));
		if ((Near!=nullptr)!=(Far!=nullptr)) { Error=TEXT("depth range override requires both near and far"); return false; }
		if(Near){R.DepthNearCm=Near->GetFloat()*100.0f;R.DepthFarCm=Far->GetFloat()*100.0f;
			if(R.DepthNearCm.GetValue()<=0.0f || R.DepthFarCm.GetValue()<=R.DepthNearCm.GetValue()){Error=TEXT("camera depth range is invalid");return false;}}
		S.Override=MoveTemp(R);
	}
	if(Out.IsEmpty()){Error=TEXT("capture cameras must be nonempty");return false;} return true;
}
bool ParseGeomPoseOverrides(const V* Value, int32 InstalledGeomCount,
	const TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
	const FURLabPayloadOwner& Owner, FRenderCameraCaptureLifetimeLeasePtr& OutLease,
	TArray<FRenderGeomPoseOverride>& Out, FString& Error)
{
	const int32 Count = Value ? Value->GetArray().Num() : 0;
	if (InstalledGeomCount < 0 || Count > InstalledGeomCount)
	{
		Error = TEXT("geometry override count exceeds the installed geometry count"); return false;
	}
	if (!FURLabRenderPoseIngress::ReserveGeomPoseOverrides(
			Store, Owner, Count, OutLease, Error)) return false;
	Out.Reserve(Count);
	if (!Value) return true;
	for (const V& Item : Value->GetArray())
	{
		const int64 RawIndex = Item.FindMapValue(TEXT("geom_index"))->GetInt();
		const TArray<V>& P = Item.FindMapValue(TEXT("position_m"))->GetArray();
		const TArray<V>& Q = Item.FindMapValue(TEXT("quaternion_wxyz"))->GetArray();
		if (RawIndex < 0 || RawIndex >= InstalledGeomCount || P.Num() != 3 || Q.Num() != 4)
		{
			Error = TEXT("geometry override index or rigid-pose shape is invalid"); return false;
		}
		double Position[3] = {P[0].GetFloat(), P[1].GetFloat(), P[2].GetFloat()};
		double Quaternion[4] = {Q[0].GetFloat(), Q[1].GetFloat(), Q[2].GetFloat(), Q[3].GetFloat()};
		double Norm2 = 0.0;
		for (double Scalar : Position) if (!FMath::IsFinite(Scalar))
		{
			Error = TEXT("geometry override position contains a non-finite value"); return false;
		}
		for (double Scalar : Quaternion)
		{
			if (!FMath::IsFinite(Scalar)) { Error = TEXT("geometry override quaternion contains a non-finite value"); return false; }
			Norm2 += Scalar * Scalar;
		}
		if (!FMath::IsNearlyEqual(Norm2, 1.0,
			FURLabRenderPoseIngress::QuaternionNormTolerance))
		{
			Error = TEXT("geometry override quaternion is not near unit length"); return false;
		}
		const FVector Translation =
			URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(Position);
		FQuat Rotation =
			URLabCoordinateConversion::RightHandedWxyzQuaternionToUnreal(Quaternion);
		Rotation.Normalize();
		if (Translation.ContainsNaN() || Rotation.ContainsNaN())
		{
			Error = TEXT("geometry override overflows Unreal representation"); return false;
		}
		FRenderGeomPoseOverride& Override = Out.AddDefaulted_GetRef();
		Override.GeomIndex = static_cast<int32>(RawIndex);
		Override.BodyLocalTransform = FTransform(Rotation, Translation, FVector::OneVector);
	}
	return true;
}

bool ReadFiniteArray(const V& Value, int32 Count, TArray<double>& Out)
{
	if (Value.GetType() != EURLabWireValueType::Array || Value.GetArray().Num() != Count)
		return false;
	Out.Reset(Count);
	for (const V& Item : Value.GetArray())
	{
		const double Number = Item.GetFloat();
		if (!FMath::IsFinite(Number)) return false;
		Out.Add(Number);
	}
	return true;
}

bool ReadPatchColor(const V& Value, FLinearColor& Out)
{
	TArray<double> A;
	if (!ReadFiniteArray(Value, 4, A)) return false;
	Out = FLinearColor(A[0], A[1], A[2], A[3]);
	return true;
}

bool ParsePatchOptics(const V& Value, FRenderCameraOptics& Out, FString& Error)
{
	const V* Resolution = Value.FindMapValue(TEXT("resolution"));
	if (!Resolution || !ParseResolution(*Resolution, Out.Resolution, Error)) return false;
	const V* Projection = Value.FindMapValue(TEXT("projection"));
	if (!Projection) { Error = TEXT("camera optics projection is required"); return false; }
	if (Projection->GetString() == TEXT("perspective")) Out.Projection = ERenderCameraProjection::Perspective;
	else if (Projection->GetString() == TEXT("orthographic")) Out.Projection = ERenderCameraProjection::Orthographic;
	else { Error = TEXT("camera optics projection is invalid"); return false; }
	Out.VerticalFovDegrees = Value.FindMapValue(TEXT("vertical_fov_degrees"))->GetFloat();
	Out.OrthographicWidthCm = Value.FindMapValue(TEXT("orthographic_width_m"))->GetFloat() * 100.0;
	Out.NearClipCm = Value.FindMapValue(TEXT("clip_near_m"))->GetFloat() * 100.0;
	Out.FarClipCm = Value.FindMapValue(TEXT("clip_far_m"))->GetFloat() * 100.0;
	if (const V* Intrinsics = Value.FindMapValue(TEXT("intrinsics")))
	{
		Out.bHasIntrinsics = true;
		Out.FocalPixelsX = Intrinsics->FindMapValue(TEXT("fx"))->GetFloat();
		Out.FocalPixelsY = Intrinsics->FindMapValue(TEXT("fy"))->GetFloat();
		Out.PrincipalPixelsX = Intrinsics->FindMapValue(TEXT("cx"))->GetFloat();
		Out.PrincipalPixelsY = Intrinsics->FindMapValue(TEXT("cy"))->GetFloat();
	}
	if (!FMath::IsFinite(Out.VerticalFovDegrees) || !FMath::IsFinite(Out.OrthographicWidthCm)
		|| !FMath::IsFinite(Out.NearClipCm) || !FMath::IsFinite(Out.FarClipCm)
		|| Out.NearClipCm <= 0.0f || Out.FarClipCm <= Out.NearClipCm
		|| (Out.Projection == ERenderCameraProjection::Perspective
			&& (Out.VerticalFovDegrees <= 0.0f || Out.OrthographicWidthCm != 0.0f))
		|| (Out.Projection == ERenderCameraProjection::Orthographic
			&& (Out.OrthographicWidthCm <= 0.0f || Out.VerticalFovDegrees != 0.0f
				|| Out.bHasIntrinsics)))
	{
		Error = TEXT("camera optics contain contradictory or invalid fields");
		return false;
	}
	return true;
}

bool ParseScenePatch(const V& Value, FURLabRenderScenePatch& Out, FString& Error)
{
	Error.Reset();
	if (const V* Entries = Value.FindMapValue(TEXT("materials")))
	for (const V& Item : Entries->GetArray())
	{
		FURLabRenderMaterialPatch& Patch = Out.Materials.AddDefaulted_GetRef();
		const int64 Index = Item.FindMapValue(TEXT("material_index"))->GetInt();
		if (Index < 0 || Index > MAX_int32) { Error = TEXT("material index is outside supported range"); return false; }
		Patch.Index = static_cast<int32>(Index);
		if (const V* X=Item.FindMapValue(TEXT("rgba"))) { FLinearColor C; if(!ReadPatchColor(*X,C)){Error=TEXT("material rgba must contain four finite values");return false;} Patch.Rgba=C; }
		if (const V* X=Item.FindMapValue(TEXT("emission"))) Patch.Emission=X->GetFloat();
		if (const V* X=Item.FindMapValue(TEXT("specular"))) Patch.Specular=X->GetFloat();
		if (const V* X=Item.FindMapValue(TEXT("shininess"))) Patch.Shininess=X->GetFloat();
		if (const V* X=Item.FindMapValue(TEXT("reflectance"))) Patch.Reflectance=X->GetFloat();
		if (const V* X=Item.FindMapValue(TEXT("metallic"))) Patch.Metallic=X->GetFloat();
		if (const V* X=Item.FindMapValue(TEXT("roughness"))) Patch.Roughness=X->GetFloat();
		if (const V* X=Item.FindMapValue(TEXT("texrepeat"))) { TArray<double>A; if(!ReadFiniteArray(*X,2,A)){Error=TEXT("texrepeat must contain two finite values");return false;} Patch.TexRepeat=FVector2D(A[0],A[1]); }
		if (const V* X=Item.FindMapValue(TEXT("texuniform"))) Patch.bTexUniform=X->GetBool();
	}
	if (const V* Entries = Value.FindMapValue(TEXT("geom_surfaces")))
	for (const V& Item : Entries->GetArray())
	{
		FURLabRenderGeomSurfacePatch& Patch = Out.GeomSurfaces.AddDefaulted_GetRef();
		const int64 Index=Item.FindMapValue(TEXT("geom_index"))->GetInt();
		if(Index<0||Index>MAX_int32){Error=TEXT("geometry surface index is outside supported range");return false;} Patch.Index=static_cast<int32>(Index);
		if(const V* X=Item.FindMapValue(TEXT("material_index"))){const int64 M=X->GetInt();if(M<0||M>MAX_int32){Error=TEXT("material assignment index is outside supported range");return false;}Patch.MaterialIndex=static_cast<int32>(M);}
		if(const V* X=Item.FindMapValue(TEXT("rgba"))){FLinearColor C;if(!ReadPatchColor(*X,C)){Error=TEXT("surface rgba must contain four finite values");return false;}Patch.Rgba=C;}
		if(const V* X=Item.FindMapValue(TEXT("texuniform_scale"))){TArray<double>A;if(!ReadFiniteArray(*X,2,A)){Error=TEXT("texuniform_scale must contain two finite values");return false;}Patch.TexUniformScale=FVector2D(A[0],A[1]);}
	}
	if (const V* Entries = Value.FindMapValue(TEXT("lights")))
	for (const V& Item : Entries->GetArray())
	{
		FURLabRenderLightPatch& Patch=Out.Lights.AddDefaulted_GetRef(); Patch.Name=Item.FindMapValue(TEXT("name"))->GetString();
		if(const V* X=Item.FindMapValue(TEXT("enabled")))Patch.bEnabled=X->GetBool();
		if(const V* X=Item.FindMapValue(TEXT("cast_shadows")))Patch.bCastShadows=X->GetBool();
		if(const V* X=Item.FindMapValue(TEXT("color_rgba"))){FLinearColor C;if(!ReadPatchColor(*X,C)){Error=TEXT("light color_rgba must contain four finite values");return false;}Patch.Color=C;}
		if(const V* X=Item.FindMapValue(TEXT("intensity")))Patch.Intensity=X->GetFloat();
		if(const V* X=Item.FindMapValue(TEXT("range_m")))Patch.RangeCm=X->GetFloat()*100.0;
		if(const V* X=Item.FindMapValue(TEXT("source_radius_m")))Patch.SourceRadiusCm=X->GetFloat()*100.0;
		if(const V* X=Item.FindMapValue(TEXT("inner_cone_degrees")))Patch.InnerConeDegrees=X->GetFloat();
		if(const V* X=Item.FindMapValue(TEXT("outer_cone_degrees")))Patch.OuterConeDegrees=X->GetFloat();
	}
	if (const V* Entries = Value.FindMapValue(TEXT("cameras")))
	for (const V& Item : Entries->GetArray())
	{
		FURLabRenderCameraPatch& Patch=Out.Cameras.AddDefaulted_GetRef(); Patch.Name=Item.FindMapValue(TEXT("name"))->GetString();
		if(const V* X=Item.FindMapValue(TEXT("mode"))){ERenderCameraMode Mode;if(!ParseCameraMode(X->GetString(),Mode)){Error=TEXT("camera mode is invalid");return false;}Patch.Mode=Mode;}
		if(const V* X=Item.FindMapValue(TEXT("optics"))){FRenderCameraOptics Optics;if(!ParsePatchOptics(*X,Optics,Error))return false;Patch.Optics=Optics;}
		if(const V* X=Item.FindMapValue(TEXT("depth_near_m")))Patch.DepthNearCm=X->GetFloat()*100.0;
		if(const V* X=Item.FindMapValue(TEXT("depth_far_m")))Patch.DepthFarCm=X->GetFloat()*100.0;
	}
	if (const V* Entries = Value.FindMapValue(TEXT("geom_poses")))
	for (const V& Item : Entries->GetArray())
	{
		FURLabRenderGeomPosePatch& Patch=Out.GeomPoses.AddDefaulted_GetRef();
		const int64 Index=Item.FindMapValue(TEXT("geom_index"))->GetInt();
		const TArray<V>& P=Item.FindMapValue(TEXT("position_m"))->GetArray(); const TArray<V>& Q=Item.FindMapValue(TEXT("quaternion_wxyz"))->GetArray();
		if(Index<0||Index>MAX_int32||P.Num()!=3||Q.Num()!=4){Error=TEXT("geometry pose shape or index is invalid");return false;}
		double Pos[3]={P[0].GetFloat(),P[1].GetFloat(),P[2].GetFloat()}, Quat[4]={Q[0].GetFloat(),Q[1].GetFloat(),Q[2].GetFloat(),Q[3].GetFloat()};
		double Norm2=0; for(double N:Pos)if(!FMath::IsFinite(N)){Error=TEXT("geometry position is not finite");return false;}
		for(double N:Quat){if(!FMath::IsFinite(N)){Error=TEXT("geometry quaternion is not finite");return false;}Norm2+=N*N;}
		if(!FMath::IsNearlyEqual(Norm2,1.0,FURLabRenderPoseIngress::QuaternionNormTolerance)){Error=TEXT("geometry quaternion must be near unit length");return false;}
		const FVector Translation=URLabCoordinateConversion::RightHandedMetresPositionToUnrealCentimetres(Pos);
		FQuat Rotation=URLabCoordinateConversion::RightHandedWxyzQuaternionToUnreal(Quat); Rotation.Normalize();
		if(Translation.ContainsNaN()||Rotation.ContainsNaN()){Error=TEXT("geometry transform overflows Unreal representation");return false;}
		Patch.Index=static_cast<int32>(Index);Patch.BodyLocalTransform=FTransform(Rotation,Translation,FVector::OneVector);
	}
	if (const V* Env=Value.FindMapValue(TEXT("environment")))
	{
		FURLabRenderEnvironmentPatch& Patch=Out.Environment.Emplace();
		if(const V* X=Env->FindMapValue(TEXT("ambient_color_rgba"))){FLinearColor C;if(!ReadPatchColor(*X,C)){Error=TEXT("ambient color must contain four finite values");return false;}Patch.AmbientColor=C;}
		if(const V* X=Env->FindMapValue(TEXT("ambient_intensity")))Patch.AmbientIntensity=X->GetFloat();
		if(const V* X=Env->FindMapValue(TEXT("sky_top_rgba"))){FLinearColor C;if(!ReadPatchColor(*X,C)){Error=TEXT("sky top color must contain four finite values");return false;}Patch.SkyTop=C;}
		if(const V* X=Env->FindMapValue(TEXT("sky_bottom_rgba"))){FLinearColor C;if(!ReadPatchColor(*X,C)){Error=TEXT("sky bottom color must contain four finite values");return false;}Patch.SkyBottom=C;}
		if(const V* X=Env->FindMapValue(TEXT("skybox_enabled")))Patch.bSkyboxEnabled=X->GetBool();
		if(const V* X=Env->FindMapValue(TEXT("headlight_enabled")))Patch.bHeadlightEnabled=X->GetBool();
		if(const V* X=Env->FindMapValue(TEXT("headlight_color_rgba"))){FLinearColor C;if(!ReadPatchColor(*X,C)){Error=TEXT("headlight color must contain four finite values");return false;}Patch.HeadlightColor=C;}
		if(const V* X=Env->FindMapValue(TEXT("headlight_intensity")))Patch.HeadlightIntensity=X->GetFloat();
	}
	if (const V* Features=Value.FindMapValue(TEXT("visualization_features")))
	{
		TArray<FString>& OutFeatures=Out.VisualizationFeatures.Emplace();
		for(const V& Feature:Features->GetArray())OutFeatures.Add(Feature.GetString());
	}
	return true;
}

const TCHAR* StateValue(EURLabRenderCaptureJobState S)
{
	switch (S)
	{
	case EURLabRenderCaptureJobState::Queued: return TEXT("queued");
	case EURLabRenderCaptureJobState::Running: return TEXT("running");
	case EURLabRenderCaptureJobState::Succeeded: return TEXT("succeeded");
	case EURLabRenderCaptureJobState::Failed: return TEXT("failed");
	default: return TEXT("cancelled");
	}
}

void ExecuteTypedRender(
	const TSharedRef<FURLabRetainedCompletion, ESPMode::ThreadSafe>& Completion,
	const TSharedPtr<FURLabRenderServiceBinding, ESPMode::ThreadSafe>& Binding,
	const FURLabRenderOwnerSessionScopePtr& Scope,
	const FURLabDataSessionPtr& DataSession,
	const TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe>& PayloadStore,
	const TWeakObjectPtr<UURLabBridgeServer>& Bridge)
{
	check(IsInGameThread());
	UURLabRenderServerWorldSubsystem* Workflow = Binding ? Binding->ResolveWorkflow() : nullptr;
	if (!Workflow || !Scope || !Scope->IsActive())
	{
		Completion->Error(TEXT("service_unavailable"), TEXT("Render service is unavailable"));
		return;
	}
	const FString& Op = Completion->Request.Op;
	const bool bInstall = Op == TEXT("render_source_install");
	const bool bScenePatch = Op == TEXT("render_scene_patch");
	const EMjCapability Permission = (bInstall || bScenePatch)
		? EMjCapability::AcceptModelLoad : EMjCapability::CameraRpc;
	if (Op != TEXT("render_owner_status")
		&& (!Bridge.IsValid() || !Bridge->GrantsPermission(Permission)))
	{
		Completion->Error(TEXT("capability_disabled"),
			TEXT("This instance does not grant the operation's permission"));
		return;
	}
	if (Op == TEXT("render_owner_status"))
	{
		const FURLabRenderOwnerStatusResult Result = Workflow->OwnerStatus();
		if (!Result.IsOk()) { Completion->Error(OwnerError(Result.Error), Result.Message); return; }
		Completion->Publish(FURLabReplyEnvelope::MakeResult(
			Completion->Request, Map({{TEXT("status"), OwnerStatus(Result.Status)}})));
		return;
	}
	if (Op == TEXT("render_owner_attach"))
	{
		const FURLabRenderOwnerAttachResult Result = Workflow->AttachOwner(
			Scope, Completion->Request.Body.FindMapValue(TEXT("ttl_s"))->GetFloat());
		if (!Result.IsOk()) { Completion->Error(OwnerError(Result.Error), Result.Message); return; }
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, Map({
			{TEXT("owner"), Map({
				{TEXT("generation"), V::MakeInt(static_cast<int64>(Result.Proof.GetGeneration()))},
				{TEXT("token"), V::MakeString(Result.Proof.GetOpaqueToken())}})},
			{TEXT("status"), OwnerStatus(Result.Status)}})));
		return;
	}
	const uint64 Generation = static_cast<uint64>(
		Completion->Request.Body.FindMapValue(TEXT("generation"))->GetInt());
	const FString Token = Completion->Request.Body.FindMapValue(TEXT("owner_token"))->GetString();
	if (Op == TEXT("render_owner_release"))
	{
		const FURLabRenderOwnerLeaseResult Result =
			Workflow->ReleaseOwnerCredentials(Scope, Generation, Token);
		if (!Result.IsOk()) { Completion->Error(OwnerError(Result.Error), Result.Message); return; }
		Binding->RetireAttachment(Scope, Generation, Token);
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, Map({
			{TEXT("changed"), V::MakeBool(Result.bChanged)},
			{TEXT("status"), OwnerStatus(Result.Status)}})));
		return;
	}
	const FURLabRenderOwnerAuthorizationResult Authorized =
		Workflow->AuthorizeOwner(Scope, Generation, Token);
	if (!Authorized.IsOk())
	{
		Completion->Error(OwnerError(Authorized.Error), Authorized.Message);
		return;
	}
	if (Op == TEXT("render_owner_renew"))
	{
		const FURLabRenderOwnerLeaseResult Result = Workflow->RenewOwner(Authorized.Proof,
			Completion->Request.Body.FindMapValue(TEXT("ttl_s"))->GetFloat());
		if (!Result.IsOk()) { Completion->Error(OwnerError(Result.Error), Result.Message); return; }
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, Map({
			{TEXT("changed"), V::MakeBool(Result.bChanged)},
			{TEXT("status"), OwnerStatus(Result.Status)}})));
		return;
	}
	if (Op == TEXT("render_scene_describe"))
	{
		FURLabRenderSourceDescription D;
		FString Error;
		if (!Workflow->DescribeSource(Authorized.Proof, D, Error))
		{
			Completion->Error(TEXT("source_invalid"), Error);
			return;
		}
		if (D.SceneRevision != Completion->Request.Body.FindMapValue(TEXT("expected_revision"))->GetString())
		{
			Completion->Error(TEXT("revision_conflict"), TEXT("installed scene revision changed"));
			return;
		}
		TArray<V> Bodies;
		for (const FString& Name : D.BodyNames) Bodies.Add(V::MakeString(Name));
		TArray<V> Cameras;
		for (const auto& Camera : D.ModelCameras) Cameras.Add(CameraValue(Camera));
		TArray<V> Lights;
		for (const FString& Name : D.LightNames) Lights.Add(V::MakeString(Name));
		TArray<FString> K{TEXT("scene_revision"), TEXT("scene_generation"), TEXT("body_names"),
			TEXT("model_cameras"), TEXT("light_names"), TEXT("supports_qpos"), TEXT("qpos_width")};
		TArray<V> X{V::MakeString(D.SceneRevision),
			V::MakeInt(static_cast<int64>(D.SceneGeneration)), V::MakeArray(MoveTemp(Bodies)),
			V::MakeArray(MoveTemp(Cameras)), V::MakeArray(MoveTemp(Lights)),
			V::MakeBool(D.bSupportsQpos), V::MakeInt(D.QposWidth)};
		if (D.UserCamera)
		{
			K.Add(TEXT("user_camera"));
			X.Add(CameraValue(*D.UserCamera));
		}
		Completion->Publish(FURLabReplyEnvelope::MakeResult(
			Completion->Request, V::MakeMap(MoveTemp(K), MoveTemp(X))));
		return;
	}
	if (Op == TEXT("render_scene_patch"))
	{
		FURLabRenderScenePatch Patch;
		FString Error;
		if (!URLabRenderScenePatchWire::Parse(*Completion->Request.Body.FindMapValue(TEXT("patch")), Patch, Error))
		{
			Completion->Error(TEXT("patch_invalid"), Error);
			return;
		}
		const FString Expected=Completion->Request.Body.FindMapValue(TEXT("expected_revision"))->GetString();
		const FURLabRenderServerInstallResult Result=Workflow->PatchScene(Authorized.Proof,Expected,Patch);
		if(!Result.bCommitted)
		{
			Completion->Error(Result.Error.Contains(TEXT("revision"))?TEXT("revision_conflict"):TEXT("patch_invalid"),Result.Error);
			return;
		}
		Binding->ResetRetentionContext(Scope,Authorized.Proof);
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request,Map({
			{TEXT("committed"),V::MakeBool(true)},
			{TEXT("scene_revision"),V::MakeString(Result.SceneRevision)},
			{TEXT("scene_generation"),V::MakeInt(static_cast<int64>(Result.SceneGeneration))}})));
		return;
	}
	if (Op == TEXT("render_capture_poll") || Op == TEXT("render_capture_cancel") || Op == TEXT("render_capture_release"))
	{
		const FString Job = Completion->Request.Body.FindMapValue(TEXT("job_id"))->GetString();
		FString Error;
		if (Op == TEXT("render_capture_cancel"))
		{
			bool bChanged = false;
			if (!Binding->CancelCapture(Scope, Authorized.Proof, Job, bChanged, Error))
			{
				Completion->Error(TEXT("capture_invalid"), Error); return;
			}
			Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request,
				Map({{TEXT("cancelled"), V::MakeBool(bChanged)}})));
			return;
		}
		if (Op == TEXT("render_capture_release"))
		{
			if (!Binding->ReleaseCapture(Scope, Authorized.Proof, Job, Error))
			{
				Completion->Error(TEXT("capture_invalid"), Error); return;
			}
			Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request,
				Map({{TEXT("released"), V::MakeBool(true)}})));
			return;
		}
		FURLabRenderServiceCaptureView View;
		if (!Binding->PollCapture(Scope, Authorized.Proof, Job, View, Error))
		{
			Completion->Error(TEXT("capture_invalid"), Error); return;
		}
		V Material = Map({{TEXT("state"), V::MakeString(TEXT("pending"))}});
		if (View.MaterializationState == EURLabRenderMaterializationState::Failed)
			Material = Map({{TEXT("state"), V::MakeString(TEXT("failed"))},
				{TEXT("error"), V::MakeString(View.Materialization.Error)}});
		else if (View.MaterializationState == EURLabRenderMaterializationState::Succeeded)
		{
			TArray<V> Cameras;
			for (const FURLabCapturePayloadCameraMetadata& M : View.Materialization.Cameras)
			{
				Cameras.Add(Map({{TEXT("name"), V::MakeString(M.Name)},
					{TEXT("mode"), V::MakeString(CameraMode(M.Mode))},
					{TEXT("optics"), OpticsValue(M.Optics)},
					{TEXT("depth_near_m"), V::MakeFloat(M.DepthNearCm / 100.0)},
					{TEXT("depth_far_m"), V::MakeFloat(M.DepthFarCm / 100.0)},
					{TEXT("byte_offset"), V::MakeInt(M.ByteOffset)},
					{TEXT("byte_length"), V::MakeInt(M.ByteLength)},
			{TEXT("baseline_configuration_revision"),
				V::MakeInt(static_cast<int64>(M.BaselineConfigurationRevision))},
			{TEXT("configuration_id"), V::MakeString(M.ConfigurationId)},
					{TEXT("capture_serial"), V::MakeInt(static_cast<int64>(M.CaptureSerial))}}));
			}
			Material = Map({{TEXT("state"), V::MakeString(TEXT("succeeded"))},
				{TEXT("payload"), DescriptorValue(View.Materialization.Descriptor)},
				{TEXT("cameras"), V::MakeArray(MoveTemp(Cameras))}});
		}
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, Map({
			{TEXT("state"), V::MakeString(StateValue(View.Native.State))},
			{TEXT("frame"), FrameValue(View.SubmittedFrame)},
			{TEXT("geometry_configuration_id"),
				V::MakeString(View.Native.GeometryConfigurationId)},
			{TEXT("gpu_writers_drained"), V::MakeBool(View.Native.bGpuWritersDrained)},
			{TEXT("materialization"), MoveTemp(Material)}})));
		return;
	}
	if (!DataSession || !DataSession->IsActive())
	{
		Completion->Error(TEXT("outcome_unknown"), TEXT("Payload session is inactive"));
		return;
	}
	if (Op == TEXT("render_capture_history_query"))
	{
		FString Error; FGuid Context;
		if (!Binding->GetRetentionContext(Scope, Authorized.Proof, Context, Error))
		{
			Completion->Error(TEXT("service_unavailable"), Error); return;
		}
		const FString Expected = Completion->Request.Body.FindMapValue(TEXT("expected_revision"))->GetString();
		const V& Selector = *Completion->Request.Body.FindMapValue(TEXT("selector"));
		FURLabRenderHistoryQuery Query; Query.RetentionContext = Context;
		Query.ExpectedSceneRevision = Expected;
		Query.GeometryConfigurationId = Completion->Request.Body.FindMapValue(
			TEXT("geometry_configuration_id"))->GetString();
		Query.TargetSourceTime = Selector.FindMapValue(TEXT("target_time"))->GetFloat();
		Query.ClockEpoch = static_cast<uint64>(Selector.FindMapValue(TEXT("clock_epoch"))->GetInt());
		for (const V& Camera : Completion->Request.Body.FindMapValue(TEXT("cameras"))->GetArray())
			Query.Cameras.Add({Camera.FindMapValue(TEXT("name"))->GetString(),
				Camera.FindMapValue(TEXT("configuration_id"))->GetString()});
		FURLabRenderExactCapturePlan Plan; FRenderFrameIdentity Identity;
		if (!Workflow->ResolveCaptureHistory(Authorized.Proof, Query, Plan, Identity, Error))
		{
			Completion->Error(Error == TEXT("history_miss") ? TEXT("history_miss") : TEXT("capture_invalid"), Error); return;
		}
		const FURLabDataSessionSnapshot Data = DataSession->Snapshot(FPlatformTime::Seconds());
		const EURLabPayloadBackingKind Backing = Data.Transfer == EURLabDataTransfer::Shm
			? EURLabPayloadBackingKind::ShmSealedFile : EURLabPayloadBackingKind::NetworkBytes;
		const FURLabShmExactRegionPtr Region = Data.Transfer == EURLabDataTransfer::Shm
			? DataSession->GetRegion() : nullptr;
		FURLabCapturePayloadAdmissionResult A = FURLabCapturePayloadAdmission::AdmitRetained(
			PayloadStore, Data.Owner, Backing, Region, Plan);
		if (!A.bAccepted) { Completion->Error(TEXT("payload_quota_exceeded"), A.Error); return; }
		FURLabRenderServerCaptureAdmission Native = Workflow->QueryCaptureHistory(
			Authorized.Proof, Query, MoveTemp(A.ResultLease));
		if (!Native.bAccepted) { A.Admission->Abort(); Completion->Error(TEXT("history_miss"), Native.Error); return; }
		FString Job;
		if (!Binding->AddCapture(Scope, Authorized.Proof, MoveTemp(Native.Handle),
			MoveTemp(A.Admission), Job, Error))
		{
			Completion->Error(TEXT("service_unavailable"), Error); return;
		}
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request,
			Map({{TEXT("job_id"), V::MakeString(Job)},
			{TEXT("geometry_configuration_id"), V::MakeString(Plan.GeometryConfigurationId)},
			{TEXT("requested"), Map({{TEXT("domain"), V::MakeString(TEXT("source"))},
				{TEXT("target_time"), V::MakeFloat(Query.TargetSourceTime)},
				{TEXT("clock_epoch"), V::MakeInt(static_cast<int64>(Query.ClockEpoch))}})}})));
		return;
	}
	if (Op == TEXT("render_capture_submit"))
	{
		FString Error;
		FURLabRenderSourceDescription D;
		if (!Binding->CanAcceptCapture(Scope, Error))
		{
			Completion->Error(TEXT("service_unavailable"), Error);
			return;
		}
		if (!Workflow->DescribeSource(Authorized.Proof, D, Error))
		{
			Completion->Error(TEXT("source_invalid"), Error); return;
		}
		const FString Expected = Completion->Request.Body.FindMapValue(TEXT("expected_revision"))->GetString();
		if (D.SceneRevision != Expected)
		{
			Completion->Error(TEXT("revision_conflict"), TEXT("installed scene revision changed")); return;
		}
		FURLabRenderPoseRequest Pose;
		Pose.Identity.SceneRevision = Expected;
		const V& Frame = *Completion->Request.Body.FindMapValue(TEXT("frame"));
		Pose.Identity.FrameId = static_cast<uint64>(Frame.FindMapValue(TEXT("frame_id"))->GetInt());
		Pose.Identity.ClockEpoch = static_cast<uint64>(Frame.FindMapValue(TEXT("clock_epoch"))->GetInt());
		Pose.Identity.bHasSourceTime = Frame.FindMapValue(TEXT("has_source_time"))->GetBool();
		const V* SourceTime = Frame.FindMapValue(TEXT("source_time"));
		if (Pose.Identity.bHasSourceTime != (SourceTime != nullptr))
		{
			Completion->Error(TEXT("bad_request"), TEXT("source_time presence must match has_source_time"));
			return;
		}
		if (SourceTime) Pose.Identity.SourceTime = SourceTime->GetFloat();
		const V& PoseValue = *Completion->Request.Body.FindMapValue(TEXT("pose"));
		Pose.Kind = PoseValue.FindMapValue(TEXT("kind"))->GetString() == TEXT("qpos")
			? EURLabRenderPoseKind::Qpos : EURLabRenderPoseKind::Cartesian;
		if (const V* Blob = PoseValue.FindMapValue(TEXT("payload")))
		{
			FURLabRenderSourceBlobRef Ref;
			if (!ParseBlob(*Blob, Ref))
			{
				Completion->Error(TEXT("bad_request"), TEXT("pose payload reference invalid")); return;
			}
			Pose.Payload = MoveTemp(Ref);
		}
		if (const V* Visualization = PoseValue.FindMapValue(TEXT("visualization")))
			Pose.Visualization = *Visualization;
		TArray<FRenderCameraCaptureSelection> Selections;
		if (!ParseSelections(*Completion->Request.Body.FindMapValue(TEXT("cameras")), Selections, Error))
		{
			Completion->Error(TEXT("bad_request"), Error); return;
		}
		FRenderCameraCaptureLifetimeLeasePtr GeomLease;
		TArray<FRenderGeomPoseOverride> GeomOverrides;
		if (!ParseGeomPoseOverrides(Completion->Request.Body.FindMapValue(TEXT("geom_overrides")),
				D.GeomCount, PayloadStore, DataSession->GetOwner(), GeomLease, GeomOverrides, Error))
		{
			Completion->Error(TEXT("bad_request"), Error); return;
		}
		FURLabRenderExactCapturePlan Plan;
		if (!Workflow->PlanCapture(Authorized.Proof, Expected, Selections,
				GeomOverrides, Plan, Error))
		{
			Completion->Error(TEXT("capture_invalid"), Error); return;
		}
		const FString GeometryConfigurationId = Plan.GeometryConfigurationId;
		if (!Binding->GetRetentionContext(Scope, Authorized.Proof, Plan.RetentionContext, Error))
		{
			Completion->Error(TEXT("service_unavailable"), Error); return;
		}
		const FURLabDataSessionSnapshot Data = DataSession->Snapshot(FPlatformTime::Seconds());
		if (!Data.bActive || Data.Transfer == EURLabDataTransfer::Provisional)
		{
			Completion->Error(TEXT("service_unavailable"), TEXT("data backing is not selected"));
			return;
		}
		const EURLabPayloadBackingKind Backing = Data.Transfer == EURLabDataTransfer::Shm
			? EURLabPayloadBackingKind::ShmSealedFile : EURLabPayloadBackingKind::NetworkBytes;
		const FURLabShmExactRegionPtr Region = Data.Transfer == EURLabDataTransfer::Shm
			? DataSession->GetRegion() : nullptr;
		FURLabCapturePayloadAdmissionResult A = FURLabCapturePayloadAdmission::Admit(
			PayloadStore, Data.Owner, Backing, Region, Plan);
		if (!A.bAccepted)
		{
			Completion->Error(TEXT("payload_quota_exceeded"), A.Error); return;
		}
		FURLabDecodedRenderPose Decoded;
		if (!FURLabRenderPoseIngress::Decode(PayloadStore, Data.Owner, D, Pose, Decoded, Error))
		{
			A.Admission->Abort();
			Completion->Error(TEXT("bad_request"), Error); return;
		}
		const FURLabRenderOwnerAuthorizationResult Reauthorized =
			Workflow->AuthorizeOwner(Scope, Generation, Token);
		if (!Binding->IsActive() || !Scope->IsActive() || !DataSession->IsActive()
			|| !Bridge.IsValid() || !Bridge->GrantsPermission(EMjCapability::CameraRpc)
			|| !Reauthorized.IsOk())
		{
			A.Admission->Abort();
			Completion->Error(TEXT("service_unavailable"),
				TEXT("Render capture authority changed during pose preparation"));
			return;
		}
		auto Life = FURLabRenderPoseIngress::ComposeLifetimeLeases(
			FURLabRenderPoseIngress::ComposeLifetimeLeases(
				MoveTemp(A.PixelLease), MoveTemp(GeomLease)), Decoded.PresentationLease);
		const double Timeout = Completion->Request.Body.FindMapValue(TEXT("timeout_s"))->GetFloat();
		FURLabRenderServerCaptureAdmission Native;
		if (Decoded.Kind == EURLabRenderPoseKind::Qpos)
		{
			Native = Workflow->SubmitQposCapture(Authorized.Proof, Expected, Decoded.Qpos,
				Pose.Identity.FrameId, Pose.Identity.SourceTime, Pose.Identity.bHasSourceTime,
				Pose.Identity.ClockEpoch, MoveTemp(Plan), MoveTemp(Life),
				MoveTemp(A.ResultLease), Timeout);
		}
		else
		{
			Native = Workflow->SubmitCartesianCapture(Authorized.Proof, Expected,
				MoveTemp(Decoded.Cartesian), MoveTemp(Plan), MoveTemp(Life),
				MoveTemp(A.ResultLease), Timeout);
		}
		if (!Native.bAccepted)
		{
			A.Admission->Abort();
			Completion->Error(TEXT("capture_invalid"), Native.Error); return;
		}
		FString Job;
		if (!Binding->AddCapture(Scope, Authorized.Proof, MoveTemp(Native.Handle),
			MoveTemp(A.Admission), Job, Error))
		{
			if (A.Admission) A.Admission->Abort();
			FString CleanupError;
			Workflow->CancelCapture(Authorized.Proof, Native.Handle, CleanupError);
			FURLabRenderCaptureJobSnapshot CleanupSnapshot;
			if (Workflow->PollCapture(Authorized.Proof, Native.Handle, CleanupSnapshot, CleanupError)
				&& CleanupSnapshot.IsTerminal())
				Workflow->ReleaseCapture(Authorized.Proof, Native.Handle, CleanupError);
			Completion->Error(TEXT("service_unavailable"), Error);
			return;
		}
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request,
			Map({{TEXT("job_id"), V::MakeString(Job)},
				{TEXT("frame"), FrameValue(Pose.Identity)},
				{TEXT("geometry_configuration_id"),
					V::MakeString(GeometryConfigurationId)}})));
		return;
	}
	FString Error;
	const V& SourceValue = *Completion->Request.Body.FindMapValue(TEXT("source"));
	const int64 VisibleMask = Completion->Request.Body.FindMapValue(TEXT("options"))
		->FindMapValue(TEXT("visible_group_mask"))->GetInt();
	if (VisibleMask > MAX_int32)
	{
		Completion->Error(TEXT("bad_request"), TEXT("visible_group_mask exceeds int32"));
		return;
	}
	const FString SceneRevision = FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphens).ToLower();
	const FString Expected = Completion->Request.Body.FindMapValue(TEXT("expected_revision"))->GetString();
	FRenderSceneInstallOptions InstallOptions;
	InstallOptions.VisibleGroupMask = static_cast<int32>(VisibleMask);
	const bool bNeutral = SourceValue.FindMapValue(TEXT("format"))->GetString() == TEXT("neutral");
	if (bNeutral)
	{
		FRenderSceneInstallPayload Payload;
		if (!FURLabNeutralRenderSourceIngress::Prepare(PayloadStore,
			DataSession->GetOwner(), SourceValue, SceneRevision, Payload, Error))
		{
			Completion->Error(TEXT("source_invalid"), Error);
			return;
		}
		const FURLabRenderOwnerAuthorizationResult Reauthorized =
			Workflow->AuthorizeOwner(Scope, Generation, Token);
		if (!Binding->IsActive() || !Scope->IsActive() || !DataSession->IsActive()
			|| !Bridge.IsValid() || !Bridge->GrantsPermission(EMjCapability::AcceptModelLoad)
			|| !Reauthorized.IsOk())
		{
			Completion->Error(TEXT("service_unavailable"),
				TEXT("Render source authority changed during preparation"));
			return;
		}
		const FURLabRenderServerInstallResult Result = Workflow->InstallNeutralScene(
			Reauthorized.Proof, Expected, MoveTemp(Payload), InstallOptions);
		if (!Result.bCommitted)
		{
			const bool bRevisionConflict = Result.Error.Contains(TEXT("revision"));
			Completion->Error(bRevisionConflict ? TEXT("revision_conflict") : TEXT("source_invalid"),
				Result.Error);
			return;
		}
		Binding->ResetRetentionContext(Scope, Reauthorized.Proof);
		Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, Map({
			{TEXT("committed"), V::MakeBool(true)},
			{TEXT("scene_revision"), V::MakeString(Result.SceneRevision)},
			{TEXT("scene_generation"), V::MakeInt(static_cast<int64>(Result.SceneGeneration))}})));
		return;
	}
	FURLabRenderSourceRequest Source;
	if (!ParseSource(SourceValue, Source, Error))
	{
		Completion->Error(TEXT("bad_request"), Error);
		return;
	}
	FMjRenderKinematicsOptions KinematicsOptions;
	KinematicsOptions.VisibleGroupMask = static_cast<int32>(VisibleMask);
	FMjRenderKinematicsCandidate Candidate;
	if (!FURLabRenderSourceIngress::Prepare(PayloadStore,
		DataSession->GetOwner(), Source, SceneRevision,
		KinematicsOptions, Candidate, Error))
	{
		Completion->Error(TEXT("source_invalid"), Error);
		return;
	}
	const FURLabRenderOwnerAuthorizationResult Reauthorized =
		Workflow->AuthorizeOwner(Scope, Generation, Token);
	if (!Binding->IsActive() || !Scope->IsActive() || !DataSession->IsActive()
		|| !Bridge.IsValid() || !Bridge->GrantsPermission(EMjCapability::AcceptModelLoad)
		|| !Reauthorized.IsOk())
	{
		Completion->Error(TEXT("service_unavailable"),
			TEXT("Render source authority changed during preparation"));
		return;
	}
	const FURLabRenderServerInstallResult Result = Workflow->InstallKinematicsCandidate(
		Reauthorized.Proof, Expected, MoveTemp(Candidate), InstallOptions);
	if (!Result.bCommitted)
	{
		const bool bRevisionConflict = Result.Error.Contains(TEXT("revision"));
		Completion->Error(bRevisionConflict ? TEXT("revision_conflict") : TEXT("source_invalid"),
			Result.Error);
		return;
	}
	Binding->ResetRetentionContext(Scope, Reauthorized.Proof);
	Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, Map({
		{TEXT("committed"), V::MakeBool(true)},
		{TEXT("scene_revision"), V::MakeString(Result.SceneRevision)},
		{TEXT("scene_generation"), V::MakeInt(static_cast<int64>(Result.SceneGeneration))}})));
}
}

bool URLabRenderScenePatchWire::Parse(const FURLabWireValue& Value,
	FURLabRenderScenePatch& Out, FString& OutError)
{
	return ParseScenePatch(Value, Out, OutError);
}

bool FURLabRpcDispatcher::DispatchTypedRender(const FURLabRequestEnvelope& Request,
	const FURLabRequestDigest& Digest, const FURLabRequestLedger::FSessionRef& Session,
	const FURLabRenderOwnerSessionScopePtr& OwnerScope,
	const FURLabDataSessionPtr& DataSession, TArray<uint8>& OutReplyBytes)
{
	FURLabRequestLedger::FAdmissionResult Admission;
	bool bStartDrain = false;
	const TSharedRef<FURLabRetainedCompletion, ESPMode::ThreadSafe> Completion =
		MakeShared<FURLabRetainedCompletion, ESPMode::ThreadSafe>(
			RequestLedger, Session, Request, Digest);
	{
		FScopeLock Lock(&DispatchMutex);
		const TSharedPtr<FURLabRenderServiceBinding, ESPMode::ThreadSafe> Binding = RenderService;
		Admission = RequestLedger->Admit(Session, static_cast<int64>(Request.RequestId), Digest);
		if (Admission.Status == FURLabRequestLedger::Accepted)
		{
			const TWeakObjectPtr<UURLabBridgeServer> Bridge = OwningBridge;
			FURLabGameThreadCommandQueue::FWork Work;
			const TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe> Store = PayloadStore;
			Work.Execute = [Completion, Binding, OwnerScope, DataSession, Store, Bridge]()
			{
				ExecuteTypedRender(Completion, Binding, OwnerScope, DataSession, Store, Bridge);
			};
			Work.Cancel = [Completion]() { Completion->Error(TEXT("service_unavailable"),
				TEXT("Render command was cancelled during service shutdown")); };
			bStartDrain = RenderCommandQueue->Enqueue(MoveTemp(Work));
		}
	}
	const EURLabRetainedAdmissionReply Reply =
		FURLabRetainedCompletion::EncodeAdmission(Admission, Request, OutReplyBytes);
	if (Reply != EURLabRetainedAdmissionReply::Accepted)
		return Reply == EURLabRetainedAdmissionReply::Replied;
	if (IsInGameThread()) RenderCommandQueue->Drain();
	else if (bStartDrain)
	{
		const TSharedRef<FURLabGameThreadCommandQueue, ESPMode::ThreadSafe> Queue = RenderCommandQueue;
		AsyncTask(ENamedThreads::GameThread, [Queue]() { Queue->Drain(); });
	}
	const FURLabRequestLedger::FAdmissionResult Completed =
		RequestLedger->Admit(Session, static_cast<int64>(Request.RequestId), Digest);
	if (Completed.Terminal == FURLabRequestLedger::Completed && Completed.ReplyBytes)
	{
		OutReplyBytes = *Completed.ReplyBytes;
		return true;
	}
	FURLabRequestEnvelopeCodec::EncodeReply(FURLabReplyEnvelope::MakePending(Request), OutReplyBytes);
	return true;
}
