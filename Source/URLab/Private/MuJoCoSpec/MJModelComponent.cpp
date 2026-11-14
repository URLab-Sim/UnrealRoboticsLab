#include "MJModelComponent.h"
#include "MJAssetLibrary.h"
#include "MJSceneComponents.h"
#include "MJActorComponents.h"
#include "MJUnitConverter.h"
#include "Utils/URLabLogging.h"
#include "Generated/MJSpecMarshal.h"
#include "GameFramework/Actor.h"
#include <cstring>

UMJModelComponent::UMJModelComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UMJModelComponent::BuildDefaultsInSpec(mjSpec* S) const
{
	TMap<FString, const FMJDefaultClassEntry*> ByName;
	for (const FMJDefaultClassEntry& Entry : DefaultClasses)
	{
		if (!Entry.ClassName.IsEmpty())
		{
			ByName.Add(Entry.ClassName, &Entry);
		}
	}

	TSet<FString> Built;
	const bool bDeg = bCompilerDegree;

	TFunction<mjsDefault*(const FMJDefaultClassEntry&)> EnsureDefault = [&](const FMJDefaultClassEntry& Entry) -> mjsDefault*
	{
		if (Built.Contains(Entry.ClassName))
		{
			return mjs_findDefault(S, TCHAR_TO_UTF8(*Entry.ClassName));
		}
		Built.Add(Entry.ClassName);

		const mjsDefault* ParentDef = nullptr;
		if (!Entry.ParentClassName.IsEmpty())
		{
			if (const FMJDefaultClassEntry** ParentEntry = ByName.Find(Entry.ParentClassName))
			{
				ParentDef = EnsureDefault(**ParentEntry);
			}
			else
			{
				ParentDef = mjs_findDefault(S, TCHAR_TO_UTF8(*Entry.ParentClassName));
			}
		}

		mjsDefault* Def = (Entry.ClassName == TEXT("main"))
			? mjs_getSpecDefault(S)
			: mjs_findDefault(S, TCHAR_TO_UTF8(*Entry.ClassName));
		if (!Def)
		{
			Def = mjs_addDefault(S, TCHAR_TO_UTF8(*Entry.ClassName), ParentDef);
		}
		if (!Def)
		{
			return nullptr;
		}
		FMJSpecMarshal::ToMJSpec(Entry.Geom, Def->geom, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Joint, Def->joint, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Site, Def->site, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Actuator, Def->actuator, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Camera, Def->camera, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Light, Def->light, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Tendon, Def->tendon, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Mesh, Def->mesh, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Texture, Def->texture, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Material, Def->material, bDeg);
		if (Entry.bOverride_GeomPos) { FMJUnitConverter::UEPositionToMJ(Entry.GeomPos, Def->geom->pos); }
		if (Entry.bOverride_GeomQuat) { FMJUnitConverter::UEQuatToMJ(Entry.GeomQuat, Def->geom->quat); }
		if (Entry.bOverride_SitePos) { FMJUnitConverter::UEPositionToMJ(Entry.SitePos, Def->site->pos); }
		if (Entry.bOverride_SiteQuat) { FMJUnitConverter::UEQuatToMJ(Entry.SiteQuat, Def->site->quat); }
		return Def;
	};

	for (const FMJDefaultClassEntry& Entry : DefaultClasses)
	{
		if (!Entry.ClassName.IsEmpty())
		{
			EnsureDefault(Entry);
		}
	}
}

void UMJModelComponent::BuildAssetsInSpec(mjSpec* S) const
{
	if (!S)
	{
		return;
	}

	const bool bDeg = bCompilerDegree;
	auto FindDef = [S](const FString& ClassName) -> const mjsDefault*
	{
		return ClassName.IsEmpty() ? nullptr : mjs_findDefault(S, TCHAR_TO_UTF8(*ClassName));
	};

	for (const FMJTextureAssetEntry& Entry : Textures)
	{
		mjsTexture* Tex = mjs_addTexture(S, FindDef(Entry.DefaultClass));
		if (!Tex) { continue; }
		if (!Entry.Name.IsEmpty())
		{
			mjs_setName(Tex->element, TCHAR_TO_UTF8(*Entry.Name));
		}
		if (!Entry.File.IsEmpty())
		{
			mjs_setString(Tex->file, TCHAR_TO_UTF8(*Entry.File));
		}
		if (!Entry.ContentType.IsEmpty())
		{
			mjs_setString(Tex->content_type, TCHAR_TO_UTF8(*Entry.ContentType));
		}
		for (int32 i = 0; i < Entry.CubeFiles.Num(); ++i)
		{
			mjs_setInStringVec(Tex->cubefiles, i, TCHAR_TO_UTF8(*Entry.CubeFiles[i]));
		}
		FMJSpecMarshal::ToMJSpec(Entry.Spec, Tex, bDeg);
	}

	for (const FMJMaterialAssetEntry& Entry : Materials)
	{
		mjsMaterial* Mat = mjs_addMaterial(S, FindDef(Entry.DefaultClass));
		if (!Mat) { continue; }
		if (!Entry.Name.IsEmpty())
		{
			mjs_setName(Mat->element, TCHAR_TO_UTF8(*Entry.Name));
		}
		for (int32 i = 0; i < Entry.Textures.Num(); ++i)
		{
			if (!Entry.Textures[i].IsEmpty())
			{
				mjs_setInStringVec(Mat->textures, i, TCHAR_TO_UTF8(*Entry.Textures[i]));
			}
		}
		FMJSpecMarshal::ToMJSpec(Entry.Spec, Mat, bDeg);
	}

	for (const FMJMeshAssetEntry& Entry : Meshes)
	{
		mjsMesh* Mesh = mjs_addMesh(S, FindDef(Entry.DefaultClass));
		if (!Mesh) { continue; }
		if (!Entry.Name.IsEmpty())
		{
			mjs_setName(Mesh->element, TCHAR_TO_UTF8(*Entry.Name));
		}
		if (!Entry.File.IsEmpty())
		{
			mjs_setString(Mesh->file, TCHAR_TO_UTF8(*Entry.File));
		}
		if (!Entry.ContentType.IsEmpty())
		{
			mjs_setString(Mesh->content_type, TCHAR_TO_UTF8(*Entry.ContentType));
		}
		if (Entry.Userverts.Num() > 0) { mjs_setFloat(Mesh->uservert, Entry.Userverts.GetData(), Entry.Userverts.Num()); }
		if (Entry.Usernormals.Num() > 0) { mjs_setFloat(Mesh->usernormal, Entry.Usernormals.GetData(), Entry.Usernormals.Num()); }
		if (Entry.Usertexcoords.Num() > 0) { mjs_setFloat(Mesh->usertexcoord, Entry.Usertexcoords.GetData(), Entry.Usertexcoords.Num()); }
		if (Entry.Userfaces.Num() > 0) { mjs_setInt(Mesh->userface, Entry.Userfaces.GetData(), Entry.Userfaces.Num()); }
		if (Entry.Userfacenormals.Num() > 0) { mjs_setInt(Mesh->userfacenormal, Entry.Userfacenormals.GetData(), Entry.Userfacenormals.Num()); }
		if (Entry.Userfacetexcoords.Num() > 0) { mjs_setInt(Mesh->userfacetexcoord, Entry.Userfacetexcoords.GetData(), Entry.Userfacetexcoords.Num()); }
		if (Entry.Usertets.Num() > 0) { mjs_setInt(Mesh->usertet, Entry.Usertets.GetData(), Entry.Usertets.Num()); }
		FMJSpecMarshal::ToMJSpec(Entry.Spec, Mesh, bDeg);
	}

	for (const FMJHFieldAssetEntry& Entry : HFields)
	{
		mjsHField* HF = mjs_addHField(S);
		if (!HF) { continue; }
		if (!Entry.Name.IsEmpty()) { mjs_setName(HF->element, TCHAR_TO_UTF8(*Entry.Name)); }
		if (!Entry.File.IsEmpty()) { mjs_setString(HF->file, TCHAR_TO_UTF8(*Entry.File)); }
		if (!Entry.ContentType.IsEmpty()) { mjs_setString(HF->content_type, TCHAR_TO_UTF8(*Entry.ContentType)); }
		for (int32 i = 0; i < FMath::Min(Entry.Size.Num(), 4); ++i) { HF->size[i] = Entry.Size[i]; }
		HF->nrow = Entry.Nrow;
		HF->ncol = Entry.Ncol;
		if (Entry.Userdata.Num() > 0) { mjs_setFloat(HF->userdata, Entry.Userdata.GetData(), Entry.Userdata.Num()); }
	}
}

void UMJModelComponent::BuildSceneSubtree(mjSpec* S, USceneComponent* Comp, mjsBody* ParentBody, mjsFrame* ParentFrame) const
{
	if (!Comp || !S || !ParentBody)
	{
		return;
	}

	const bool bDeg = bCompilerDegree;
	FTransform RelXform = Comp->GetRelativeTransform();
	if (const UMJSceneComponent* MJComp = Cast<UMJSceneComponent>(Comp))
	{
		if (MJComp->bHasImportedMJQuat && MJComp->GetRelativeRotation().Equals(MJComp->ImportedUERotator, 1e-4))
		{
			RelXform.SetRotation(MJComp->ImportedMJQuat);
		}
	}
	mjsBody* NextBody = ParentBody;
	mjsFrame* NextFrame = ParentFrame;

	auto FindDef = [S](const FString& ClassName) -> const mjsDefault*
	{
		return ClassName.IsEmpty() ? nullptr : mjs_findDefault(S, TCHAR_TO_UTF8(*ClassName));
	};

	if (const UMJBodyComponent* BodyComp = Cast<UMJBodyComponent>(Comp))
	{
		mjsBody* Body = mjs_addBody(ParentBody, FindDef(BodyComp->DefaultClass));
		if (Body)
		{
			mjs_setName(Body->element, TCHAR_TO_UTF8(*BodyComp->GetMJName()));
			if (ParentFrame) { mjs_setFrame(Body->element, ParentFrame); }
			FMJUnitConverter::UETransformToMJ(RelXform, Body->pos, Body->quat);
			FMJSpecMarshal::ToMJSpec(BodyComp->Spec, Body, bDeg);
			NextBody = Body;
			NextFrame = nullptr;
		}
	}
	else if (const UMJFrameComponent* FrameComp = Cast<UMJFrameComponent>(Comp))
	{
		mjsFrame* Frame = mjs_addFrame(ParentBody, ParentFrame);
		if (Frame)
		{
			mjs_setName(Frame->element, TCHAR_TO_UTF8(*FrameComp->GetMJName()));
			FMJUnitConverter::UETransformToMJ(RelXform, Frame->pos, Frame->quat);
			FMJSpecMarshal::ToMJSpec(FrameComp->Spec, Frame, bDeg);
			NextFrame = Frame;
		}
	}
	else if (const UMJGeomComponent* GeomComp = Cast<UMJGeomComponent>(Comp))
	{
		mjsGeom* Geom = mjs_addGeom(ParentBody, FindDef(GeomComp->DefaultClass));
		if (Geom)
		{
			mjs_setName(Geom->element, TCHAR_TO_UTF8(*GeomComp->GetMJName()));
			if (ParentFrame) { mjs_setFrame(Geom->element, ParentFrame); }
			if (!GeomComp->Spec.bUseFromTo)
			{
				FMJUnitConverter::UETransformToMJ(RelXform, Geom->pos, Geom->quat);
			}
			FMJSpecMarshal::ToMJSpec(GeomComp->Spec, Geom, bDeg);
			if (!GeomComp->MaterialRef.IsEmpty())
			{
				mjs_setString(Geom->material, TCHAR_TO_UTF8(*GeomComp->MaterialRef.ResolveName(GetOwner())));
			}
			if (!GeomComp->MeshRef.IsEmpty())
			{
				mjs_setString(Geom->meshname, TCHAR_TO_UTF8(*GeomComp->MeshRef.ResolveName(GetOwner())));
			}
			if (!GeomComp->HFieldName.IsEmpty())
			{
				mjs_setString(Geom->hfieldname, TCHAR_TO_UTF8(*GeomComp->HFieldName));
			}
		}
	}
	else if (const UMJJointComponent* JointComp = Cast<UMJJointComponent>(Comp))
	{
		mjsJoint* Joint = mjs_addJoint(ParentBody, FindDef(JointComp->DefaultClass));
		if (Joint)
		{
			mjs_setName(Joint->element, TCHAR_TO_UTF8(*JointComp->GetMJName()));
			if (ParentFrame) { mjs_setFrame(Joint->element, ParentFrame); }
			FMJUnitConverter::UEPositionToMJ(RelXform.GetLocation(), Joint->pos);
			FMJSpecMarshal::ToMJSpec(JointComp->Spec, Joint, bDeg);
		}
	}
	else if (const UMJSiteComponent* SiteComp = Cast<UMJSiteComponent>(Comp))
	{
		mjsSite* Site = mjs_addSite(ParentBody, FindDef(SiteComp->DefaultClass));
		if (Site)
		{
			mjs_setName(Site->element, TCHAR_TO_UTF8(*SiteComp->GetMJName()));
			if (ParentFrame) { mjs_setFrame(Site->element, ParentFrame); }
			if (!SiteComp->Spec.bUseFromTo)
			{
				FMJUnitConverter::UETransformToMJ(RelXform, Site->pos, Site->quat);
			}
			FMJSpecMarshal::ToMJSpec(SiteComp->Spec, Site, bDeg);
			if (!SiteComp->MaterialRef.IsEmpty())
			{
				mjs_setString(Site->material, TCHAR_TO_UTF8(*SiteComp->MaterialRef.ResolveName(GetOwner())));
			}
			if (!SiteComp->MeshRef.IsEmpty())
			{
				mjs_setString(Site->meshname, TCHAR_TO_UTF8(*SiteComp->MeshRef.ResolveName(GetOwner())));
			}
		}
	}
	else if (const UMJCameraComponent* CamComp = Cast<UMJCameraComponent>(Comp))
	{
		mjsCamera* Cam = mjs_addCamera(ParentBody, FindDef(CamComp->DefaultClass));
		if (Cam)
		{
			mjs_setName(Cam->element, TCHAR_TO_UTF8(*CamComp->GetMJName()));
			if (ParentFrame) { mjs_setFrame(Cam->element, ParentFrame); }
			FMJUnitConverter::UETransformToMJ(RelXform, Cam->pos, Cam->quat);
			FMJSpecMarshal::ToMJSpec(CamComp->Spec, Cam, bDeg);
			if (!CamComp->TargetBodyRef.IsEmpty())
			{
				mjs_setString(Cam->targetbody, TCHAR_TO_UTF8(*CamComp->TargetBodyRef.ResolveName(GetOwner())));
			}
		}
	}
	else if (const UMJLightComponent* LightComp = Cast<UMJLightComponent>(Comp))
	{
		mjsLight* Light = mjs_addLight(ParentBody, FindDef(LightComp->DefaultClass));
		if (Light)
		{
			mjs_setName(Light->element, TCHAR_TO_UTF8(*LightComp->GetMJName()));
			if (ParentFrame) { mjs_setFrame(Light->element, ParentFrame); }
			FMJUnitConverter::UEPositionToMJ(RelXform.GetLocation(), Light->pos);
			FMJUnitConverter::UEDirectionToMJ(RelXform.GetRotation().GetForwardVector(), Light->dir);
			FMJSpecMarshal::ToMJSpec(LightComp->Spec, Light, bDeg);
			if (!LightComp->TargetBodyRef.IsEmpty())
			{
				mjs_setString(Light->targetbody, TCHAR_TO_UTF8(*LightComp->TargetBodyRef.ResolveName(GetOwner())));
			}
			if (!LightComp->TextureRef.IsEmpty())
			{
				mjs_setString(Light->texture, TCHAR_TO_UTF8(*LightComp->TextureRef.ResolveName(GetOwner())));
			}
		}
	}
	else if (const UMJAttachComponent* AttachComp = Cast<UMJAttachComponent>(Comp))
	{
		if (!AttachComp->ChildXml.IsEmpty())
		{
			char ErrBuf[512] = {};
			mjSpec* ChildSpec = mj_parseXMLString(TCHAR_TO_UTF8(*AttachComp->ChildXml), VfsHandle.Get(), ErrBuf, sizeof(ErrBuf));
			if (ChildSpec)
			{
				mjsBody* ChildWorld = mjs_findBody(ChildSpec, "world");
				if (ChildWorld)
				{
					mjs_attach(
						ParentBody->element,
						ChildWorld->element,
						TCHAR_TO_UTF8(*AttachComp->Prefix),
						TCHAR_TO_UTF8(*AttachComp->Suffix));
				}
				mj_deleteSpec(ChildSpec);
			}
		}
	}

	const TArray<USceneComponent*>& Children = Comp->GetAttachChildren();
	for (USceneComponent* Child : Children)
	{
		BuildSceneSubtree(S, Child, NextBody, NextFrame);
	}
}

void UMJModelComponent::BuildNonSpatialComponents(mjSpec* S, const FString& Prefix) const
{
	AActor* Owner = GetOwner();
	if (!Owner || !S)
	{
		return;
	}

	const bool bDeg = bCompilerDegree;
	auto FindDef = [S](const FString& ClassName) -> const mjsDefault*
	{
		return ClassName.IsEmpty() ? nullptr : mjs_findDefault(S, TCHAR_TO_UTF8(*ClassName));
	};
	auto ScopedName = [&Prefix](const FString& Name) -> FString
	{
		return (Prefix.IsEmpty() || Name.IsEmpty()) ? Name : (Prefix + Name);
	};

	TArray<UMJTendonComponent*> Tendons;
	Owner->GetComponents<UMJTendonComponent>(Tendons);
	for (const UMJTendonComponent* TenComp : Tendons)
	{
		mjsTendon* Ten = mjs_addTendon(S, FindDef(TenComp->DefaultClass));
		if (!Ten) { continue; }
		mjs_setName(Ten->element, TCHAR_TO_UTF8(*ScopedName(TenComp->GetMJName())));
		FMJSpecMarshal::ToMJSpec(TenComp->Spec, Ten, bDeg);
		if (!TenComp->MaterialRef.IsEmpty())
		{
			mjs_setString(Ten->material, TCHAR_TO_UTF8(*TenComp->MaterialRef.ResolveName(Owner)));
		}
		for (const FMJTendonWrapEntry& Wrap : TenComp->Wraps)
		{
			const FString TargetName = ScopedName(Wrap.TargetRef.ResolveName(Owner));
			switch (Wrap.WrapType)
			{
			case EMJWrap::Site:
				mjs_wrapSite(Ten, TCHAR_TO_UTF8(*TargetName));
				break;
			case EMJWrap::Joint:
				mjs_wrapJoint(Ten, TCHAR_TO_UTF8(*TargetName), Wrap.CoefOrDivisor);
				break;
			case EMJWrap::Pulley:
				mjs_wrapPulley(Ten, Wrap.CoefOrDivisor);
				break;
			case EMJWrap::Sphere:
			case EMJWrap::Cylinder:
			{
				const FString SideSite = ScopedName(Wrap.SideSiteRef.ResolveName(Owner));
				mjs_wrapGeom(Ten, TCHAR_TO_UTF8(*TargetName), SideSite.IsEmpty() ? "" : TCHAR_TO_UTF8(*SideSite));
				break;
			}
			default:
				break;
			}
		}
	}

	TArray<UMJActuatorComponent*> Actuators;
	Owner->GetComponents<UMJActuatorComponent>(Actuators);
	for (const UMJActuatorComponent* ActComp : Actuators)
	{
		mjsActuator* Act = mjs_addActuator(S, FindDef(ActComp->DefaultClass));
		if (!Act) { continue; }
		mjs_setName(Act->element, TCHAR_TO_UTF8(*ScopedName(ActComp->GetMJName())));
		FMJSpecMarshal::ToMJSpec(ActComp->Spec, Act, bDeg);
		if (ActComp->Shortcut != EMJActuatorShortcut::General)
		{
			FMJSpecMarshal::ApplyActuatorShortcut(Act, ActComp->Shortcut, ActComp->ShortcutParams);
		}
		if (!ActComp->TargetRef.IsEmpty())
		{
			mjs_setString(Act->target, TCHAR_TO_UTF8(*ScopedName(ActComp->TargetRef.ResolveName(Owner))));
		}
		if (!ActComp->SliderSite.IsEmpty())
		{
			mjs_setString(Act->slidersite, TCHAR_TO_UTF8(*ScopedName(ActComp->SliderSite)));
		}
	}

	TArray<UMJSensorComponent*> Sensors;
	Owner->GetComponents<UMJSensorComponent>(Sensors);
	for (const UMJSensorComponent* SenComp : Sensors)
	{
		mjsSensor* Sen = mjs_addSensor(S);
		if (!Sen) { continue; }
		mjs_setName(Sen->element, TCHAR_TO_UTF8(*ScopedName(SenComp->GetMJName())));
		FMJSpecMarshal::ToMJSpec(SenComp->Spec, Sen, bDeg);
		if (!SenComp->ObjRef.IsEmpty())
		{
			mjs_setString(Sen->objname, TCHAR_TO_UTF8(*ScopedName(SenComp->ObjRef.ResolveName(Owner))));
		}
		if (!SenComp->RefObjRef.IsEmpty())
		{
			mjs_setString(Sen->refname, TCHAR_TO_UTF8(*ScopedName(SenComp->RefObjRef.ResolveName(Owner))));
		}
	}

	for (const FMJEqualityEntry& Entry : Equalities)
	{
		mjsEquality* Eq = mjs_addEquality(S, FindDef(Entry.DefaultClass));
		if (!Eq) { continue; }
		if (!Entry.Name.IsEmpty())
		{
			mjs_setName(Eq->element, TCHAR_TO_UTF8(*ScopedName(Entry.Name)));
		}
		Eq->type = static_cast<mjtEq>(Entry.Type);
		Eq->objtype = static_cast<mjtObj>(Entry.ObjType);
		Eq->active = Entry.bActive ? 1 : 0;
		if (!Entry.Name1.IsEmpty()) { mjs_setString(Eq->name1, TCHAR_TO_UTF8(*ScopedName(Entry.Name1))); }
		if (!Entry.Name2.IsEmpty()) { mjs_setString(Eq->name2, TCHAR_TO_UTF8(*ScopedName(Entry.Name2))); }
		for (int32 i = 0; i < FMath::Min(Entry.Data.Num(), static_cast<int32>(mjNEQDATA)); ++i) { Eq->data[i] = Entry.Data[i]; }
		for (int32 i = 0; i < FMath::Min(Entry.Solref.Num(), static_cast<int32>(mjNREF)); ++i) { Eq->solref[i] = Entry.Solref[i]; }
		for (int32 i = 0; i < FMath::Min(Entry.Solimp.Num(), static_cast<int32>(mjNIMP)); ++i) { Eq->solimp[i] = Entry.Solimp[i]; }
	}

	for (const FMJExcludeEntry& Entry : Excludes)
	{
		mjsExclude* Exc = mjs_addExclude(S);
		if (!Exc) { continue; }
		if (!Entry.Name.IsEmpty())
		{
			mjs_setName(Exc->element, TCHAR_TO_UTF8(*ScopedName(Entry.Name)));
		}
		if (!Entry.BodyName1.IsEmpty()) { mjs_setString(Exc->bodyname1, TCHAR_TO_UTF8(*ScopedName(Entry.BodyName1))); }
		if (!Entry.BodyName2.IsEmpty()) { mjs_setString(Exc->bodyname2, TCHAR_TO_UTF8(*ScopedName(Entry.BodyName2))); }
	}

	for (const FMJPairEntry& Entry : Pairs)
	{
		mjsPair* Pair = mjs_addPair(S, FindDef(Entry.DefaultClass));
		if (!Pair) { continue; }
		if (!Entry.Name.IsEmpty())
		{
			mjs_setName(Pair->element, TCHAR_TO_UTF8(*ScopedName(Entry.Name)));
		}
		if (!Entry.GeomName1.IsEmpty()) { mjs_setString(Pair->geomname1, TCHAR_TO_UTF8(*ScopedName(Entry.GeomName1))); }
		if (!Entry.GeomName2.IsEmpty()) { mjs_setString(Pair->geomname2, TCHAR_TO_UTF8(*ScopedName(Entry.GeomName2))); }
		Pair->condim = Entry.Condim;
		Pair->margin = Entry.Margin;
		Pair->gap = Entry.Gap;
		Pair->adhesion = Entry.Adhesion;
		for (int32 i = 0; i < FMath::Min(Entry.Solref.Num(), static_cast<int32>(mjNREF)); ++i) { Pair->solref[i] = Entry.Solref[i]; }
		for (int32 i = 0; i < FMath::Min(Entry.SolrefFriction.Num(), static_cast<int32>(mjNREF)); ++i) { Pair->solreffriction[i] = Entry.SolrefFriction[i]; }
		for (int32 i = 0; i < FMath::Min(Entry.Solimp.Num(), static_cast<int32>(mjNIMP)); ++i) { Pair->solimp[i] = Entry.Solimp[i]; }
		for (int32 i = 0; i < FMath::Min(Entry.Friction.Num(), 5); ++i) { Pair->friction[i] = Entry.Friction[i]; }
	}

	for (const FMJFlexEntry& Entry : Flexes)
	{
		mjsFlex* Flx = mjs_addFlex(S);
		if (!Flx) { continue; }
		if (!Entry.Name.IsEmpty()) { mjs_setName(Flx->element, TCHAR_TO_UTF8(*ScopedName(Entry.Name))); }
		Flx->contype = Entry.Contype;
		Flx->conaffinity = Entry.Conaffinity;
		Flx->condim = Entry.Condim;
		Flx->priority = Entry.Priority;
		for (int32 i = 0; i < FMath::Min(Entry.Friction.Num(), 3); ++i) { Flx->friction[i] = Entry.Friction[i]; }
		Flx->solmix = Entry.Solmix;
		for (int32 i = 0; i < FMath::Min(Entry.Solref.Num(), static_cast<int32>(mjNREF)); ++i) { Flx->solref[i] = Entry.Solref[i]; }
		for (int32 i = 0; i < FMath::Min(Entry.Solimp.Num(), static_cast<int32>(mjNIMP)); ++i) { Flx->solimp[i] = Entry.Solimp[i]; }
		Flx->margin = Entry.Margin;
		Flx->gap = Entry.Gap;
		Flx->dim = Entry.Dim;
		Flx->radius = Entry.Radius;
		for (int32 i = 0; i < FMath::Min(Entry.Size.Num(), 3); ++i) { Flx->size[i] = Entry.Size[i]; }
		Flx->flatskin = Entry.bFlatskin ? 1 : 0;
		Flx->selfcollide = static_cast<mjtFlexSelf>(Entry.Selfcollide);
		Flx->passive = Entry.Passive;
		Flx->activelayers = Entry.Activelayers;
		Flx->group = Entry.Group;
		Flx->edgestiffness = Entry.Edgestiffness;
		Flx->edgedamping = Entry.Edgedamping;
		Flx->rgba[0] = Entry.Rgba.R; Flx->rgba[1] = Entry.Rgba.G; Flx->rgba[2] = Entry.Rgba.B; Flx->rgba[3] = Entry.Rgba.A;
		if (!Entry.Material.IsEmpty()) { Flx->material->push_back(TCHAR_TO_UTF8(*Entry.Material)); }
		Flx->young = Entry.Young;
		Flx->poisson = Entry.Poisson;
		Flx->shear = Entry.Shear;
		Flx->damping = Entry.Damping;
		Flx->thickness = Entry.Thickness;
		for (int32 i = 0; i < FMath::Min(Entry.Stressyield.Num(), 2); ++i) { Flx->stressyield[i] = Entry.Stressyield[i]; }
		Flx->elastic2d = Entry.Elastic2d;
		Flx->elastic3d = Entry.Elastic3d;
		Flx->section = static_cast<mjtFlexSection>(Entry.Section);
		for (int32 i = 0; i < FMath::Min(Entry.Sectionsize.Num(), 2); ++i) { Flx->sectionsize[i] = Entry.Sectionsize[i]; }
		for (int32 i = 0; i < FMath::Min(Entry.Cellcount.Num(), 3); ++i) { Flx->cellcount[i] = Entry.Cellcount[i]; }
		Flx->order = Entry.Order;
		for (int32 i = 0; i < Entry.Nodebody.Num(); ++i) { Flx->nodebody->push_back(TCHAR_TO_UTF8(*ScopedName(Entry.Nodebody[i]))); }
		for (int32 i = 0; i < Entry.Vertbody.Num(); ++i) { Flx->vertbody->push_back(TCHAR_TO_UTF8(*ScopedName(Entry.Vertbody[i]))); }
		if (Entry.Node.Num() > 0) { mjs_setDouble(Flx->node, Entry.Node.GetData(), Entry.Node.Num()); }
		if (Entry.Vert.Num() > 0) { mjs_setDouble(Flx->vert, Entry.Vert.GetData(), Entry.Vert.Num()); }
		if (Entry.Elem.Num() > 0) { mjs_setInt(Flx->elem, Entry.Elem.GetData(), Entry.Elem.Num()); }
		if (Entry.Texcoord.Num() > 0) { mjs_setFloat(Flx->texcoord, Entry.Texcoord.GetData(), Entry.Texcoord.Num()); }
		if (Entry.Elemtexcoord.Num() > 0) { mjs_setInt(Flx->elemtexcoord, Entry.Elemtexcoord.GetData(), Entry.Elemtexcoord.Num()); }
	}

	for (const FMJTupleEntry& Entry : Tuples)
	{
		mjsTuple* Tup = mjs_addTuple(S);
		if (!Tup) { continue; }
		if (!Entry.Name.IsEmpty()) { mjs_setName(Tup->element, TCHAR_TO_UTF8(*ScopedName(Entry.Name))); }
		for (int32 i = 0; i < Entry.Objtype.Num(); ++i) { Tup->objtype->push_back(Entry.Objtype[i]); }
		for (int32 i = 0; i < Entry.Objname.Num(); ++i) { Tup->objname->push_back(TCHAR_TO_UTF8(*ScopedName(Entry.Objname[i]))); }
		for (int32 i = 0; i < Entry.Objprm.Num(); ++i) { Tup->objprm->push_back(Entry.Objprm[i]); }
	}
}

bool UMJModelComponent::BuildSpecFromActor(FString& OutError)
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		OutError = TEXT("UMJModelComponent has no owning Actor");
		return false;
	}

	if (AssetLibrary)
	{
		VfsHandle.Init();
		AssetLibrary->PopulateVFS(VfsHandle.Get());
	}

	mjSpec* S = mj_makeSpec();
	if (!S)
	{
		OutError = TEXT("mj_makeSpec returned null");
		return false;
	}

	mjs_setString(S->modelname, TCHAR_TO_UTF8(*ModelName));
	S->compiler.degree = bCompilerDegree ? 1 : 0;

	if (CompilerSettings.bEnabled)
	{
		S->compiler.autolimits = CompilerSettings.bAutolimits ? 1 : 0;
		S->compiler.boundmass = CompilerSettings.BoundMass;
		S->compiler.boundinertia = CompilerSettings.BoundInertia;
		S->compiler.settotalmass = CompilerSettings.SetTotalMass;
		S->compiler.balanceinertia = CompilerSettings.bBalanceInertia ? 1 : 0;
		S->compiler.fitaabb = CompilerSettings.bFitAabb ? 1 : 0;
		S->compiler.discardvisual = CompilerSettings.bDiscardVisual ? 1 : 0;
		S->compiler.fusestatic = CompilerSettings.bFuseStatic ? 1 : 0;
		S->compiler.inertiafromgeom = static_cast<mjtInertiaFromGeom>(CompilerSettings.InertiaFromGeom);
		S->compiler.inertiagrouprange[0] = CompilerSettings.InertiaGroupRangeMin;
		S->compiler.inertiagrouprange[1] = CompilerSettings.InertiaGroupRangeMax;
		S->compiler.alignfree = CompilerSettings.bAlignFree ? 1 : 0;
		if (CompilerSettings.EulerSeq.Len() >= 3)
		{
			FTCHARToUTF8 SeqUtf8(*CompilerSettings.EulerSeq);
			std::memcpy(S->compiler.eulerseq, SeqUtf8.Get(), 3);
		}
		if (!CompilerSettings.MeshDir.IsEmpty())
		{
			mjs_setString(S->compiler.meshdir, TCHAR_TO_UTF8(*CompilerSettings.MeshDir));
		}
		if (!CompilerSettings.TextureDir.IsEmpty())
		{
			mjs_setString(S->compiler.texturedir, TCHAR_TO_UTF8(*CompilerSettings.TextureDir));
		}
	}

	if (OptionSettings.bEnabled)
	{
		S->option.timestep = OptionSettings.Timestep;
		S->option.impratio = OptionSettings.ImpRatio;
		S->option.tolerance = OptionSettings.Tolerance;
		S->option.ls_tolerance = OptionSettings.LsTolerance;
		S->option.noslip_tolerance = OptionSettings.NoslipTolerance;
		S->option.ccd_tolerance = OptionSettings.CcdTolerance;
		S->option.sleep_tolerance = OptionSettings.SleepTolerance;
		S->option.gravity[0] = OptionSettings.Gravity.X;
		S->option.gravity[1] = OptionSettings.Gravity.Y;
		S->option.gravity[2] = OptionSettings.Gravity.Z;
		S->option.wind[0] = OptionSettings.Wind.X;
		S->option.wind[1] = OptionSettings.Wind.Y;
		S->option.wind[2] = OptionSettings.Wind.Z;
		S->option.magnetic[0] = OptionSettings.Magnetic.X;
		S->option.magnetic[1] = OptionSettings.Magnetic.Y;
		S->option.magnetic[2] = OptionSettings.Magnetic.Z;
		S->option.density = OptionSettings.Density;
		S->option.viscosity = OptionSettings.Viscosity;
		S->option.integrator = OptionSettings.Integrator;
		S->option.cone = OptionSettings.Cone;
		S->option.jacobian = OptionSettings.Jacobian;
		S->option.solver = OptionSettings.Solver;
		S->option.iterations = OptionSettings.Iterations;
		S->option.ls_iterations = OptionSettings.LsIterations;
		S->option.noslip_iterations = OptionSettings.NoslipIterations;
		S->option.ccd_iterations = OptionSettings.CcdIterations;
		S->option.disableflags = OptionSettings.DisableFlags;
		S->option.enableflags = OptionSettings.EnableFlags;
		S->option.disableactuator = OptionSettings.DisableActuator;
		S->option.sdf_initpoints = OptionSettings.SdfInitPoints;
		S->option.sdf_iterations = OptionSettings.SdfIterations;
	}

	if (SizeSettings.bEnabled)
	{
		S->strippath = SizeSettings.bStripPath ? 1 : 0;
		S->memory = static_cast<mjtSize>(SizeSettings.Memory);
		S->nemax = SizeSettings.Nemax;
		S->nuserdata = SizeSettings.Nuserdata;
		S->nuser_body = SizeSettings.NuserBody;
		S->nuser_jnt = SizeSettings.NuserJnt;
		S->nuser_geom = SizeSettings.NuserGeom;
		S->nuser_site = SizeSettings.NuserSite;
		S->nuser_cam = SizeSettings.NuserCam;
		S->nuser_tendon = SizeSettings.NuserTendon;
		S->nuser_actuator = SizeSettings.NuserActuator;
		S->nuser_sensor = SizeSettings.NuserSensor;
		S->nkey = SizeSettings.Nkey;
		S->njmax = SizeSettings.Njmax;
		S->nconmax = SizeSettings.Nconmax;
		S->nstack = static_cast<mjtSize>(SizeSettings.Nstack);
	}

	BuildDefaultsInSpec(S);
	BuildAssetsInSpec(S);

	mjsBody* WorldBody = mjs_findBody(S, "world");
	if (USceneComponent* Root = Owner->GetRootComponent())
	{
		if (Root->IsA<UMJSceneComponent>())
		{
			BuildSceneSubtree(S, Root, WorldBody, nullptr);
		}
		else
		{
			for (USceneComponent* Child : Root->GetAttachChildren())
			{
				BuildSceneSubtree(S, Child, WorldBody, nullptr);
			}
		}
	}

	BuildNonSpatialComponents(S);

	SpecHandle.Reset(S);
	return true;
}

bool UMJModelComponent::CompileModel(FString& OutError)
{
	if (!SpecHandle)
	{
		if (!BuildSpecFromActor(OutError))
		{
			return false;
		}
	}
	else if (AssetLibrary && !VfsHandle)
	{
		VfsHandle.Init();
		AssetLibrary->PopulateVFS(VfsHandle.Get());
	}

	mjModel* M = mj_compile(SpecHandle.Get(), VfsHandle.Get());
	if (!M)
	{
		const char* Err = mjs_getError(SpecHandle.Get());
		OutError = UTF8_TO_TCHAR(Err ? Err : "mj_compile failed");
		return false;
	}

	mjData* D = mj_makeData(M);
	if (!D)
	{
		mj_deleteModel(M);
		OutError = TEXT("mj_makeData failed");
		return false;
	}

	ModelHandle.Reset(M);
	DataHandle.Reset(D);
	RefreshRuntimeIds();
	return true;
}

bool UMJModelComponent::RecompileModel(FString& OutError)
{
	if (!ModelHandle || !DataHandle || !SpecHandle)
	{
		return CompileModel(OutError);
	}

	// Update existing SpecHandle elements in-place so mj_recompile preserves qpos/qvel addresses
	mjSpec* S = SpecHandle.Get();
	const bool bDeg = bCompilerDegree;
	if (AActor* Owner = GetOwner())
	{
		TArray<UMJGeomComponent*> Geoms;
		Owner->GetComponents<UMJGeomComponent>(Geoms);
		for (const UMJGeomComponent* G : Geoms)
		{
			if (mjsElement* E = mjs_findElement(S, mjOBJ_GEOM, TCHAR_TO_UTF8(*G->GetMJName())))
			{
				FMJSpecMarshal::ToMJSpec(G->Spec, mjs_asGeom(E), bDeg);
			}
		}
		TArray<UMJJointComponent*> Joints;
		Owner->GetComponents<UMJJointComponent>(Joints);
		for (const UMJJointComponent* J : Joints)
		{
			if (mjsElement* E = mjs_findElement(S, mjOBJ_JOINT, TCHAR_TO_UTF8(*J->GetMJName())))
			{
				FMJSpecMarshal::ToMJSpec(J->Spec, mjs_asJoint(E), bDeg);
			}
		}
		TArray<UMJActuatorComponent*> Actuators;
		Owner->GetComponents<UMJActuatorComponent>(Actuators);
		for (const UMJActuatorComponent* A : Actuators)
		{
			if (mjsElement* E = mjs_findElement(S, mjOBJ_ACTUATOR, TCHAR_TO_UTF8(*A->GetMJName())))
			{
				mjsActuator* Act = mjs_asActuator(E);
				if (A->Shortcut != EMJActuatorShortcut::General)
				{
					FMJSpecMarshal::ApplyActuatorShortcut(Act, A->Shortcut, A->ShortcutParams);
				}
				else
				{
					FMJSpecMarshal::ToMJSpec(A->Spec, Act, bDeg);
				}
			}
		}
	}

	const int Rc = mj_recompile(S, VfsHandle.Get(), ModelHandle.Get(), DataHandle.Get());
	if (Rc != 0)
	{
		const char* Err = mjs_getError(S);
		OutError = UTF8_TO_TCHAR(Err ? Err : "mj_recompile failed");
		return false;
	}

	RefreshRuntimeIds();
	return true;
}

void UMJModelComponent::RefreshRuntimeIds()
{
	CachedBodies.Reset();
	AActor* Owner = GetOwner();
	mjModel* M = ModelHandle.Get();
	if (!Owner || !M)
	{
		return;
	}

	TArray<UMJBodyComponent*> Bodies;
	Owner->GetComponents<UMJBodyComponent>(Bodies);
	for (UMJBodyComponent* B : Bodies)
	{
		B->BodyId = mj_name2id(M, mjOBJ_BODY, TCHAR_TO_UTF8(*B->GetMJName()));
		CachedBodies.Add(B);
	}

	TArray<UMJGeomComponent*> Geoms;
	Owner->GetComponents<UMJGeomComponent>(Geoms);
	for (UMJGeomComponent* G : Geoms)
	{
		G->GeomId = mj_name2id(M, mjOBJ_GEOM, TCHAR_TO_UTF8(*G->GetMJName()));
	}

	TArray<UMJJointComponent*> Joints;
	Owner->GetComponents<UMJJointComponent>(Joints);
	for (UMJJointComponent* J : Joints)
	{
		J->JointId = mj_name2id(M, mjOBJ_JOINT, TCHAR_TO_UTF8(*J->GetMJName()));
	}

	TArray<UMJSiteComponent*> Sites;
	Owner->GetComponents<UMJSiteComponent>(Sites);
	for (UMJSiteComponent* Site : Sites)
	{
		Site->SiteId = mj_name2id(M, mjOBJ_SITE, TCHAR_TO_UTF8(*Site->GetMJName()));
	}

	TArray<UMJActuatorComponent*> Actuators;
	Owner->GetComponents<UMJActuatorComponent>(Actuators);
	for (UMJActuatorComponent* Act : Actuators)
	{
		Act->ActuatorId = mj_name2id(M, mjOBJ_ACTUATOR, TCHAR_TO_UTF8(*Act->GetMJName()));
	}
}

void UMJModelComponent::StepSimulation(double Dt)
{
	mjModel* M = ModelHandle.Get();
	mjData* D = DataHandle.Get();
	if (!M || !D || Dt <= 0.0)
	{
		return;
	}

	if (AActor* Owner = GetOwner())
	{
		TArray<UMJActuatorComponent*> Actuators;
		Owner->GetComponents<UMJActuatorComponent>(Actuators);
		for (const UMJActuatorComponent* Act : Actuators)
		{
			if (Act->ActuatorId >= 0 && Act->ActuatorId < M->nu)
			{
				D->ctrl[Act->ActuatorId] = Act->Ctrl;
			}
		}
	}

	const double TargetTime = D->time + Dt;
	while (D->time < TargetTime)
	{
		mj_step(M, D);
	}

	SyncTransformsFromData();
}

void UMJModelComponent::SyncTransformsFromData()
{
	mjModel* M = ModelHandle.Get();
	mjData* D = DataHandle.Get();
	AActor* Owner = GetOwner();
	if (!M || !D || !Owner)
	{
		return;
	}

	const FTransform ActorWorld = Owner->GetActorTransform();
	for (const TWeakObjectPtr<UMJBodyComponent>& WeakBody : CachedBodies)
	{
		UMJBodyComponent* Body = WeakBody.Get();
		if (!Body || Body->BodyId <= 0 || Body->BodyId >= M->nbody)
		{
			continue;
		}
		FTransform BodyLocalToModel;
		FMJUnitConverter::MJTransformToUE(D->xpos + 3 * Body->BodyId, D->xquat + 4 * Body->BodyId, BodyLocalToModel);
		Body->SetWorldTransform(BodyLocalToModel * ActorWorld);
	}
}

bool UMJModelComponent::ExportXML(
	FString& OutXml,
	FString& OutError,
	bool bSaveCompiled,
	bool bSaveCanonical,
	bool bSaveInertial)
{
	if (!SpecHandle)
	{
		if (!BuildSpecFromActor(OutError))
		{
			return false;
		}
	}

	mjSpec* S = SpecHandle.Get();
	S->compiler.savecompiled = bSaveCompiled ? 1 : 0;
	S->compiler.savecanonical = bSaveCanonical ? 1 : 0;
	S->compiler.saveinertial = bSaveInertial ? 1 : 0;

	TArray<char> Buffer;
	Buffer.SetNumZeroed(256 * 1024);
	char ErrBuf[1024] = {};
	const int Written = mj_saveXMLString(S, Buffer.GetData(), Buffer.Num(), ErrBuf, sizeof(ErrBuf));
	if (Written < 0)
	{
		const char* Msg = ErrBuf[0] ? ErrBuf : "mj_saveXMLString failed";
		OutError = UTF8_TO_TCHAR(Msg);
		return false;
	}

	OutXml = UTF8_TO_TCHAR(Buffer.GetData());
	return true;
}
