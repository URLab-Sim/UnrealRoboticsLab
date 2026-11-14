#include "MJModelComponent.h"
#include "MJAssetLibrary.h"
#include "MJSceneComponents.h"
#include "MJActorComponents.h"
#include "MJUnitConverter.h"
#include "Utils/URLabLogging.h"
#include "Generated/MJSpecMarshal.h"
#include "GameFramework/Actor.h"

UMJModelComponent::UMJModelComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UMJModelComponent::BuildDefaultsInSpec(mjSpec* S) const
{
	for (const FMJDefaultClassEntry& Entry : DefaultClasses)
	{
		if (Entry.ClassName.IsEmpty())
		{
			continue;
		}
		FTCHARToUTF8 ClassUtf8(*Entry.ClassName);
		mjsDefault* Def = mjs_findDefault(S, ClassUtf8.Get());
		if (!Def)
		{
			const mjsDefault* ParentDef = Entry.ParentClassName.IsEmpty()
				? nullptr
				: mjs_findDefault(S, TCHAR_TO_UTF8(*Entry.ParentClassName));
			Def = mjs_addDefault(S, ClassUtf8.Get(), ParentDef);
		}
		if (!Def)
		{
			continue;
		}
		const bool bDeg = bCompilerDegree;
		FMJSpecMarshal::ToMJSpec(Entry.Geom, Def->geom, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Joint, Def->joint, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Site, Def->site, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Actuator, Def->actuator, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Camera, Def->camera, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Light, Def->light, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Tendon, Def->tendon, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Mesh, Def->mesh, bDeg);
		FMJSpecMarshal::ToMJSpec(Entry.Material, Def->material, bDeg);
	}
}

void UMJModelComponent::BuildSceneSubtree(mjSpec* S, USceneComponent* Comp, mjsBody* ParentBody, mjsFrame* ParentFrame) const
{
	if (!Comp || !S || !ParentBody)
	{
		return;
	}

	const bool bDeg = bCompilerDegree;
	const FTransform RelXform = Comp->GetRelativeTransform();
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

	TArray<UMJTendonComponent*> Tendons;
	Owner->GetComponents<UMJTendonComponent>(Tendons);
	for (const UMJTendonComponent* TenComp : Tendons)
	{
		mjsTendon* Ten = mjs_addTendon(S, FindDef(TenComp->DefaultClass));
		if (!Ten) { continue; }
		mjs_setName(Ten->element, TCHAR_TO_UTF8(*TenComp->GetMJName()));
		FMJSpecMarshal::ToMJSpec(TenComp->Spec, Ten, bDeg);
		if (!TenComp->MaterialRef.IsEmpty())
		{
			mjs_setString(Ten->material, TCHAR_TO_UTF8(*TenComp->MaterialRef.ResolveName(Owner)));
		}
		for (const FMJTendonWrapEntry& Wrap : TenComp->Wraps)
		{
			const FString TargetName = Wrap.TargetRef.ResolveName(Owner);
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
				const FString SideSite = Wrap.SideSiteRef.ResolveName(Owner);
				mjs_wrapGeom(Ten, TCHAR_TO_UTF8(*TargetName), SideSite.IsEmpty() ? "" : TCHAR_TO_UTF8(*SideSite));
				break;
			}
			default:
				break;
			}
		}
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

	BuildDefaultsInSpec(S);

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
