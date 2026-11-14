#include "MJXmlImporterExporter.h"
#include "MJModelComponent.h"
#include "MJSceneComponents.h"
#include "MJActorComponents.h"
#include "MJUnitConverter.h"
#include "MJSpecHandle.h"
#include "Generated/MJSpecMarshal.h"
#include "GameFramework/Actor.h"
#include "Engine/Blueprint.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/SCS_Node.h"

namespace
{
FString GetElemName(const mjsElement* Elem)
{
	if (!Elem) { return FString(); }
	const mjString* S = mjs_getName(const_cast<mjsElement*>(Elem));
	const char* C = mjs_getString(S);
	return C ? FString(UTF8_TO_TCHAR(C)) : FString();
}

FString GetDefaultClassName(const mjsElement* Elem)
{
	if (!Elem) { return FString(); }
	const mjsDefault* Def = mjs_getDefault(Elem);
	if (!Def || !Def->element) { return FString(); }
	return GetElemName(Def->element);
}

void CollectDefaultFromElement(const mjsElement* Elem, UMJModelComponent* ModelComp, TSet<const mjsDefault*>& SeenDefaults, bool bDeg)
{
	if (!Elem || !ModelComp) { return; }
	const mjsDefault* Def = mjs_getDefault(Elem);
	if (!Def || !Def->element || SeenDefaults.Contains(Def)) { return; }
	SeenDefaults.Add(Def);

	FMJDefaultClassEntry Entry;
	Entry.ClassName = GetElemName(Def->element);
	if (Entry.ClassName.IsEmpty() || Entry.ClassName == TEXT("main")) { return; }
	FMJSpecMarshal::FromMJSpec(Def->geom, Entry.Geom, nullptr, bDeg);
	FMJSpecMarshal::FromMJSpec(Def->joint, Entry.Joint, nullptr, bDeg);
	FMJSpecMarshal::FromMJSpec(Def->site, Entry.Site, nullptr, bDeg);
	FMJSpecMarshal::FromMJSpec(Def->actuator, Entry.Actuator, nullptr, bDeg);
	FMJSpecMarshal::FromMJSpec(Def->camera, Entry.Camera, nullptr, bDeg);
	FMJSpecMarshal::FromMJSpec(Def->light, Entry.Light, nullptr, bDeg);
	FMJSpecMarshal::FromMJSpec(Def->tendon, Entry.Tendon, nullptr, bDeg);
	FMJSpecMarshal::FromMJSpec(Def->mesh, Entry.Mesh, nullptr, bDeg);
	FMJSpecMarshal::FromMJSpec(Def->material, Entry.Material, nullptr, bDeg);
	ModelComp->DefaultClasses.Add(MoveTemp(Entry));
}

void ExtractDefaults(mjSpec* Spec, UMJModelComponent* ModelComp, bool bDeg)
{
	ModelComp->DefaultClasses.Reset();
	TSet<const mjsDefault*> SeenDefaults;
	const mjtObj ElemTypes[] = { mjOBJ_BODY, mjOBJ_GEOM, mjOBJ_JOINT, mjOBJ_SITE, mjOBJ_CAMERA, mjOBJ_LIGHT, mjOBJ_ACTUATOR, mjOBJ_TENDON };
	for (mjtObj T : ElemTypes)
	{
		for (mjsElement* Elem = mjs_firstElement(Spec, T); Elem; Elem = mjs_nextElement(Spec, Elem))
		{
			CollectDefaultFromElement(Elem, ModelComp, SeenDefaults, bDeg);
		}
	}
}

template <typename TComp>
TComp* CreateSceneComp(AActor* Owner, USceneComponent* Parent, const FString& Name)
{
	TComp* Comp = NewObject<TComp>(Owner, NAME_None, RF_Transactional);
	Comp->ElementName = Name;
	Comp->RegisterComponent();
	if (Parent)
	{
		Comp->AttachToComponent(Parent, FAttachmentTransformRules::KeepRelativeTransform);
	}
	else if (!Owner->GetRootComponent())
	{
		Owner->SetRootComponent(Comp);
	}
	else
	{
		Comp->AttachToComponent(Owner->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
	}
	Owner->AddInstanceComponent(Comp);
	return Comp;
}

void ImportBodyChildrenToActor(
	AActor* Owner,
	mjsBody* Body,
	USceneComponent* ParentComp,
	bool bDeg)
{
	if (!Body || !Owner) { return; }

	// First pass: create UMJFrameComponent for all immediate frames in this body
	TMap<mjsFrame*, USceneComponent*> FrameMap;
	for (mjsElement* Elem = mjs_firstChild(Body, mjOBJ_FRAME, 0); Elem; Elem = mjs_nextChild(Body, Elem, 0))
	{
		mjsFrame* Frame = mjs_asFrame(Elem);
		if (!Frame) { continue; }
		mjsFrame* ParentFrame = mjs_getFrame(Elem);
		USceneComponent* AttachParent = FrameMap.Contains(ParentFrame) ? FrameMap[ParentFrame] : ParentComp;

		UMJFrameComponent* FrameComp = CreateSceneComp<UMJFrameComponent>(Owner, AttachParent, GetElemName(Elem));
		FTransform Xform;
		FMJUnitConverter::MJTransformToUE(Frame->pos, Frame->quat, Xform);
		FrameComp->SetRelativeTransform(Xform);
		FMJSpecMarshal::FromMJSpec(Frame, FrameComp->Spec, nullptr, bDeg);
		FrameMap.Add(Frame, FrameComp);
	}

	auto ResolveParent = [&FrameMap, ParentComp](mjsElement* Elem) -> USceneComponent*
	{
		mjsFrame* F = mjs_getFrame(Elem);
		return (F && FrameMap.Contains(F)) ? FrameMap[F] : ParentComp;
	};

	// Second pass: geoms, joints, sites, cameras, lights, child bodies
	const mjtObj ChildTypes[] = { mjOBJ_BODY, mjOBJ_GEOM, mjOBJ_JOINT, mjOBJ_SITE, mjOBJ_CAMERA, mjOBJ_LIGHT };
	for (mjtObj ChildType : ChildTypes)
	for (mjsElement* Elem = mjs_firstChild(Body, ChildType, 0); Elem; Elem = mjs_nextChild(Body, Elem, 0))
	{
		USceneComponent* AttachParent = ResolveParent(Elem);
		const mjsDefault* Def = mjs_getDefault(Elem);
		const FString DefName = GetDefaultClassName(Elem);
		const FString ElemName = GetElemName(Elem);

		switch (Elem->elemtype)
		{
		case mjOBJ_BODY:
		{
			mjsBody* ChildBody = mjs_asBody(Elem);
			UMJBodyComponent* BodyComp = CreateSceneComp<UMJBodyComponent>(Owner, AttachParent, ElemName);
			BodyComp->DefaultClass = DefName;
			FTransform Xform;
			FMJUnitConverter::MJTransformToUE(ChildBody->pos, ChildBody->quat, Xform);
			BodyComp->SetRelativeTransform(Xform);
			FMJSpecMarshal::FromMJSpec(ChildBody, BodyComp->Spec, nullptr, bDeg);
			ImportBodyChildrenToActor(Owner, ChildBody, BodyComp, bDeg);
			break;
		}
		case mjOBJ_GEOM:
		{
			mjsGeom* Geom = mjs_asGeom(Elem);
			UMJGeomComponent* GeomComp = CreateSceneComp<UMJGeomComponent>(Owner, AttachParent, ElemName);
			GeomComp->DefaultClass = DefName;
			FTransform Xform;
			FMJUnitConverter::MJTransformToUE(Geom->pos, Geom->quat, Xform);
			GeomComp->SetRelativeTransform(Xform);
			FMJSpecMarshal::FromMJSpec(Geom, GeomComp->Spec, Def ? Def->geom : nullptr, bDeg);
			if (const char* Mat = mjs_getString(Geom->material)) { if (Mat[0]) { GeomComp->MaterialRef.FallbackName = UTF8_TO_TCHAR(Mat); } }
			if (const char* Mesh = mjs_getString(Geom->meshname)) { if (Mesh[0]) { GeomComp->MeshRef.FallbackName = UTF8_TO_TCHAR(Mesh); } }
			break;
		}
		case mjOBJ_JOINT:
		{
			mjsJoint* Joint = mjs_asJoint(Elem);
			UMJJointComponent* JointComp = CreateSceneComp<UMJJointComponent>(Owner, AttachParent, ElemName);
			JointComp->DefaultClass = DefName;
			FVector Pos;
			FMJUnitConverter::MJPositionToUE(Joint->pos, Pos);
			JointComp->SetRelativeLocation(Pos);
			FMJSpecMarshal::FromMJSpec(Joint, JointComp->Spec, Def ? Def->joint : nullptr, bDeg);
			break;
		}
		case mjOBJ_SITE:
		{
			mjsSite* Site = mjs_asSite(Elem);
			UMJSiteComponent* SiteComp = CreateSceneComp<UMJSiteComponent>(Owner, AttachParent, ElemName);
			SiteComp->DefaultClass = DefName;
			FTransform Xform;
			FMJUnitConverter::MJTransformToUE(Site->pos, Site->quat, Xform);
			SiteComp->SetRelativeTransform(Xform);
			FMJSpecMarshal::FromMJSpec(Site, SiteComp->Spec, Def ? Def->site : nullptr, bDeg);
			if (const char* Mat = mjs_getString(Site->material)) { if (Mat[0]) { SiteComp->MaterialRef.FallbackName = UTF8_TO_TCHAR(Mat); } }
			if (const char* Mesh = mjs_getString(Site->meshname)) { if (Mesh[0]) { SiteComp->MeshRef.FallbackName = UTF8_TO_TCHAR(Mesh); } }
			break;
		}
		case mjOBJ_CAMERA:
		{
			mjsCamera* Cam = mjs_asCamera(Elem);
			UMJCameraComponent* CamComp = CreateSceneComp<UMJCameraComponent>(Owner, AttachParent, ElemName);
			CamComp->DefaultClass = DefName;
			FTransform Xform;
			FMJUnitConverter::MJTransformToUE(Cam->pos, Cam->quat, Xform);
			CamComp->SetRelativeTransform(Xform);
			FMJSpecMarshal::FromMJSpec(Cam, CamComp->Spec, Def ? Def->camera : nullptr, bDeg);
			if (const char* Tgt = mjs_getString(Cam->targetbody)) { if (Tgt[0]) { CamComp->TargetBodyRef.FallbackName = UTF8_TO_TCHAR(Tgt); } }
			break;
		}
		case mjOBJ_LIGHT:
		{
			mjsLight* Light = mjs_asLight(Elem);
			UMJLightComponent* LightComp = CreateSceneComp<UMJLightComponent>(Owner, AttachParent, ElemName);
			LightComp->DefaultClass = DefName;
			FVector Pos, Dir;
			FMJUnitConverter::MJPositionToUE(Light->pos, Pos);
			FMJUnitConverter::MJDirectionToUE(Light->dir, Dir);
			LightComp->SetRelativeLocation(Pos);
			if (!Dir.IsNearlyZero()) { LightComp->SetRelativeRotation(Dir.Rotation()); }
			FMJSpecMarshal::FromMJSpec(Light, LightComp->Spec, Def ? Def->light : nullptr, bDeg);
			if (const char* Tgt = mjs_getString(Light->targetbody)) { if (Tgt[0]) { LightComp->TargetBodyRef.FallbackName = UTF8_TO_TCHAR(Tgt); } }
			if (const char* Tex = mjs_getString(Light->texture)) { if (Tex[0]) { LightComp->TextureRef.FallbackName = UTF8_TO_TCHAR(Tex); } }
			break;
		}
		default:
			break;
		}
	}
}

void ImportBodyChildrenToSCS(
	UBlueprint* BP,
	USimpleConstructionScript* SCS,
	mjsBody* Body,
	USCS_Node* ParentNode,
	bool bDeg)
{
	if (!Body || !SCS) { return; }

	const mjtObj ChildTypes[] = { mjOBJ_FRAME, mjOBJ_BODY, mjOBJ_GEOM, mjOBJ_JOINT };
	for (mjtObj ChildType : ChildTypes)
	for (mjsElement* Elem = mjs_firstChild(Body, ChildType, 0); Elem; Elem = mjs_nextChild(Body, Elem, 0))
	{
		const mjsDefault* Def = mjs_getDefault(Elem);
		const FString DefName = GetDefaultClassName(Elem);
		const FString ElemName = GetElemName(Elem);
		const FName NodeVarName = ElemName.IsEmpty() ? NAME_None : FName(*ElemName);

		auto AttachNode = [SCS, ParentNode](USCS_Node* Node)
		{
			if (ParentNode) { ParentNode->AddChildNode(Node); }
			else { SCS->AddNode(Node); }
		};

		if (Elem->elemtype == mjOBJ_BODY)
		{
			mjsBody* ChildBody = mjs_asBody(Elem);
			USCS_Node* Node = SCS->CreateNode(UMJBodyComponent::StaticClass(), NodeVarName);
			UMJBodyComponent* Comp = CastChecked<UMJBodyComponent>(Node->ComponentTemplate);
			Comp->ElementName = ElemName;
			Comp->DefaultClass = DefName;
			FTransform Xform;
			FMJUnitConverter::MJTransformToUE(ChildBody->pos, ChildBody->quat, Xform);
			Comp->SetRelativeTransform(Xform);
			FMJSpecMarshal::FromMJSpec(ChildBody, Comp->Spec, nullptr, bDeg);
			AttachNode(Node);
			ImportBodyChildrenToSCS(BP, SCS, ChildBody, Node, bDeg);
		}
		else if (Elem->elemtype == mjOBJ_GEOM)
		{
			mjsGeom* Geom = mjs_asGeom(Elem);
			USCS_Node* Node = SCS->CreateNode(UMJGeomComponent::StaticClass(), NodeVarName);
			UMJGeomComponent* Comp = CastChecked<UMJGeomComponent>(Node->ComponentTemplate);
			Comp->ElementName = ElemName;
			Comp->DefaultClass = DefName;
			FTransform Xform;
			FMJUnitConverter::MJTransformToUE(Geom->pos, Geom->quat, Xform);
			Comp->SetRelativeTransform(Xform);
			FMJSpecMarshal::FromMJSpec(Geom, Comp->Spec, Def ? Def->geom : nullptr, bDeg);
			AttachNode(Node);
		}
		else if (Elem->elemtype == mjOBJ_JOINT)
		{
			mjsJoint* Joint = mjs_asJoint(Elem);
			USCS_Node* Node = SCS->CreateNode(UMJJointComponent::StaticClass(), NodeVarName);
			UMJJointComponent* Comp = CastChecked<UMJJointComponent>(Node->ComponentTemplate);
			Comp->ElementName = ElemName;
			Comp->DefaultClass = DefName;
			FVector Pos;
			FMJUnitConverter::MJPositionToUE(Joint->pos, Pos);
			Comp->SetRelativeLocation(Pos);
			FMJSpecMarshal::FromMJSpec(Joint, Comp->Spec, Def ? Def->joint : nullptr, bDeg);
			AttachNode(Node);
		}
		else if (Elem->elemtype == mjOBJ_FRAME)
		{
			mjsFrame* Frame = mjs_asFrame(Elem);
			USCS_Node* Node = SCS->CreateNode(UMJFrameComponent::StaticClass(), NodeVarName);
			UMJFrameComponent* Comp = CastChecked<UMJFrameComponent>(Node->ComponentTemplate);
			Comp->ElementName = ElemName;
			FTransform Xform;
			FMJUnitConverter::MJTransformToUE(Frame->pos, Frame->quat, Xform);
			Comp->SetRelativeTransform(Xform);
			FMJSpecMarshal::FromMJSpec(Frame, Comp->Spec, nullptr, bDeg);
			AttachNode(Node);
		}
	}
}
} // namespace

bool FMJXmlImporterExporter::ImportXMLToActor(const FString& Xml, AActor* TargetActor, const mjVFS* Vfs, FString& OutError)
{
	if (!TargetActor)
	{
		OutError = TEXT("TargetActor is null");
		return false;
	}

	char ErrBuf[1024] = {};
	mjSpec* Spec = mj_parseXMLString(TCHAR_TO_UTF8(*Xml), Vfs, ErrBuf, sizeof(ErrBuf));
	if (!Spec)
	{
		const char* Msg = ErrBuf[0] ? ErrBuf : "mj_parseXMLString failed";
		OutError = UTF8_TO_TCHAR(Msg);
		return false;
	}

	UMJModelComponent* ModelComp = TargetActor->FindComponentByClass<UMJModelComponent>();
	if (!ModelComp)
	{
		ModelComp = NewObject<UMJModelComponent>(TargetActor, TEXT("MuJoCoModel"), RF_Transactional);
		ModelComp->RegisterComponent();
		TargetActor->AddInstanceComponent(ModelComp);
	}

	if (const char* MName = mjs_getString(Spec->modelname))
	{
		if (MName[0]) { ModelComp->ModelName = UTF8_TO_TCHAR(MName); }
	}
	const bool bDeg = (Spec->compiler.degree != 0);
	ModelComp->bCompilerDegree = bDeg;

	ExtractDefaults(Spec, ModelComp, bDeg);

	if (!TargetActor->GetRootComponent())
	{
		USceneComponent* Root = NewObject<USceneComponent>(TargetActor, TEXT("WorldRoot"), RF_Transactional);
		Root->RegisterComponent();
		TargetActor->SetRootComponent(Root);
		TargetActor->AddInstanceComponent(Root);
	}

	mjsBody* WorldBody = mjs_findBody(Spec, "world");
	if (WorldBody)
	{
		ImportBodyChildrenToActor(TargetActor, WorldBody, TargetActor->GetRootComponent(), bDeg);
	}

	// Actuators
	for (mjsElement* Elem = mjs_firstElement(Spec, mjOBJ_ACTUATOR); Elem; Elem = mjs_nextElement(Spec, Elem))
	{
		mjsActuator* Act = mjs_asActuator(Elem);
		const mjsDefault* Def = mjs_getDefault(Elem);
		UMJActuatorComponent* ActComp = NewObject<UMJActuatorComponent>(TargetActor, NAME_None, RF_Transactional);
		ActComp->ElementName = GetElemName(Elem);
		ActComp->DefaultClass = GetDefaultClassName(Elem);
		FMJSpecMarshal::ExtractActuatorShortcut(Act, ActComp->Shortcut, ActComp->ShortcutParams);
		FMJSpecMarshal::FromMJSpec(Act, ActComp->Spec, Def ? Def->actuator : nullptr, bDeg);
		if (const char* Tgt = mjs_getString(Act->target)) { if (Tgt[0]) { ActComp->TargetRef.FallbackName = UTF8_TO_TCHAR(Tgt); } }
		ActComp->RegisterComponent();
		TargetActor->AddInstanceComponent(ActComp);
	}

	// Sensors
	for (mjsElement* Elem = mjs_firstElement(Spec, mjOBJ_SENSOR); Elem; Elem = mjs_nextElement(Spec, Elem))
	{
		mjsSensor* Sen = mjs_asSensor(Elem);
		UMJSensorComponent* SenComp = NewObject<UMJSensorComponent>(TargetActor, NAME_None, RF_Transactional);
		SenComp->ElementName = GetElemName(Elem);
		FMJSpecMarshal::FromMJSpec(Sen, SenComp->Spec, nullptr, bDeg);
		if (const char* Obj = mjs_getString(Sen->objname)) { if (Obj[0]) { SenComp->ObjRef.FallbackName = UTF8_TO_TCHAR(Obj); } }
		if (const char* Ref = mjs_getString(Sen->refname)) { if (Ref[0]) { SenComp->RefObjRef.FallbackName = UTF8_TO_TCHAR(Ref); } }
		SenComp->RegisterComponent();
		TargetActor->AddInstanceComponent(SenComp);
	}

	ModelComp->AdoptSpec(Spec);
	return true;
}

bool FMJXmlImporterExporter::ImportXMLToBlueprintSCS(const FString& Xml, UBlueprint* Blueprint, const mjVFS* Vfs, FString& OutError)
{
	if (!Blueprint || !Blueprint->SimpleConstructionScript)
	{
		OutError = TEXT("Invalid Blueprint or SimpleConstructionScript");
		return false;
	}

	char ErrBuf[1024] = {};
	FMJSpecHandle Spec(mj_parseXMLString(TCHAR_TO_UTF8(*Xml), Vfs, ErrBuf, sizeof(ErrBuf)));
	if (!Spec)
	{
		const char* Msg = ErrBuf[0] ? ErrBuf : "mj_parseXMLString failed";
		OutError = UTF8_TO_TCHAR(Msg);
		return false;
	}

	USimpleConstructionScript* SCS = Blueprint->SimpleConstructionScript;
	USCS_Node* ModelNode = SCS->CreateNode(UMJModelComponent::StaticClass(), TEXT("MuJoCoModel"));
	UMJModelComponent* ModelComp = CastChecked<UMJModelComponent>(ModelNode->ComponentTemplate);
	const bool bDeg = (Spec.Get()->compiler.degree != 0);
	ModelComp->bCompilerDegree = bDeg;
	ExtractDefaults(Spec.Get(), ModelComp, bDeg);
	SCS->AddNode(ModelNode);

	if (mjsBody* WorldBody = mjs_findBody(Spec.Get(), "world"))
	{
		ImportBodyChildrenToSCS(Blueprint, SCS, WorldBody, nullptr, bDeg);
	}

	// Actuators
	for (mjsElement* Elem = mjs_firstElement(Spec.Get(), mjOBJ_ACTUATOR); Elem; Elem = mjs_nextElement(Spec.Get(), Elem))
	{
		mjsActuator* Act = mjs_asActuator(Elem);
		const mjsDefault* Def = mjs_getDefault(Elem);
		const FString ElemName = GetElemName(Elem);
		const FName NodeVarName = ElemName.IsEmpty() ? NAME_None : FName(*ElemName);
		USCS_Node* Node = SCS->CreateNode(UMJActuatorComponent::StaticClass(), NodeVarName);
		UMJActuatorComponent* ActComp = CastChecked<UMJActuatorComponent>(Node->ComponentTemplate);
		ActComp->ElementName = ElemName;
		ActComp->DefaultClass = GetDefaultClassName(Elem);
		if (ActComp->DefaultClass.IsEmpty() || ActComp->DefaultClass == TEXT("main"))
		{
			FMJSpecMarshal::ExtractActuatorShortcut(Act, ActComp->Shortcut, ActComp->ShortcutParams);
		}
		FMJSpecMarshal::FromMJSpec(Act, ActComp->Spec, Def ? Def->actuator : nullptr, bDeg);
		if (const char* Tgt = mjs_getString(Act->target)) { if (Tgt[0]) { ActComp->TargetRef.FallbackName = UTF8_TO_TCHAR(Tgt); } }
		SCS->AddNode(Node);
	}

	// Sensors
	for (mjsElement* Elem = mjs_firstElement(Spec.Get(), mjOBJ_SENSOR); Elem; Elem = mjs_nextElement(Spec.Get(), Elem))
	{
		mjsSensor* Sen = mjs_asSensor(Elem);
		const FString ElemName = GetElemName(Elem);
		const FName NodeVarName = ElemName.IsEmpty() ? NAME_None : FName(*ElemName);
		USCS_Node* Node = SCS->CreateNode(UMJSensorComponent::StaticClass(), NodeVarName);
		UMJSensorComponent* SenComp = CastChecked<UMJSensorComponent>(Node->ComponentTemplate);
		SenComp->ElementName = ElemName;
		SenComp->DefaultClass = GetDefaultClassName(Elem);
		FMJSpecMarshal::FromMJSpec(Sen, SenComp->Spec, nullptr, bDeg);
		if (const char* Obj = mjs_getString(Sen->objname)) { if (Obj[0]) { SenComp->ObjRef.FallbackName = UTF8_TO_TCHAR(Obj); } }
		if (const char* Ref = mjs_getString(Sen->refname)) { if (Ref[0]) { SenComp->RefObjRef.FallbackName = UTF8_TO_TCHAR(Ref); } }
		SCS->AddNode(Node);
	}

	return true;
}

bool FMJXmlImporterExporter::ExportActorToXML(AActor* SourceActor, FString& OutXml, FString& OutError)
{
	if (!SourceActor)
	{
		OutError = TEXT("SourceActor is null");
		return false;
	}
	UMJModelComponent* ModelComp = SourceActor->FindComponentByClass<UMJModelComponent>();
	if (!ModelComp)
	{
		OutError = TEXT("SourceActor has no UMJModelComponent");
		return false;
	}
	if (!ModelComp->BuildSpecFromActor(OutError))
	{
		return false;
	}
	return ModelComp->ExportXML(OutXml, OutError, false, false, false);
}
