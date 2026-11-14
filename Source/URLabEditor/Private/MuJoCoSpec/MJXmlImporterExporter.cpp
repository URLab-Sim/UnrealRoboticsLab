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


void ResolveMJTransform(
	const mjSpec* Spec,
	const double Pos[3],
	const double Quat[4],
	const mjsOrientation& Alt,
	FTransform& OutXform)
{
	double ResolvedQuat[4] = { Quat[0], Quat[1], Quat[2], Quat[3] };
	if (Alt.type != mjORIENTATION_QUAT && Spec)
	{
		mjs_resolveOrientation(ResolvedQuat, Spec->compiler.degree, Spec->compiler.eulerseq, &Alt);
	}
	FMJUnitConverter::MJTransformToUE(Pos, ResolvedQuat, OutXform);
}

EMJWrap FromMJWrapType(mjtWrap T)
{
	switch (T)
	{
	case mjWRAP_JOINT: return EMJWrap::Joint;
	case mjWRAP_PULLEY: return EMJWrap::Pulley;
	case mjWRAP_SITE: return EMJWrap::Site;
	case mjWRAP_SPHERE: return EMJWrap::Sphere;
	case mjWRAP_CYLINDER: return EMJWrap::Cylinder;
	default: return EMJWrap::None;
	}
}

void ExtractDefaults(mjSpec* Spec, UMJModelComponent* ModelComp, bool bDeg)
{
	ModelComp->DefaultClasses.Reset();
	TSet<const mjsDefault*> SeenDefaults;

	const mjsDefault* MainDefPtr = mjs_getSpecDefault(Spec);
	auto AddDefaultEntry = [&](const mjsDefault* Def)
	{
		if (!Def || !Def->element || SeenDefaults.Contains(Def)) { return; }
		SeenDefaults.Add(Def);

		FString ClassName = GetElemName(Def->element);
		if (ClassName.IsEmpty())
		{
			ClassName = TEXT("main");
		}

		const mjsDefault* RefDef = (Def != MainDefPtr && ClassName != TEXT("main")) ? MainDefPtr : nullptr;

		FMJDefaultClassEntry Entry;
		Entry.ClassName = ClassName;
		FMJSpecMarshal::FromMJSpec(Def->geom, Entry.Geom, RefDef ? RefDef->geom : nullptr, bDeg);
		FMJSpecMarshal::FromMJSpec(Def->joint, Entry.Joint, RefDef ? RefDef->joint : nullptr, bDeg);
		FMJSpecMarshal::FromMJSpec(Def->site, Entry.Site, RefDef ? RefDef->site : nullptr, bDeg);
		FMJSpecMarshal::FromMJSpec(Def->actuator, Entry.Actuator, RefDef ? RefDef->actuator : nullptr, bDeg);
		FMJSpecMarshal::FromMJSpec(Def->camera, Entry.Camera, RefDef ? RefDef->camera : nullptr, bDeg);
		FMJSpecMarshal::FromMJSpec(Def->light, Entry.Light, RefDef ? RefDef->light : nullptr, bDeg);
		FMJSpecMarshal::FromMJSpec(Def->tendon, Entry.Tendon, RefDef ? RefDef->tendon : nullptr, bDeg);
		FMJSpecMarshal::FromMJSpec(Def->mesh, Entry.Mesh, RefDef ? RefDef->mesh : nullptr, bDeg);
		FMJSpecMarshal::FromMJSpec(Def->texture, Entry.Texture, RefDef ? RefDef->texture : nullptr, bDeg);
		FMJSpecMarshal::FromMJSpec(Def->material, Entry.Material, RefDef ? RefDef->material : nullptr, bDeg);
		{
			mjsGeom DefGeom; mjs_defaultGeom(&DefGeom);
			FTransform GX;
			ResolveMJTransform(Spec, Def->geom->pos, Def->geom->quat, Def->geom->alt, GX);
			Entry.bOverride_GeomPos = (Def->geom->pos[0] != DefGeom.pos[0] || Def->geom->pos[1] != DefGeom.pos[1] || Def->geom->pos[2] != DefGeom.pos[2]);
			Entry.GeomPos = GX.GetLocation();
			Entry.bOverride_GeomQuat = (Def->geom->alt.type != mjORIENTATION_QUAT || Def->geom->quat[0] != DefGeom.quat[0] || Def->geom->quat[1] != DefGeom.quat[1] || Def->geom->quat[2] != DefGeom.quat[2] || Def->geom->quat[3] != DefGeom.quat[3]);
			Entry.GeomQuat = GX.GetRotation();
		}
		{
			mjsSite DefSite; mjs_defaultSite(&DefSite);
			FTransform SX;
			ResolveMJTransform(Spec, Def->site->pos, Def->site->quat, Def->site->alt, SX);
			Entry.bOverride_SitePos = (Def->site->pos[0] != DefSite.pos[0] || Def->site->pos[1] != DefSite.pos[1] || Def->site->pos[2] != DefSite.pos[2]);
			Entry.SitePos = SX.GetLocation();
			Entry.bOverride_SiteQuat = (Def->site->alt.type != mjORIENTATION_QUAT || Def->site->quat[0] != DefSite.quat[0] || Def->site->quat[1] != DefSite.quat[1] || Def->site->quat[2] != DefSite.quat[2] || Def->site->quat[3] != DefSite.quat[3]);
			Entry.SiteQuat = SX.GetRotation();
		}
		ModelComp->DefaultClasses.Add(MoveTemp(Entry));
	};

	if (const mjsDefault* MainDef = mjs_getSpecDefault(Spec))
	{
		AddDefaultEntry(MainDef);
	}
	const mjtObj ElemTypes[] = {
		mjOBJ_BODY, mjOBJ_GEOM, mjOBJ_JOINT, mjOBJ_SITE, mjOBJ_CAMERA,
		mjOBJ_LIGHT, mjOBJ_ACTUATOR, mjOBJ_TENDON, mjOBJ_MESH, mjOBJ_MATERIAL,
		mjOBJ_TEXTURE, mjOBJ_PAIR, mjOBJ_EQUALITY
	};
	for (mjtObj T : ElemTypes)
	{
		for (mjsElement* Elem = mjs_firstElement(Spec, T); Elem; Elem = mjs_nextElement(Spec, Elem))
		{
			if (const mjsDefault* Def = mjs_getDefault(Elem))
			{
				AddDefaultEntry(Def);
			}
		}
	}
}

void ExtractModelSettingsAndAssets(mjSpec* Spec, UMJModelComponent* ModelComp, bool bDeg)
{
	if (!Spec || !ModelComp) { return; }

	if (const char* MName = mjs_getString(Spec->modelname))
	{
		if (MName[0]) { ModelComp->ModelName = UTF8_TO_TCHAR(MName); }
	}
	ModelComp->bCompilerDegree = bDeg;

	// Compiler settings
	ModelComp->CompilerSettings.bEnabled = true;
	ModelComp->CompilerSettings.bAutolimits = (Spec->compiler.autolimits != 0);
	ModelComp->CompilerSettings.BoundMass = Spec->compiler.boundmass;
	ModelComp->CompilerSettings.BoundInertia = Spec->compiler.boundinertia;
	ModelComp->CompilerSettings.SetTotalMass = Spec->compiler.settotalmass;
	ModelComp->CompilerSettings.bBalanceInertia = (Spec->compiler.balanceinertia != 0);
	ModelComp->CompilerSettings.bFitAabb = (Spec->compiler.fitaabb != 0);
	ModelComp->CompilerSettings.bDiscardVisual = (Spec->compiler.discardvisual != 0);
	ModelComp->CompilerSettings.bFuseStatic = (Spec->compiler.fusestatic != 0);
	ModelComp->CompilerSettings.InertiaFromGeom = static_cast<int32>(Spec->compiler.inertiafromgeom);
	ModelComp->CompilerSettings.InertiaGroupRangeMin = Spec->compiler.inertiagrouprange[0];
	ModelComp->CompilerSettings.InertiaGroupRangeMax = Spec->compiler.inertiagrouprange[1];
	ModelComp->CompilerSettings.bAlignFree = (Spec->compiler.alignfree != 0);
	{
		char SeqBuf[4] = { Spec->compiler.eulerseq[0], Spec->compiler.eulerseq[1], Spec->compiler.eulerseq[2], '\0' };
		ModelComp->CompilerSettings.EulerSeq = UTF8_TO_TCHAR(SeqBuf);
	}
	if (const char* MD = mjs_getString(Spec->compiler.meshdir))
	{
		ModelComp->CompilerSettings.MeshDir = UTF8_TO_TCHAR(MD);
	}
	if (const char* TD = mjs_getString(Spec->compiler.texturedir))
	{
		ModelComp->CompilerSettings.TextureDir = UTF8_TO_TCHAR(TD);
	}

	// Option settings
	ModelComp->OptionSettings.bEnabled = true;
	ModelComp->OptionSettings.Timestep = Spec->option.timestep;
	ModelComp->OptionSettings.ImpRatio = Spec->option.impratio;
	ModelComp->OptionSettings.Tolerance = Spec->option.tolerance;
	ModelComp->OptionSettings.LsTolerance = Spec->option.ls_tolerance;
	ModelComp->OptionSettings.NoslipTolerance = Spec->option.noslip_tolerance;
	ModelComp->OptionSettings.CcdTolerance = Spec->option.ccd_tolerance;
	ModelComp->OptionSettings.SleepTolerance = Spec->option.sleep_tolerance;
	ModelComp->OptionSettings.Gravity = FVector(Spec->option.gravity[0], Spec->option.gravity[1], Spec->option.gravity[2]);
	ModelComp->OptionSettings.Wind = FVector(Spec->option.wind[0], Spec->option.wind[1], Spec->option.wind[2]);
	ModelComp->OptionSettings.Magnetic = FVector(Spec->option.magnetic[0], Spec->option.magnetic[1], Spec->option.magnetic[2]);
	ModelComp->OptionSettings.Density = Spec->option.density;
	ModelComp->OptionSettings.Viscosity = Spec->option.viscosity;
	ModelComp->OptionSettings.Integrator = Spec->option.integrator;
	ModelComp->OptionSettings.Cone = Spec->option.cone;
	ModelComp->OptionSettings.Jacobian = Spec->option.jacobian;
	ModelComp->OptionSettings.Solver = Spec->option.solver;
	ModelComp->OptionSettings.Iterations = Spec->option.iterations;
	ModelComp->OptionSettings.LsIterations = Spec->option.ls_iterations;
	ModelComp->OptionSettings.NoslipIterations = Spec->option.noslip_iterations;
	ModelComp->OptionSettings.CcdIterations = Spec->option.ccd_iterations;
	ModelComp->OptionSettings.DisableFlags = Spec->option.disableflags;
	ModelComp->OptionSettings.EnableFlags = Spec->option.enableflags;
	ModelComp->OptionSettings.DisableActuator = Spec->option.disableactuator;
	ModelComp->OptionSettings.SdfInitPoints = Spec->option.sdf_initpoints;
	ModelComp->OptionSettings.SdfIterations = Spec->option.sdf_iterations;

	// Size settings
	ModelComp->SizeSettings.bEnabled = true;
	ModelComp->SizeSettings.bStripPath = (Spec->strippath != 0);
	ModelComp->SizeSettings.Memory = static_cast<int64>(Spec->memory);
	ModelComp->SizeSettings.Nemax = Spec->nemax;
	ModelComp->SizeSettings.Nuserdata = Spec->nuserdata;
	ModelComp->SizeSettings.NuserBody = Spec->nuser_body;
	ModelComp->SizeSettings.NuserJnt = Spec->nuser_jnt;
	ModelComp->SizeSettings.NuserGeom = Spec->nuser_geom;
	ModelComp->SizeSettings.NuserSite = Spec->nuser_site;
	ModelComp->SizeSettings.NuserCam = Spec->nuser_cam;
	ModelComp->SizeSettings.NuserTendon = Spec->nuser_tendon;
	ModelComp->SizeSettings.NuserActuator = Spec->nuser_actuator;
	ModelComp->SizeSettings.NuserSensor = Spec->nuser_sensor;
	ModelComp->SizeSettings.Nkey = Spec->nkey;
	ModelComp->SizeSettings.Njmax = Spec->njmax;
	ModelComp->SizeSettings.Nconmax = Spec->nconmax;
	ModelComp->SizeSettings.Nstack = static_cast<int64>(Spec->nstack);

	ExtractDefaults(Spec, ModelComp, bDeg);

	// Textures
	ModelComp->Textures.Reset();
	for (mjsElement* Elem = mjs_firstElement(Spec, mjOBJ_TEXTURE); Elem; Elem = mjs_nextElement(Spec, Elem))
	{
		mjsTexture* Tex = mjs_asTexture(Elem);
		if (!Tex) { continue; }
		const mjsDefault* Def = mjs_getDefault(Elem);
		FMJTextureAssetEntry Entry;
		Entry.Name = GetElemName(Elem);
		Entry.DefaultClass = GetDefaultClassName(Elem);
		if (const char* F = mjs_getString(Tex->file)) { if (F[0]) { Entry.File = UTF8_TO_TCHAR(F); } }
		if (const char* CT = mjs_getString(Tex->content_type)) { if (CT[0]) { Entry.ContentType = UTF8_TO_TCHAR(CT); } }
		for (int32 i = 0; i < 6; ++i)
		{
			const char* CF = mjs_getInStringVec(Tex->cubefiles, i);
			if (!CF) { break; }
			Entry.CubeFiles.Add(UTF8_TO_TCHAR(CF));
		}
		FMJSpecMarshal::FromMJSpec(Tex, Entry.Spec, Def ? Def->texture : nullptr, bDeg);
		ModelComp->Textures.Add(MoveTemp(Entry));
	}

	// Materials
	ModelComp->Materials.Reset();
	for (mjsElement* Elem = mjs_firstElement(Spec, mjOBJ_MATERIAL); Elem; Elem = mjs_nextElement(Spec, Elem))
	{
		mjsMaterial* Mat = mjs_asMaterial(Elem);
		if (!Mat) { continue; }
		const mjsDefault* Def = mjs_getDefault(Elem);
		FMJMaterialAssetEntry Entry;
		Entry.Name = GetElemName(Elem);
		Entry.DefaultClass = GetDefaultClassName(Elem);
		for (int32 i = 0; i < mjNTEXROLE; ++i)
		{
			const char* T = mjs_getInStringVec(Mat->textures, i);
			if (!T) { break; }
			Entry.Textures.Add(UTF8_TO_TCHAR(T));
		}
		FMJSpecMarshal::FromMJSpec(Mat, Entry.Spec, Def ? Def->material : nullptr, bDeg);
		ModelComp->Materials.Add(MoveTemp(Entry));
	}

	// Meshes
	ModelComp->Meshes.Reset();
	for (mjsElement* Elem = mjs_firstElement(Spec, mjOBJ_MESH); Elem; Elem = mjs_nextElement(Spec, Elem))
	{
		mjsMesh* Mesh = mjs_asMesh(Elem);
		if (!Mesh) { continue; }
		const mjsDefault* Def = mjs_getDefault(Elem);
		FMJMeshAssetEntry Entry;
		Entry.Name = GetElemName(Elem);
		Entry.DefaultClass = GetDefaultClassName(Elem);
		if (const char* F = mjs_getString(Mesh->file)) { if (F[0]) { Entry.File = UTF8_TO_TCHAR(F); } }
		if (const char* CT = mjs_getString(Mesh->content_type)) { if (CT[0]) { Entry.ContentType = UTF8_TO_TCHAR(CT); } }
		if (Mesh->uservert && !Mesh->uservert->empty()) { Entry.Userverts.Append(Mesh->uservert->data(), static_cast<int32>(Mesh->uservert->size())); }
		if (Mesh->usernormal && !Mesh->usernormal->empty()) { Entry.Usernormals.Append(Mesh->usernormal->data(), static_cast<int32>(Mesh->usernormal->size())); }
		if (Mesh->usertexcoord && !Mesh->usertexcoord->empty()) { Entry.Usertexcoords.Append(Mesh->usertexcoord->data(), static_cast<int32>(Mesh->usertexcoord->size())); }
		if (Mesh->userface && !Mesh->userface->empty()) { Entry.Userfaces.Append(Mesh->userface->data(), static_cast<int32>(Mesh->userface->size())); }
		if (Mesh->userfacenormal && !Mesh->userfacenormal->empty()) { Entry.Userfacenormals.Append(Mesh->userfacenormal->data(), static_cast<int32>(Mesh->userfacenormal->size())); }
		if (Mesh->userfacetexcoord && !Mesh->userfacetexcoord->empty()) { Entry.Userfacetexcoords.Append(Mesh->userfacetexcoord->data(), static_cast<int32>(Mesh->userfacetexcoord->size())); }
		if (Mesh->usertet && !Mesh->usertet->empty()) { Entry.Usertets.Append(Mesh->usertet->data(), static_cast<int32>(Mesh->usertet->size())); }
		FMJSpecMarshal::FromMJSpec(Mesh, Entry.Spec, Def ? Def->mesh : nullptr, bDeg);
		ModelComp->Meshes.Add(MoveTemp(Entry));
	}

	// HFields
	ModelComp->HFields.Reset();
	for (mjsElement* Elem = mjs_firstElement(Spec, mjOBJ_HFIELD); Elem; Elem = mjs_nextElement(Spec, Elem))
	{
		mjsHField* HF = mjs_asHField(Elem);
		if (!HF) { continue; }
		FMJHFieldAssetEntry Entry;
		Entry.Name = GetElemName(Elem);
		if (const char* F = mjs_getString(HF->file)) { if (F[0]) { Entry.File = UTF8_TO_TCHAR(F); } }
		if (const char* CT = mjs_getString(HF->content_type)) { if (CT[0]) { Entry.ContentType = UTF8_TO_TCHAR(CT); } }
		Entry.Size.Append(HF->size, 4);
		Entry.Nrow = HF->nrow;
		Entry.Ncol = HF->ncol;
		if (HF->userdata && !HF->userdata->empty()) { Entry.Userdata.Append(HF->userdata->data(), static_cast<int32>(HF->userdata->size())); }
		ModelComp->HFields.Add(MoveTemp(Entry));
	}

	// Equalities
	ModelComp->Equalities.Reset();
	for (mjsElement* Elem = mjs_firstElement(Spec, mjOBJ_EQUALITY); Elem; Elem = mjs_nextElement(Spec, Elem))
	{
		mjsEquality* Eq = mjs_asEquality(Elem);
		if (!Eq) { continue; }
		FMJEqualityEntry Entry;
		Entry.Name = GetElemName(Elem);
		Entry.DefaultClass = GetDefaultClassName(Elem);
		Entry.Type = static_cast<int32>(Eq->type);
		Entry.ObjType = static_cast<int32>(Eq->objtype);
		Entry.bActive = (Eq->active != 0);
		if (const char* N1 = mjs_getString(Eq->name1)) { if (N1[0]) { Entry.Name1 = UTF8_TO_TCHAR(N1); } }
		if (const char* N2 = mjs_getString(Eq->name2)) { if (N2[0]) { Entry.Name2 = UTF8_TO_TCHAR(N2); } }
		Entry.Data.Append(Eq->data, mjNEQDATA);
		Entry.Solref.Append(Eq->solref, mjNREF);
		Entry.Solimp.Append(Eq->solimp, mjNIMP);
		ModelComp->Equalities.Add(MoveTemp(Entry));
	}

	// Excludes
	ModelComp->Excludes.Reset();
	for (mjsElement* Elem = mjs_firstElement(Spec, mjOBJ_EXCLUDE); Elem; Elem = mjs_nextElement(Spec, Elem))
	{
		mjsExclude* Exc = mjs_asExclude(Elem);
		if (!Exc) { continue; }
		FMJExcludeEntry Entry;
		Entry.Name = GetElemName(Elem);
		if (const char* B1 = mjs_getString(Exc->bodyname1)) { if (B1[0]) { Entry.BodyName1 = UTF8_TO_TCHAR(B1); } }
		if (const char* B2 = mjs_getString(Exc->bodyname2)) { if (B2[0]) { Entry.BodyName2 = UTF8_TO_TCHAR(B2); } }
		ModelComp->Excludes.Add(MoveTemp(Entry));
	}

	// Pairs
	ModelComp->Pairs.Reset();
	for (mjsElement* Elem = mjs_firstElement(Spec, mjOBJ_PAIR); Elem; Elem = mjs_nextElement(Spec, Elem))
	{
		mjsPair* Pair = mjs_asPair(Elem);
		if (!Pair) { continue; }
		FMJPairEntry Entry;
		Entry.Name = GetElemName(Elem);
		Entry.DefaultClass = GetDefaultClassName(Elem);
		if (const char* G1 = mjs_getString(Pair->geomname1)) { if (G1[0]) { Entry.GeomName1 = UTF8_TO_TCHAR(G1); } }
		if (const char* G2 = mjs_getString(Pair->geomname2)) { if (G2[0]) { Entry.GeomName2 = UTF8_TO_TCHAR(G2); } }
		Entry.Condim = Pair->condim;
		Entry.Margin = Pair->margin;
		Entry.Gap = Pair->gap;
		Entry.Adhesion = Pair->adhesion;
		Entry.Solref.Append(Pair->solref, mjNREF);
		Entry.SolrefFriction.Append(Pair->solreffriction, mjNREF);
		Entry.Solimp.Append(Pair->solimp, mjNIMP);
		Entry.Friction.Append(Pair->friction, 5);
		ModelComp->Pairs.Add(MoveTemp(Entry));
	}

	// Flexes
	ModelComp->Flexes.Reset();
	for (mjsElement* Elem = mjs_firstElement(Spec, mjOBJ_FLEX); Elem; Elem = mjs_nextElement(Spec, Elem))
	{
		mjsFlex* Flx = mjs_asFlex(Elem);
		if (!Flx) { continue; }
		FMJFlexEntry Entry;
		Entry.Name = GetElemName(Elem);
		Entry.Contype = Flx->contype;
		Entry.Conaffinity = Flx->conaffinity;
		Entry.Condim = Flx->condim;
		Entry.Priority = Flx->priority;
		Entry.Friction.Append(Flx->friction, 3);
		Entry.Solmix = Flx->solmix;
		Entry.Solref.Append(Flx->solref, mjNREF);
		Entry.Solimp.Append(Flx->solimp, mjNIMP);
		Entry.Margin = Flx->margin;
		Entry.Gap = Flx->gap;
		Entry.Dim = Flx->dim;
		Entry.Radius = Flx->radius;
		Entry.Size.Append(Flx->size, 3);
		Entry.bFlatskin = (Flx->flatskin != 0);
		Entry.Selfcollide = static_cast<int32>(Flx->selfcollide);
		Entry.Passive = Flx->passive;
		Entry.Activelayers = Flx->activelayers;
		Entry.Group = Flx->group;
		Entry.Edgestiffness = Flx->edgestiffness;
		Entry.Edgedamping = Flx->edgedamping;
		Entry.Rgba = FLinearColor(Flx->rgba[0], Flx->rgba[1], Flx->rgba[2], Flx->rgba[3]);
		if (const char* M = mjs_getInStringVec(Flx->material, 0)) { if (M[0]) { Entry.Material = UTF8_TO_TCHAR(M); } }
		Entry.Young = Flx->young;
		Entry.Poisson = Flx->poisson;
		Entry.Shear = Flx->shear;
		Entry.Damping = Flx->damping;
		Entry.Thickness = Flx->thickness;
		Entry.Stressyield.Append(Flx->stressyield, 2);
		Entry.Elastic2d = Flx->elastic2d;
		Entry.Elastic3d = Flx->elastic3d;
		Entry.Section = static_cast<int32>(Flx->section);
		Entry.Sectionsize.Append(Flx->sectionsize, 2);
		Entry.Cellcount.Append(Flx->cellcount, 3);
		Entry.Order = Flx->order;
		for (int32 i = 0; ; ++i)
		{
			const char* S = mjs_getInStringVec(Flx->nodebody, i);
			if (!S) { break; }
			Entry.Nodebody.Add(UTF8_TO_TCHAR(S));
		}
		for (int32 i = 0; ; ++i)
		{
			const char* S = mjs_getInStringVec(Flx->vertbody, i);
			if (!S) { break; }
			Entry.Vertbody.Add(UTF8_TO_TCHAR(S));
		}
		if (Flx->node) { int32 N = 0; const double* P = mjs_getDouble(Flx->node, &N); if (P && N > 0) { Entry.Node.Append(P, N); } }
		if (Flx->vert) { int32 N = 0; const double* P = mjs_getDouble(Flx->vert, &N); if (P && N > 0) { Entry.Vert.Append(P, N); } }
		if (Flx->elem && !Flx->elem->empty()) { Entry.Elem.Append(Flx->elem->data(), static_cast<int32>(Flx->elem->size())); }
		if (Flx->texcoord && !Flx->texcoord->empty()) { Entry.Texcoord.Append(Flx->texcoord->data(), static_cast<int32>(Flx->texcoord->size())); }
		if (Flx->elemtexcoord && !Flx->elemtexcoord->empty()) { Entry.Elemtexcoord.Append(Flx->elemtexcoord->data(), static_cast<int32>(Flx->elemtexcoord->size())); }
		ModelComp->Flexes.Add(MoveTemp(Entry));
	}

	// Tuples
	ModelComp->Tuples.Reset();
	for (mjsElement* Elem = mjs_firstElement(Spec, mjOBJ_TUPLE); Elem; Elem = mjs_nextElement(Spec, Elem))
	{
		mjsTuple* Tup = mjs_asTuple(Elem);
		if (!Tup) { continue; }
		FMJTupleEntry Entry;
		Entry.Name = GetElemName(Elem);
		if (Tup->objtype && !Tup->objtype->empty()) { Entry.Objtype.Append(Tup->objtype->data(), static_cast<int32>(Tup->objtype->size())); }
		for (int32 i = 0; ; ++i)
		{
			const char* S = mjs_getInStringVec(Tup->objname, i);
			if (!S) { break; }
			Entry.Objname.Add(UTF8_TO_TCHAR(S));
		}
		if (Tup->objprm && !Tup->objprm->empty()) { Entry.Objprm.Append(Tup->objprm->data(), static_cast<int32>(Tup->objprm->size())); }
		ModelComp->Tuples.Add(MoveTemp(Entry));
	}
}

void PopulateTendonComponent(mjsTendon* Ten, UMJTendonComponent* TenComp, const mjsDefault* Def, bool bDeg)
{
	if (!Ten || !TenComp) { return; }
	TenComp->ElementName = GetElemName(Ten->element);
	TenComp->DefaultClass = GetDefaultClassName(Ten->element);
	FMJSpecMarshal::FromMJSpec(Ten, TenComp->Spec, Def ? Def->tendon : nullptr, bDeg);
	if (const char* Mat = mjs_getString(Ten->material))
	{
		if (Mat[0]) { TenComp->MaterialRef.FallbackName = UTF8_TO_TCHAR(Mat); }
	}
	TenComp->Wraps.Reset();
	const int32 NumWraps = mjs_getWrapNum(Ten);
	for (int32 i = 0; i < NumWraps; ++i)
	{
		const mjsWrap* Wrap = mjs_getWrap(Ten, i);
		if (!Wrap) { continue; }
		FMJTendonWrapEntry Entry;
		Entry.WrapType = FromMJWrapType(Wrap->type);
		if (mjsElement* Target = mjs_getWrapTarget(Wrap))
		{
			Entry.TargetRef.FallbackName = GetElemName(Target);
		}
		if (mjsSite* SideSite = mjs_getWrapSideSite(Wrap))
		{
			Entry.SideSiteRef.FallbackName = GetElemName(SideSite->element);
		}
		if (Wrap->type == mjWRAP_JOINT)
		{
			Entry.CoefOrDivisor = mjs_getWrapCoef(Wrap);
		}
		else if (Wrap->type == mjWRAP_PULLEY)
		{
			Entry.CoefOrDivisor = mjs_getWrapDivisor(Wrap);
		}
		TenComp->Wraps.Add(MoveTemp(Entry));
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
	const mjSpec* Spec,
	AActor* Owner,
	mjsBody* Body,
	USceneComponent* ParentComp,
	bool bDeg)
{
	if (!Body || !Owner) { return; }

	// First pass: create UMJFrameComponent for all immediate frames in this body, deferring attachment so XML child order is preserved
	TMap<mjsFrame*, UMJFrameComponent*> FrameMap;
	TMap<mjsFrame*, mjsFrame*> ParentFrameMap;
	TArray<mjsFrame*> FrameOrder;
	for (mjsElement* Elem = mjs_firstChild(Body, mjOBJ_FRAME, 0); Elem; Elem = mjs_nextChild(Body, Elem, 0))
	{
		mjsFrame* Frame = mjs_asFrame(Elem);
		if (!Frame) { continue; }
		UMJFrameComponent* FrameComp = NewObject<UMJFrameComponent>(Owner, NAME_None, RF_Transactional);
		FrameComp->ElementName = GetElemName(Elem);
		FTransform Xform;
		ResolveMJTransform(Spec, Frame->pos, Frame->quat, Frame->alt, Xform);
		FrameComp->SetImportedTransform(Xform);
		FMJSpecMarshal::FromMJSpec(Frame, FrameComp->Spec, nullptr, bDeg);
		FrameMap.Add(Frame, FrameComp);
		ParentFrameMap.Add(Frame, mjs_getFrame(Elem));
		FrameOrder.Add(Frame);
	}

	TSet<mjsFrame*> AttachedFrames;
	TFunction<USceneComponent*(mjsFrame*)> EnsureFrameAttached = [&](mjsFrame* F) -> USceneComponent*
	{
		if (!F || !FrameMap.Contains(F)) { return ParentComp; }
		UMJFrameComponent* FrameComp = FrameMap[F];
		if (!AttachedFrames.Contains(F))
		{
			AttachedFrames.Add(F);
			USceneComponent* FrameParent = EnsureFrameAttached(ParentFrameMap.FindRef(F));
			FrameComp->RegisterComponent();
			if (FrameParent)
			{
				FrameComp->AttachToComponent(FrameParent, FAttachmentTransformRules::KeepRelativeTransform);
			}
			else if (!Owner->GetRootComponent())
			{
				Owner->SetRootComponent(FrameComp);
			}
			else
			{
				FrameComp->AttachToComponent(Owner->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
			}
			Owner->AddInstanceComponent(FrameComp);
		}
		return FrameComp;
	};

	auto ResolveParent = [&EnsureFrameAttached](mjsElement* Elem) -> USceneComponent*
	{
		return EnsureFrameAttached(mjs_getFrame(Elem));
	};

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
			ResolveMJTransform(Spec, ChildBody->pos, ChildBody->quat, ChildBody->alt, Xform);
			BodyComp->SetImportedTransform(Xform);
			FMJSpecMarshal::FromMJSpec(ChildBody, BodyComp->Spec, nullptr, bDeg);
			ImportBodyChildrenToActor(Spec, Owner, ChildBody, BodyComp, bDeg);
			break;
		}
		case mjOBJ_GEOM:
		{
			mjsGeom* Geom = mjs_asGeom(Elem);
			UMJGeomComponent* GeomComp = CreateSceneComp<UMJGeomComponent>(Owner, AttachParent, ElemName);
			GeomComp->DefaultClass = DefName;
			FTransform Xform;
			ResolveMJTransform(Spec, Geom->pos, Geom->quat, Geom->alt, Xform);
			GeomComp->SetImportedTransform(Xform);
			FMJSpecMarshal::FromMJSpec(Geom, GeomComp->Spec, Def ? Def->geom : nullptr, bDeg);
			if (const char* Mat = mjs_getString(Geom->material)) { if (Mat[0]) { GeomComp->MaterialRef.FallbackName = UTF8_TO_TCHAR(Mat); } }
			if (const char* Mesh = mjs_getString(Geom->meshname)) { if (Mesh[0]) { GeomComp->MeshRef.FallbackName = UTF8_TO_TCHAR(Mesh); } }
			if (const char* HF = mjs_getString(Geom->hfieldname)) { if (HF[0]) { GeomComp->HFieldName = UTF8_TO_TCHAR(HF); } }
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
			ResolveMJTransform(Spec, Site->pos, Site->quat, Site->alt, Xform);
			SiteComp->SetImportedTransform(Xform);
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
			ResolveMJTransform(Spec, Cam->pos, Cam->quat, Cam->alt, Xform);
			CamComp->SetImportedTransform(Xform);
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
	for (mjsFrame* F : FrameOrder)
	{
		EnsureFrameAttached(F);
	}
}

void ImportBodyChildrenToSCS(
	const mjSpec* Spec,
	UBlueprint* BP,
	USimpleConstructionScript* SCS,
	mjsBody* Body,
	USCS_Node* ParentNode,
	bool bDeg)
{
	if (!Body || !SCS) { return; }

	auto AttachNodeTo = [SCS](USCS_Node* Node, USCS_Node* TargetParent)
	{
		if (TargetParent) { TargetParent->AddChildNode(Node); }
		else { SCS->AddNode(Node); }
	};

	// First pass: record frames in this body, creating+attaching SCS nodes lazily so XML child order and SCS name uniqueness are both preserved
	TSet<mjsFrame*> KnownFrames;
	TMap<mjsFrame*, mjsFrame*> ParentFrameMap;
	TArray<mjsFrame*> FrameOrder;
	for (mjsElement* Elem = mjs_firstChild(Body, mjOBJ_FRAME, 0); Elem; Elem = mjs_nextChild(Body, Elem, 0))
	{
		mjsFrame* Frame = mjs_asFrame(Elem);
		if (!Frame) { continue; }
		KnownFrames.Add(Frame);
		ParentFrameMap.Add(Frame, mjs_getFrame(Elem));
		FrameOrder.Add(Frame);
	}

	TMap<mjsFrame*, USCS_Node*> FrameMap;
	TFunction<USCS_Node*(mjsFrame*)> EnsureFrameAttached = [&](mjsFrame* F) -> USCS_Node*
	{
		if (!F || !KnownFrames.Contains(F)) { return ParentNode; }
		if (USCS_Node** Existing = FrameMap.Find(F))
		{
			return *Existing;
		}
		// Reserve entry before recursing in case of unexpected cycles
		FrameMap.Add(F, nullptr);
		USCS_Node* FrameParent = EnsureFrameAttached(ParentFrameMap.FindRef(F));
		const FString ElemName = GetElemName(F->element);
		const FName NodeVarName = ElemName.IsEmpty() ? NAME_None : FName(*ElemName);
		USCS_Node* Node = SCS->CreateNode(UMJFrameComponent::StaticClass(), NodeVarName);
		UMJFrameComponent* Comp = CastChecked<UMJFrameComponent>(Node->ComponentTemplate);
		Comp->ElementName = ElemName;
		FTransform Xform;
		ResolveMJTransform(Spec, F->pos, F->quat, F->alt, Xform);
		Comp->SetImportedTransform(Xform);
		FMJSpecMarshal::FromMJSpec(F, Comp->Spec, nullptr, bDeg);
		AttachNodeTo(Node, FrameParent);
		FrameMap[F] = Node;
		return Node;
	};

	auto ResolveParentNode = [&EnsureFrameAttached](mjsElement* Elem) -> USCS_Node*
	{
		return EnsureFrameAttached(mjs_getFrame(Elem));
	};

	const mjtObj ChildTypes[] = { mjOBJ_BODY, mjOBJ_GEOM, mjOBJ_JOINT, mjOBJ_SITE, mjOBJ_CAMERA, mjOBJ_LIGHT };
	for (mjtObj ChildType : ChildTypes)
	for (mjsElement* Elem = mjs_firstChild(Body, ChildType, 0); Elem; Elem = mjs_nextChild(Body, Elem, 0))
	{
		USCS_Node* AttachParent = ResolveParentNode(Elem);
		const mjsDefault* Def = mjs_getDefault(Elem);
		const FString DefName = GetDefaultClassName(Elem);
		const FString ElemName = GetElemName(Elem);
		const FName NodeVarName = ElemName.IsEmpty() ? NAME_None : FName(*ElemName);

		switch (Elem->elemtype)
		{
		case mjOBJ_BODY:
		{
			mjsBody* ChildBody = mjs_asBody(Elem);
			USCS_Node* Node = SCS->CreateNode(UMJBodyComponent::StaticClass(), NodeVarName);
			UMJBodyComponent* Comp = CastChecked<UMJBodyComponent>(Node->ComponentTemplate);
			Comp->ElementName = ElemName;
			Comp->DefaultClass = DefName;
			FTransform Xform;
			ResolveMJTransform(Spec, ChildBody->pos, ChildBody->quat, ChildBody->alt, Xform);
			Comp->SetImportedTransform(Xform);
			FMJSpecMarshal::FromMJSpec(ChildBody, Comp->Spec, nullptr, bDeg);
			AttachNodeTo(Node, AttachParent);
			ImportBodyChildrenToSCS(Spec, BP, SCS, ChildBody, Node, bDeg);
			break;
		}
		case mjOBJ_GEOM:
		{
			mjsGeom* Geom = mjs_asGeom(Elem);
			USCS_Node* Node = SCS->CreateNode(UMJGeomComponent::StaticClass(), NodeVarName);
			UMJGeomComponent* Comp = CastChecked<UMJGeomComponent>(Node->ComponentTemplate);
			Comp->ElementName = ElemName;
			Comp->DefaultClass = DefName;
			FTransform Xform;
			ResolveMJTransform(Spec, Geom->pos, Geom->quat, Geom->alt, Xform);
			Comp->SetImportedTransform(Xform);
			FMJSpecMarshal::FromMJSpec(Geom, Comp->Spec, Def ? Def->geom : nullptr, bDeg);
			if (const char* Mat = mjs_getString(Geom->material)) { if (Mat[0]) { Comp->MaterialRef.FallbackName = UTF8_TO_TCHAR(Mat); } }
			if (const char* Mesh = mjs_getString(Geom->meshname)) { if (Mesh[0]) { Comp->MeshRef.FallbackName = UTF8_TO_TCHAR(Mesh); } }
			if (const char* HF = mjs_getString(Geom->hfieldname)) { if (HF[0]) { Comp->HFieldName = UTF8_TO_TCHAR(HF); } }
			AttachNodeTo(Node, AttachParent);
			break;
		}
		case mjOBJ_JOINT:
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
			AttachNodeTo(Node, AttachParent);
			break;
		}
		case mjOBJ_SITE:
		{
			mjsSite* Site = mjs_asSite(Elem);
			USCS_Node* Node = SCS->CreateNode(UMJSiteComponent::StaticClass(), NodeVarName);
			UMJSiteComponent* Comp = CastChecked<UMJSiteComponent>(Node->ComponentTemplate);
			Comp->ElementName = ElemName;
			Comp->DefaultClass = DefName;
			FTransform Xform;
			ResolveMJTransform(Spec, Site->pos, Site->quat, Site->alt, Xform);
			Comp->SetImportedTransform(Xform);
			FMJSpecMarshal::FromMJSpec(Site, Comp->Spec, Def ? Def->site : nullptr, bDeg);
			if (const char* Mat = mjs_getString(Site->material)) { if (Mat[0]) { Comp->MaterialRef.FallbackName = UTF8_TO_TCHAR(Mat); } }
			if (const char* Mesh = mjs_getString(Site->meshname)) { if (Mesh[0]) { Comp->MeshRef.FallbackName = UTF8_TO_TCHAR(Mesh); } }
			AttachNodeTo(Node, AttachParent);
			break;
		}
		case mjOBJ_CAMERA:
		{
			mjsCamera* Cam = mjs_asCamera(Elem);
			USCS_Node* Node = SCS->CreateNode(UMJCameraComponent::StaticClass(), NodeVarName);
			UMJCameraComponent* Comp = CastChecked<UMJCameraComponent>(Node->ComponentTemplate);
			Comp->ElementName = ElemName;
			Comp->DefaultClass = DefName;
			FTransform Xform;
			ResolveMJTransform(Spec, Cam->pos, Cam->quat, Cam->alt, Xform);
			Comp->SetImportedTransform(Xform);
			FMJSpecMarshal::FromMJSpec(Cam, Comp->Spec, Def ? Def->camera : nullptr, bDeg);
			if (const char* Tgt = mjs_getString(Cam->targetbody)) { if (Tgt[0]) { Comp->TargetBodyRef.FallbackName = UTF8_TO_TCHAR(Tgt); } }
			AttachNodeTo(Node, AttachParent);
			break;
		}
		case mjOBJ_LIGHT:
		{
			mjsLight* Light = mjs_asLight(Elem);
			USCS_Node* Node = SCS->CreateNode(UMJLightComponent::StaticClass(), NodeVarName);
			UMJLightComponent* Comp = CastChecked<UMJLightComponent>(Node->ComponentTemplate);
			Comp->ElementName = ElemName;
			Comp->DefaultClass = DefName;
			FVector Pos, Dir;
			FMJUnitConverter::MJPositionToUE(Light->pos, Pos);
			FMJUnitConverter::MJDirectionToUE(Light->dir, Dir);
			Comp->SetRelativeLocation(Pos);
			if (!Dir.IsNearlyZero()) { Comp->SetRelativeRotation(Dir.Rotation()); }
			FMJSpecMarshal::FromMJSpec(Light, Comp->Spec, Def ? Def->light : nullptr, bDeg);
			if (const char* Tgt = mjs_getString(Light->targetbody)) { if (Tgt[0]) { Comp->TargetBodyRef.FallbackName = UTF8_TO_TCHAR(Tgt); } }
			if (const char* Tex = mjs_getString(Light->texture)) { if (Tex[0]) { Comp->TextureRef.FallbackName = UTF8_TO_TCHAR(Tex); } }
			AttachNodeTo(Node, AttachParent);
			break;
		}
		default:
			break;
		}
	}
	for (mjsFrame* F : FrameOrder)
	{
		EnsureFrameAttached(F);
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

	const bool bDeg = (Spec->compiler.degree != 0);
	ExtractModelSettingsAndAssets(Spec, ModelComp, bDeg);

	if (!TargetActor->GetRootComponent())
	{
		USceneComponent* Root = NewObject<USceneComponent>(TargetActor, TEXT("Root"), RF_Transactional);
		Root->RegisterComponent();
		TargetActor->SetRootComponent(Root);
		TargetActor->AddInstanceComponent(Root);
	}

	if (mjsBody* WorldBody = mjs_findBody(Spec, "world"))
	{
		ImportBodyChildrenToActor(Spec, TargetActor, WorldBody, TargetActor->GetRootComponent(), bDeg);
	}

	// Tendons
	for (mjsElement* Elem = mjs_firstElement(Spec, mjOBJ_TENDON); Elem; Elem = mjs_nextElement(Spec, Elem))
	{
		mjsTendon* Ten = mjs_asTendon(Elem);
		const mjsDefault* Def = mjs_getDefault(Elem);
		UMJTendonComponent* TenComp = NewObject<UMJTendonComponent>(TargetActor, NAME_None, RF_Transactional);
		PopulateTendonComponent(Ten, TenComp, Def, bDeg);
		TenComp->RegisterComponent();
		TargetActor->AddInstanceComponent(TenComp);
	}

	// Actuators
	for (mjsElement* Elem = mjs_firstElement(Spec, mjOBJ_ACTUATOR); Elem; Elem = mjs_nextElement(Spec, Elem))
	{
		mjsActuator* Act = mjs_asActuator(Elem);
		const mjsDefault* Def = mjs_getDefault(Elem);
		UMJActuatorComponent* ActComp = NewObject<UMJActuatorComponent>(TargetActor, NAME_None, RF_Transactional);
		ActComp->ElementName = GetElemName(Elem);
		ActComp->DefaultClass = GetDefaultClassName(Elem);
		if (ActComp->DefaultClass.IsEmpty() || ActComp->DefaultClass == TEXT("main"))
		{
			FMJSpecMarshal::ExtractActuatorShortcut(Act, ActComp->Shortcut, ActComp->ShortcutParams);
		}
		FMJSpecMarshal::FromMJSpec(Act, ActComp->Spec, Def ? Def->actuator : nullptr, bDeg);
		if (const char* Tgt = mjs_getString(Act->target)) { if (Tgt[0]) { ActComp->TargetRef.FallbackName = UTF8_TO_TCHAR(Tgt); } }
		if (const char* SS = mjs_getString(Act->slidersite)) { if (SS[0]) { ActComp->SliderSite = UTF8_TO_TCHAR(SS); } }
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
	ExtractModelSettingsAndAssets(Spec.Get(), ModelComp, bDeg);
	SCS->AddNode(ModelNode);

	USCS_Node* RootNode = SCS->CreateNode(USceneComponent::StaticClass(), TEXT("Root"));
	SCS->AddNode(RootNode);

	if (mjsBody* WorldBody = mjs_findBody(Spec.Get(), "world"))
	{
		ImportBodyChildrenToSCS(Spec.Get(), Blueprint, SCS, WorldBody, RootNode, bDeg);
	}

	// Tendons
	for (mjsElement* Elem = mjs_firstElement(Spec.Get(), mjOBJ_TENDON); Elem; Elem = mjs_nextElement(Spec.Get(), Elem))
	{
		mjsTendon* Ten = mjs_asTendon(Elem);
		const mjsDefault* Def = mjs_getDefault(Elem);
		const FString ElemName = GetElemName(Elem);
		const FName NodeVarName = ElemName.IsEmpty() ? NAME_None : FName(*ElemName);
		USCS_Node* Node = SCS->CreateNode(UMJTendonComponent::StaticClass(), NodeVarName);
		UMJTendonComponent* TenComp = CastChecked<UMJTendonComponent>(Node->ComponentTemplate);
		PopulateTendonComponent(Ten, TenComp, Def, bDeg);
		SCS->AddNode(Node);
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
		if (const char* SS = mjs_getString(Act->slidersite)) { if (SS[0]) { ActComp->SliderSite = UTF8_TO_TCHAR(SS); } }
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
