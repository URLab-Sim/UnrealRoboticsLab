// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjCompile.h"

#include "MuJoCo/Spec/MjAssetFiles.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

#if URLAB_MJ_GEN
#include "MjReservedNames.h"
#include "MuJoCo/Spec/MjSpecProfile.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#endif

// --- FMjBinding -------------------------------------------------------------- //

void FMjBinding::Add(FEntry Entry)
{
	const int32 Index = Entries.Num();
	ByNode.Add(Entry.Node, Index);
	Entries.Add(MoveTemp(Entry));
}

const FMjBinding::FEntry* FMjBinding::Find(const UMjNodeComponent& Node) const
{
	const int32* Index = ByNode.Find(&Node);
	return Index != nullptr ? &Entries[*Index] : nullptr;
}

TOptional<int32> FMjBinding::Id(const UMjNodeComponent& Node) const
{
	const FEntry* Entry = Find(Node);
	if (Entry == nullptr || Entry->Id < 0)
	{
		return TOptional<int32>();
	}
	return Entry->Id;
}

int32 FMjBinding::ObjTypeOf(const UMjNodeComponent& Node) const
{
	const FEntry* Entry = Find(Node);
	return Entry != nullptr ? Entry->ObjType : mjOBJ_UNKNOWN;
}

FString FMjBinding::NameOf(const UMjNodeComponent& Node) const
{
	const FEntry* Entry = Find(Node);
	return Entry != nullptr ? Entry->Name : FString();
}

namespace
{
/**
 * How many objects of `ObjType` the model holds.
 *
 * MuJoCo publishes the counts as named size fields rather than as a table, so
 * this is the table.
 */
int32 ObjectCountOf(const mjModel* Model, int32 ObjType)
{
	switch (ObjType)
	{
		case mjOBJ_BODY: return Model->nbody;
		case mjOBJ_JOINT: return Model->njnt;
		case mjOBJ_GEOM: return Model->ngeom;
		case mjOBJ_SITE: return Model->nsite;
		case mjOBJ_CAMERA: return Model->ncam;
		case mjOBJ_LIGHT: return Model->nlight;
		case mjOBJ_FLEX: return Model->nflex;
		case mjOBJ_MESH: return Model->nmesh;
		case mjOBJ_SKIN: return Model->nskin;
		case mjOBJ_HFIELD: return Model->nhfield;
		case mjOBJ_TEXTURE: return Model->ntex;
		case mjOBJ_MATERIAL: return Model->nmat;
		case mjOBJ_PAIR: return Model->npair;
		case mjOBJ_EXCLUDE: return Model->nexclude;
		case mjOBJ_EQUALITY: return Model->neq;
		case mjOBJ_TENDON: return Model->ntendon;
		case mjOBJ_ACTUATOR: return Model->nu;
		case mjOBJ_SENSOR: return Model->nsensor;
		case mjOBJ_NUMERIC: return Model->nnumeric;
		case mjOBJ_TEXT: return Model->ntext;
		case mjOBJ_TUPLE: return Model->ntuple;
		case mjOBJ_KEY: return Model->nkey;
		case mjOBJ_PLUGIN: return Model->nplugin;
		default: return 0;
	}
}

/** An id of the expected family, or nothing. Guards every address lookup below. */
TOptional<int32> IdOfType(const FMjBinding& Binding, const UMjNodeComponent& Node, int32 ObjType)
{
	if (Binding.ObjTypeOf(Node) != ObjType)
	{
		return TOptional<int32>();
	}
	return Binding.Id(Node);
}
}  // namespace

TOptional<int32> FMjBinding::QposAdr(const UMjNodeComponent& Joint) const
{
	const TOptional<int32> JointId = IdOfType(*this, Joint, mjOBJ_JOINT);
	if (!JointId.IsSet() || Model == nullptr)
	{
		return TOptional<int32>();
	}
	return Model->jnt_qposadr[*JointId];
}

TOptional<int32> FMjBinding::DofAdr(const UMjNodeComponent& Joint) const
{
	const TOptional<int32> JointId = IdOfType(*this, Joint, mjOBJ_JOINT);
	if (!JointId.IsSet() || Model == nullptr)
	{
		return TOptional<int32>();
	}
	return Model->jnt_dofadr[*JointId];
}

TOptional<int32> FMjBinding::SensorAdr(const UMjNodeComponent& Sensor) const
{
	const TOptional<int32> SensorId = IdOfType(*this, Sensor, mjOBJ_SENSOR);
	if (!SensorId.IsSet() || Model == nullptr)
	{
		return TOptional<int32>();
	}
	return Model->sensor_adr[*SensorId];
}

TOptional<int32> FMjBinding::ActAdr(const UMjNodeComponent& Actuator) const
{
	const TOptional<int32> ActuatorId = IdOfType(*this, Actuator, mjOBJ_ACTUATOR);
	if (!ActuatorId.IsSet() || Model == nullptr)
	{
		return TOptional<int32>();
	}
	const int32 Address = Model->actuator_actadr[*ActuatorId];
	if (Address < 0)
	{
		return TOptional<int32>();
	}
	return Address;
}

TArray<int32> FMjBinding::Find(int32 ObjType, const FString& Glob) const
{
	TArray<int32> Out;
	if (Model == nullptr)
	{
		return Out;
	}
	const int32 Count = ObjectCountOf(Model, ObjType);
	for (int32 Id = 0; Id < Count; ++Id)
	{
		const char* Name = mj_id2name(Model, ObjType, Id);
		if (Name != nullptr && FString(UTF8_TO_TCHAR(Name)).MatchesWildcard(Glob))
		{
			Out.Add(Id);
		}
	}
	return Out;
}

// --- FMjCompiled ------------------------------------------------------------- //

FMjCompiled::~FMjCompiled()
{
	if (Model != nullptr)
	{
		mj_deleteModel(Model);
		Model = nullptr;
	}
}

FMjCompiled::FMjCompiled(FMjCompiled&& Other)
	: Model(Other.Model)
	, Binding(MoveTemp(Other.Binding))
	, Xml(MoveTemp(Other.Xml))
	, ParticipantXml(MoveTemp(Other.ParticipantXml))
	, Errors(MoveTemp(Other.Errors))
{
	Other.Model = nullptr;
}

FMjCompiled& FMjCompiled::operator=(FMjCompiled&& Other)
{
	if (this != &Other)
	{
		if (Model != nullptr)
		{
			mj_deleteModel(Model);
		}
		Model = Other.Model;
		Other.Model = nullptr;
		Binding = MoveTemp(Other.Binding);
		Xml = MoveTemp(Other.Xml);
		ParticipantXml = MoveTemp(Other.ParticipantXml);
		Errors = MoveTemp(Other.Errors);
	}
	return *this;
}

mjModel* FMjCompiled::Release()
{
	mjModel* Out = Model;
	Model = nullptr;
	return Out;
}

// --- The compile ------------------------------------------------------------- //

#if URLAB_MJ_GEN

namespace
{
using namespace urlab::spec;

/**
 * The `mjtObj` family an element compiles into, or `mjOBJ_UNKNOWN`.
 *
 * Section containers (`<asset>`, `<worldbody>`, `<actuator>`, ...) and
 * authoring-only elements (`<default>`, `<frame>`) are deliberately unknown:
 * they exist in the spec and not in the model, so binding them would be
 * inventing an id for something MuJoCo never gave one to.
 */
int32 ObjTypeOfElement(psm::ElementType Type)
{
	switch (Type)
	{
		case psm::ElementType::Body:
			return mjOBJ_BODY;
		case psm::ElementType::Joint:
		case psm::ElementType::FreeJoint:
			return mjOBJ_JOINT;
		case psm::ElementType::Geom:
			return mjOBJ_GEOM;
		case psm::ElementType::Site:
			return mjOBJ_SITE;
		case psm::ElementType::Camera:
			return mjOBJ_CAMERA;
		case psm::ElementType::Light:
			return mjOBJ_LIGHT;
		case psm::ElementType::Mesh:
			return mjOBJ_MESH;
		case psm::ElementType::Skin:
			return mjOBJ_SKIN;
		case psm::ElementType::Hfield:
			return mjOBJ_HFIELD;
		case psm::ElementType::Texture:
			return mjOBJ_TEXTURE;
		case psm::ElementType::Material:
			return mjOBJ_MATERIAL;
		case psm::ElementType::Pair:
			return mjOBJ_PAIR;
		case psm::ElementType::Exclude:
			return mjOBJ_EXCLUDE;
		case psm::ElementType::Connect:
		case psm::ElementType::Weld:
		case psm::ElementType::EqualityJoint:
		case psm::ElementType::EqualityTendon:
		case psm::ElementType::EqualityFlex:
		case psm::ElementType::Flexvert:
		case psm::ElementType::Flexstrain:
			return mjOBJ_EQUALITY;
		case psm::ElementType::Spatial:
		case psm::ElementType::Fixed:
			return mjOBJ_TENDON;
		case psm::ElementType::ActuatorGeneral:
		case psm::ElementType::Motor:
		case psm::ElementType::Position:
		case psm::ElementType::Velocity:
		case psm::ElementType::IntVelocity:
		case psm::ElementType::OrientationActuator:
		case psm::ElementType::Pid:
		case psm::ElementType::Damper:
		case psm::ElementType::Cylinder:
		case psm::ElementType::Muscle:
		case psm::ElementType::Adhesion:
		case psm::ElementType::DcMotor:
		case psm::ElementType::ActuatorPlugin:
			return mjOBJ_ACTUATOR;
		case psm::ElementType::Touch:
		case psm::ElementType::Accelerometer:
		case psm::ElementType::Velocimeter:
		case psm::ElementType::Gyro:
		case psm::ElementType::Force:
		case psm::ElementType::Torque:
		case psm::ElementType::Magnetometer:
		case psm::ElementType::Camprojection:
		case psm::ElementType::Rangefinder:
		case psm::ElementType::Jointpos:
		case psm::ElementType::Jointvel:
		case psm::ElementType::Tendonpos:
		case psm::ElementType::Tendonvel:
		case psm::ElementType::Actuatorpos:
		case psm::ElementType::Actuatorvel:
		case psm::ElementType::Actuatorfrc:
		case psm::ElementType::Jointactuatorfrc:
		case psm::ElementType::Tendonactuatorfrc:
		case psm::ElementType::Ballquat:
		case psm::ElementType::Ballangvel:
		case psm::ElementType::Jointlimitpos:
		case psm::ElementType::Jointlimitvel:
		case psm::ElementType::Jointlimitfrc:
		case psm::ElementType::Tendonlimitpos:
		case psm::ElementType::Tendonlimitvel:
		case psm::ElementType::Tendonlimitfrc:
		case psm::ElementType::Framepos:
		case psm::ElementType::Framequat:
		case psm::ElementType::Framexaxis:
		case psm::ElementType::Frameyaxis:
		case psm::ElementType::Framezaxis:
		case psm::ElementType::Framelinvel:
		case psm::ElementType::Frameangvel:
		case psm::ElementType::Framelinacc:
		case psm::ElementType::Frameangacc:
		case psm::ElementType::Subtreecom:
		case psm::ElementType::Subtreelinvel:
		case psm::ElementType::Subtreeangmom:
		case psm::ElementType::Insidesite:
		case psm::ElementType::Distance:
		case psm::ElementType::Normal:
		case psm::ElementType::Fromto:
		case psm::ElementType::SensorContact:
		case psm::ElementType::EPotential:
		case psm::ElementType::EKinetic:
		case psm::ElementType::Clock:
		case psm::ElementType::Tactile:
		case psm::ElementType::SensorUser:
		case psm::ElementType::SensorPlugin:
			return mjOBJ_SENSOR;
		case psm::ElementType::Numeric:
			return mjOBJ_NUMERIC;
		case psm::ElementType::Text:
			return mjOBJ_TEXT;
		case psm::ElementType::Tuple:
			return mjOBJ_TUPLE;
		case psm::ElementType::Key:
			return mjOBJ_KEY;
		case psm::ElementType::Flex:
			return mjOBJ_FLEX;
		case psm::ElementType::PluginInstance:
			return mjOBJ_PLUGIN;
		default:
			return mjOBJ_UNKNOWN;
	}
}

/** A spec's elements, and the ones that must not be given a name. */
struct FDocNodes
{
	TArray<UMjNodeComponent*> Nodes;

	/**
	 * Elements the auto-namer must leave alone.
	 *
	 * Two kinds, for two different reasons. `<worldbody>` is a `body` that sits
	 * directly under the root and MuJoCo rejects every attribute on it, name
	 * included. Everything under `<default>` is a class partial rather than an
	 * element, and the schema's reduced rows do not carry `name` at all. Both
	 * are decidable only from position, which is why this is collected during
	 * the walk rather than derived from the element's own type.
	 */
	TSet<const UMjNodeComponent*> Unnamable;
};

template <class Adapter>
void CollectNodes(UMjNodeComponent& Node, bool bTopLevel, bool bInDefault, FDocNodes& Out)
{
	using namespace urlab::spec;
	Out.Nodes.Add(&Node);
	if (bTopLevel || bInDefault)
	{
		Out.Unnamable.Add(&Node);
	}

	psm::ElementType Type{};
	const bool bDefaultBelow =
		bInDefault || (gen::ElementTypeOfNode(Node, Type) && Type == psm::ElementType::Default);

	for (const FMjOrderedChild& Child : Adapter::OrderedChildren(Node))
	{
		if (Child.Node != nullptr)
		{
			CollectNodes<Adapter>(*Child.Node, false, bDefaultBelow, Out);
		}
	}
}

FDocNodes NodesOf(const FSpecRef& Spec)
{
	FDocNodes Out;
	UMjNodeComponent* Root = Spec.GetRoot();
	if (Root == nullptr)
	{
		return Out;
	}
#if WITH_EDITOR
	if (Spec.GetGraph() == EMjSpecGraph::Scs)
	{
		UBlueprint* Blueprint = Spec.GetBlueprint();
		if (Blueprint == nullptr)
		{
			return Out;
		}
		FMjScsScope Scope(*Blueprint);
		Out.Nodes.Add(Root);
		Out.Unnamable.Add(Root);
		for (const FMjOrderedChild& Child : FMjScsAdapter::OrderedChildren(*Root))
		{
			if (Child.Node != nullptr)
			{
				CollectNodes<FMjScsAdapter>(*Child.Node, true, false, Out);
			}
		}
		return Out;
	}
#endif
	Out.Nodes.Add(Root);
	Out.Unnamable.Add(Root);
	for (const FMjOrderedChild& Child : FMjInstanceAdapter::OrderedChildren(*Root))
	{
		if (Child.Node != nullptr)
		{
			CollectNodes<FMjInstanceAdapter>(*Child.Node, true, false, Out);
		}
	}
	return Out;
}

/**
 * Reserved names for the unnamed, in the emitted text only.
 *
 * The spec is handed back exactly as it was found: the names exist for the
 * duration of one write and are then taken off again. Authoring them for real
 * would put identity nobody asked for into the user's Blueprint, and it would
 * show up in the next diff of their MJCF.
 */
class FReservedNames
{
public:
	FReservedNames(const TArray<UMjNodeComponent*>& Nodes,
		const TSet<const UMjNodeComponent*>& Unnamable, const FMjCompileOptions& Options)
	{
		// A file-backed asset that authors no name is not anonymous to MuJoCo: it
		// takes the file's basename, and that is the name every material or geom
		// referring to it was written against. Scene assembly prefixes `file=` so
		// two participants cannot collide in the VFS, which moves that derived
		// name and dangles the references. Reserving the basename pins it.
		for (UMjNodeComponent* Node : Nodes)
		{
			if (Node == nullptr || Unnamable.Contains(Node))
			{
				continue;
			}
			if (Node->MjName.IsSet() && !Node->MjName.GetValue().IsEmpty())
			{
				continue;
			}
			const FString Derived = MjAssetElementName(*Node);
			if (Derived.IsEmpty())
			{
				continue;
			}
			Node->MjName = Derived;
			Renamed.Add(Node);
		}

		if (!Options.bAutoName)
		{
			return;
		}
		for (UMjNodeComponent* Node : Nodes)
		{
			psm::ElementType Type{};
			if (Node == nullptr || !gen::ElementTypeOfNode(*Node, Type))
			{
				continue;
			}
			const int32 ObjType = ObjTypeOfElement(Type);
			if (ObjType == mjOBJ_UNKNOWN || Unnamable.Contains(Node))
			{
				continue;
			}
			if (Node->MjName.IsSet() && !Node->MjName.GetValue().IsEmpty())
			{
				continue;
			}
			Node->EnsureSerial();
			Node->MjName = FString::Printf(TEXT("%s%s:%llu"), *Options.AutoNamePrefix,
				UTF8_TO_TCHAR(mju_type2Str(ObjType)), Node->Serial);
			Renamed.Add(Node);
		}
	}

	~FReservedNames()
	{
		for (UMjNodeComponent* Node : Renamed)
		{
			Node->MjName.Reset();
		}
	}

	FReservedNames(const FReservedNames&) = delete;
	FReservedNames& operator=(const FReservedNames&) = delete;

private:
	TArray<UMjNodeComponent*> Renamed;
};

/** Record what each node of `Spec` bound to, under `Prefix`. */
void BindSpec(const FSpecRef& Spec, const FString& Prefix, const mjModel* Model, FMjBinding& Out)
{
	for (UMjNodeComponent* Node : NodesOf(Spec).Nodes)
	{
		psm::ElementType Type{};
		if (Node == nullptr || !gen::ElementTypeOfNode(*Node, Type))
		{
			continue;
		}
		const int32 ObjType = ObjTypeOfElement(Type);
		if (ObjType == mjOBJ_UNKNOWN || !Node->MjName.IsSet())
		{
			continue;
		}
		const FString Name = Prefix + Node->MjName.GetValue();
		Node->EnsureSerial();
		FMjBinding::FEntry Entry;
		Entry.Node = Node;
		Entry.Name = Name;
		Entry.ObjType = ObjType;
		Entry.Id = mj_name2id(Model, ObjType, TCHAR_TO_UTF8(*Name));
		Entry.Serial = Node->Serial;
		Out.Add(MoveTemp(Entry));
	}
}

/** Collect one spec's assets under `Prefix`, as VFS-ready bytes. */
class FVfsCollector final : public IMjAssetSink
{
public:
	TArray<FMjVfsAsset> Assets;

	void OnMesh(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }
	void OnTexture(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }
	void OnHeightField(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }

private:
	void Take(const FMjAssetRequest& Request, const TArray<uint8>& Bytes)
	{
		if (!Request.VfsName.IsEmpty() && Bytes.Num() > 0)
		{
			FMjVfsAsset Asset;
			Asset.Name = Request.VfsName;
			Asset.Bytes = Bytes;
			Assets.Add(MoveTemp(Asset));
		}
	}
};

/** The VFS, the load, and the errors, for text that is already assembled. */
mjModel* LoadFromVfs(const FString& RootXml, const TMap<FString, FString>& ParticipantXml,
	const TArray<FMjVfsAsset>& Assets, TArray<FMjSpecDiagnostic>& OutErrors)
{
	mjVFS Vfs;
	mj_defaultVFS(&Vfs);

	auto AddText = [&Vfs](const FString& Name, const FString& Text) {
		const FTCHARToUTF8 Utf8(*Text);
		mj_addBufferVFS(&Vfs, TCHAR_TO_UTF8(*Name), Utf8.Get(), Utf8.Length());
	};

	const FString RootName = TEXT("__urlab_root.xml");
	AddText(RootName, RootXml);
	for (const TPair<FString, FString>& Spec : ParticipantXml)
	{
		AddText(Spec.Key, Spec.Value);
	}
	for (const FMjVfsAsset& Asset : Assets)
	{
		mj_addBufferVFS(&Vfs, TCHAR_TO_UTF8(*Asset.Name), Asset.Bytes.GetData(), Asset.Bytes.Num());
	}

	char Error[1024] = {0};
	mjModel* Model = mj_loadXML(TCHAR_TO_UTF8(*RootName), &Vfs, Error, sizeof(Error));
	mj_deleteVFS(&Vfs);

	if (Model == nullptr)
	{
		FMjSpecDiagnostic Diagnostic;
		Diagnostic.Message = UTF8_TO_TCHAR(Error);
		OutErrors.Add(MoveTemp(Diagnostic));
	}
	return Model;
}
}  // namespace

// --- Reserved names, for the spec path --------------------------------------- //

namespace urlab::spec
{

/**
 * The same reservation FReservedNames performs, for a caller that builds an
 * mjSpec instead of emitting text.
 *
 * It sits beside the original rather than beneath it because the original is
 * the specification: the two have to produce the same name for the same
 * element, and the way to keep them that way is to write them against the same
 * serial, the same `mju_type2Str` spelling and the same prefix, in one file.
 *
 * One half of the original is missing here, and deliberately: the file-derived
 * name of an unnamed asset. The spec path pins that one onto the ELEMENT, from
 * the same derivation, at the point it rewrites `file` -- so the elements the
 * original reserves a basename for are exactly the ones skipped below, and they
 * end up named the same either way.
 */
FMjReservedNames::FMjReservedNames(const FSpecRef& Spec)
{
	const FMjCompileOptions Options;
	if (!Options.bAutoName)
	{
		return;
	}

	const FDocNodes Tree = NodesOf(Spec);
	for (UMjNodeComponent* Node : Tree.Nodes)
	{
		psm::ElementType Type{};
		if (Node == nullptr || !gen::ElementTypeOfNode(*Node, Type))
		{
			continue;
		}
		const int32 ObjType = ObjTypeOfElement(Type);
		if (ObjType == mjOBJ_UNKNOWN || Tree.Unnamable.Contains(Node))
		{
			continue;
		}
		if (Node->MjName.IsSet() && !Node->MjName.GetValue().IsEmpty())
		{
			continue;
		}
		if (!MjAssetElementName(*Node).IsEmpty())
		{
			continue;
		}
		Node->EnsureSerial();
		Node->MjName = FString::Printf(TEXT("%s%s:%llu"), *Options.AutoNamePrefix,
			UTF8_TO_TCHAR(mju_type2Str(ObjType)), Node->Serial);
		Renamed.Add(Node);
	}
}

FMjReservedNames::~FMjReservedNames()
{
	for (UMjNodeComponent* Node : Renamed)
	{
		Node->MjName.Reset();
	}
}

}  // namespace urlab::spec

FMjCompiled MjCompileSpec(const FSpecRef& Spec, const FMjCompileOptions& Options)
{
	FMjCompiled Out;
	if (!Spec.IsValid())
	{
		FMjSpecDiagnostic Diagnostic;
		Diagnostic.Message = TEXT("no spec to compile");
		Out.Errors.Add(MoveTemp(Diagnostic));
		return Out;
	}

	// Before anything reads `file`, and so before the write below emits it: an
	// element whose asset the user replaced has to have that asset on disk and
	// `file` pointing at it, or the compiler is handed the mesh it displaced.
	MjSyncAssetFiles(Spec);

	FVfsCollector Collector;
	FMjAssetSink Sink(Collector);
	Sink.Collect(Spec);

	{
		const FDocNodes Tree = NodesOf(Spec);
		const FReservedNames Reserved(Tree.Nodes, Tree.Unnamable, Options);
		Out.Xml = Spec.WriteMjcf(&Out.Errors);
		Out.Model = Out.Errors.Num() == 0
			? LoadFromVfs(Out.Xml, Out.ParticipantXml, Collector.Assets, Out.Errors)
			: nullptr;
		if (Out.Model != nullptr)
		{
			BindSpec(Spec, FString(), Out.Model, Out.Binding);
		}
	}

	Out.Binding.SetModel(Out.Model);
	return Out;
}

FMjCompiled MjCompileScene(const FSceneAssembly& Scene, const FMjCompileOptions& Options)
{
	FMjCompiled Out;

	// Every participant plus the scene root is named at once, because the write
	// below emits them all and the binding afterwards has to read the same names
	// back. Held for the whole block for that reason, not for tidiness.
	TArray<UMjNodeComponent*> Nodes;
	TSet<const UMjNodeComponent*> Unnamable;
	for (const FMjSceneParticipant& Participant : Scene.GetParticipants())
	{
		MjSyncAssetFiles(Participant.Spec);
		FDocNodes Tree = NodesOf(Participant.Spec);
		Nodes.Append(Tree.Nodes);
		Unnamable.Append(Tree.Unnamable);
	}

	const TArray<FMjVfsAsset> Assets = Scene.CollectAssets();

	{
		const FReservedNames Reserved(Nodes, Unnamable, Options);
		Out.Xml = Scene.WriteSceneMjcf(Out.ParticipantXml, &Out.Errors);
		Out.Model = Out.Errors.Num() == 0 ? LoadFromVfs(Out.Xml, Out.ParticipantXml, Assets, Out.Errors) : nullptr;
		if (Out.Model != nullptr)
		{
			for (const FMjSceneParticipant& Participant : Scene.GetParticipants())
			{
				BindSpec(Participant.Spec, Participant.Prefix, Out.Model, Out.Binding);
			}
		}
	}

	Out.Binding.SetModel(Out.Model);
	return Out;
}

#else  // URLAB_MJ_GEN

FMjCompiled MjCompileSpec(const FSpecRef&, const FMjCompileOptions&)
{
	FMjCompiled Out;
	FMjSpecDiagnostic Diagnostic;
	Diagnostic.Message = TEXT("built without the generated MuJoCo profile");
	Out.Errors.Add(MoveTemp(Diagnostic));
	return Out;
}

FMjCompiled MjCompileScene(const FSceneAssembly&, const FMjCompileOptions&)
{
	FMjCompiled Out;
	FMjSpecDiagnostic Diagnostic;
	Diagnostic.Message = TEXT("built without the generated MuJoCo profile");
	Out.Errors.Add(MoveTemp(Diagnostic));
	return Out;
}

#endif  // URLAB_MJ_GEN
