// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjSceneSpec.h"

#if URLAB_MJ_GEN

#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/ScopeExit.h"

#include "MjSectionFold.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSceneAssembly.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

namespace urlab::spec
{
namespace
{

using ps::mjcf::ElementType;

/** Where SaveDebugArtifacts puts the assets, relative to the XML beside them. */
const TCHAR* const SceneAssetFolder = TEXT("scene_assets");

/** Bytes and mount names, in the order the sink emitted them. */
class FMjSceneAssetCollector final : public IMjAssetSink
{
public:
	TArray<FMjSceneAsset> Assets;

	void OnMesh(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }
	void OnTexture(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }
	void OnHeightField(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }
	void OnSkin(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }

private:
	void Take(const FMjAssetRequest& Request, const TArray<uint8>& Bytes)
	{
		if (Request.VfsName.IsEmpty() || Bytes.Num() == 0)
		{
			return;
		}
		FMjSceneAsset& Asset = Assets.AddDefaulted_GetRef();
		Asset.Name = Request.VfsName;
		Asset.Bytes = Bytes;
	}
};

/**
 * A diagnostic naming where the offending component was authored.
 *
 * The severity is carried on the entry as well as implied by the array it goes
 * into, so the two say the same thing: a consumer reading one and a consumer
 * reading the other cannot come to different conclusions about the same report.
 */
FMjSpecDiagnostic DiagnosticFor(const UMjNodeComponent* Node, FString Message,
	EMjDiagnosticSeverity Severity = EMjDiagnosticSeverity::Error)
{
	FMjSpecDiagnostic Out;
	Out.Message = MoveTemp(Message);
	Out.Severity = Severity;
	if (Node != nullptr)
	{
		Out.File = Node->SourceFile;
		Out.Line = Node->SourceLine;
	}
	return Out;
}

/** The `file` slot of an asset element, or null when it has none. */
mjString* FileSlotOf(mjsElement* Element)
{
	if (Element == nullptr)
	{
		return nullptr;
	}
	if (mjsMesh* Mesh = mjs_asMesh(Element))
	{
		return Mesh->file;
	}
	if (mjsTexture* Texture = mjs_asTexture(Element))
	{
		return Texture->file;
	}
	if (mjsHField* HField = mjs_asHField(Element))
	{
		return HField->file;
	}
	if (mjsSkin* Skin = mjs_asSkin(Element))
	{
		return Skin->file;
	}
	return nullptr;
}

/** True when nothing has written a name onto this element. */
bool IsUnnamed(mjsElement* Element)
{
	mjString* const Name = mjs_getName(Element);
	const char* const Text = Name != nullptr ? mjs_getString(Name) : nullptr;
	return Text == nullptr || Text[0] == '\0';
}

/**
 * Point a spec's asset references at the names its bytes are mounted under.
 *
 * The sink decides both: what it emits as a request's VfsName is what goes into
 * the VFS, and this writes the same string back onto the element, so the two
 * cannot drift into a lookup that resolves by luck. Prefixed basenames rather
 * than directories, because MuJoCo's VFS falls back to a case-insensitive
 * basename match across every mount: two participants each carrying their own
 * `base.obj` would otherwise silently share whichever was mounted first.
 *
 * An asset that authored no name is given the one MuJoCo would derive for it
 * FIRST, because MuJoCo derives it from the file and then prefixes the result
 * (`user_mesh.cc:297`, `user_objects.cc:4681`, `:4972`). Rewrite the file first
 * and the derivation runs over an already-prefixed basename, so `p0_base.obj`
 * becomes the mesh `p0_p0_base` while the geom referring to it was prefixed
 * once to `p0_base`, and the compile fails on a reference to nothing. Naming it
 * explicitly means attach prefixes an explicit name exactly once.
 *
 * The spec's own meshdir and texturedir go with them. They resolved the
 * authored paths, the sink has already applied them, and leaving them behind
 * would prepend a directory to a name that is now complete.
 */
void NamespaceAssetsIn(mjSpec& Spec, const TMap<TObjectPtr<const UMjNodeComponent>, mjsElement*>& ElementFor,
	const TArray<FMjAssetRequest>& Requests)
{
	for (const FMjAssetRequest& Request : Requests)
	{
		if (Request.Element == nullptr || Request.VfsName.IsEmpty())
		{
			continue;
		}
		mjsElement* const* const Element = ElementFor.Find(Request.Element);
		if (Element == nullptr)
		{
			continue;
		}
		if (!Request.Name.IsEmpty() && IsUnnamed(*Element))
		{
			// The sink's name for an unnamed element IS the derivation MuJoCo
			// applies, so this pins what would otherwise be derived later.
			const FTCHARToUTF8 Derived(*Request.Name);
			mjs_setName(*Element, Derived.Get());
		}
		if (mjString* const File = FileSlotOf(*Element))
		{
			const FTCHARToUTF8 Name(*Request.VfsName);
			mjs_setString(File, Name.Get());
		}
	}
	if (Spec.compiler.meshdir != nullptr)
	{
		mjs_setString(Spec.compiler.meshdir, "");
	}
	if (Spec.compiler.texturedir != nullptr)
	{
		mjs_setString(Spec.compiler.texturedir, "");
	}
}

/**
 * What a participant's model-wide sections do when it joins a scene.
 *
 * A scene has one `<option>` and one `<size>`, and they are the scene's.
 * MuJoCo's attach conflict resolver decides what happens to a participant's
 * own: the scene's `<compiler conflict>` policy chooses between keeping the
 * scene's value with a warning, merging the two, and refusing the attach.
 *
 * `<size>` is reported as information because the resolver settles it outright
 * and the sizes it settles -- memory, arena, stack -- are capacities rather
 * than physics: a participant that asked for more and got the scene's answer is
 * not a robot behaving differently.
 *
 * `<option>` keeps a warning, as a policy choice rather than a statement about
 * the mechanism. Under the default policy the scene's value wins, and a robot
 * authored against a timestep it no longer gets behaves differently and looks
 * fine, which is exactly the thing worth being told about.
 */
void ReportParticipantGlobals(const FSpecRef& Spec, const FString& Prefix,
	TArray<FMjSpecDiagnostic>& OutWarnings, TArray<FMjSpecDiagnostic>& OutInfos)
{
	UMjNodeComponent* const Root = Spec.GetRoot();
	if (Root == nullptr)
	{
		return;
	}
	for (const FMjOrderedChild& Child : MjOrderedChildrenOf(Spec, *Root))
	{
		ElementType Type{};
		if (Child.Node == nullptr || !gen::ElementTypeOfNode(*Child.Node, Type))
		{
			continue;
		}
		// Every participant HAS these sections; what is worth reporting is a
		// participant that authored one. An unauthored `<option>` asks for no
		// timestep, so there is no value for the scene's policy to resolve away
		// and nothing the author would want to be told.
		if (!MjAuthorsAnything(Spec, *Child.Node))
		{
			continue;
		}
		if (Type == ElementType::Option)
		{
			OutWarnings.Add(DiagnosticFor(Child.Node,
				FString::Printf(TEXT("participant '%s' authors <option>, which does not become the "
									 "scene's: the scene's conflict policy resolves it against the "
									 "scene's own"),
					*Prefix),
				EMjDiagnosticSeverity::Warning));
		}
		else if (Type == ElementType::Size)
		{
			OutInfos.Add(DiagnosticFor(Child.Node,
				FString::Printf(TEXT("participant '%s' authors <size>, which the scene's conflict "
									 "policy resolves against the scene's own"),
					*Prefix),
				EMjDiagnosticSeverity::Info));
		}
	}
}

/** Whatever an error slot holds, never empty. */
FString ErrorTextAt(const char* const Slot)
{
	return Slot != nullptr && Slot[0] != '\0' ? FString(UTF8_TO_TCHAR(Slot)) : FString(TEXT("no reason given"));
}

/** Whatever the spec has to say about its last failure, never null. */
FString SpecErrorText(mjSpec& Spec)
{
	return ErrorTextAt(mjs_getError(&Spec));
}

} // namespace

void MjNamespaceAssets(const FMjBuiltSpec& Built, const TArray<FMjAssetRequest>& Requests)
{
	if (Built.Spec != nullptr)
	{
		NamespaceAssetsIn(*Built.Spec, Built.ElementFor, Requests);
	}
}

// --- FMjCompiledScene ------------------------------------------------------ //

FMjCompiledScene::FMjCompiledScene(FMjCompiledScene&& Other)
	: Model(Other.Model)
	, Scene(MoveTemp(Other.Scene))
	, Participants(MoveTemp(Other.Participants))
	, BoundIds(MoveTemp(Other.BoundIds))
	, Assets(MoveTemp(Other.Assets))
	, Errors(MoveTemp(Other.Errors))
	, Warnings(MoveTemp(Other.Warnings))
	, Infos(MoveTemp(Other.Infos))
{
	Other.Model = nullptr;
}

FMjCompiledScene& FMjCompiledScene::operator=(FMjCompiledScene&& Other)
{
	if (this != &Other)
	{
		Release();
		Model = Other.Model;
		Scene = MoveTemp(Other.Scene);
		Participants = MoveTemp(Other.Participants);
		BoundIds = MoveTemp(Other.BoundIds);
		Assets = MoveTemp(Other.Assets);
		Errors = MoveTemp(Other.Errors);
		Warnings = MoveTemp(Other.Warnings);
		Infos = MoveTemp(Other.Infos);
		Other.Model = nullptr;
	}
	return *this;
}

FMjCompiledScene::~FMjCompiledScene()
{
	Release();
}

void FMjCompiledScene::Release()
{
	// The model first: it was compiled from the specs below it, and the order
	// is the one thing a destructor here has to get right.
	if (Model != nullptr)
	{
		mj_deleteModel(Model);
		Model = nullptr;
	}
	Scene = FMjBuiltSpec();
	Participants.Empty();
	BoundIds.Empty();
}

bool FMjCompiledScene::SaveDebugArtifacts(const FString& Dir, TArray<FMjSpecDiagnostic>& OutDiags) const
{
	// This runs when somebody is already trying to understand a scene that may
	// well be malformed, so every step below reports rather than trusts: the
	// one thing a debug dump must never do is take the editor down with it.
	const auto Fail = [&OutDiags](FString Message) {
		FMjSpecDiagnostic& Diagnostic = OutDiags.AddDefaulted_GetRef();
		Diagnostic.Message = MoveTemp(Message);
		return false;
	};

	if (Scene.Spec == nullptr || Dir.IsEmpty())
	{
		return Fail(TEXT("there is no compiled scene to write"));
	}

	// The assets first, because the XML below is written to point at them.
	const FString AssetDir = FPaths::Combine(Dir, SceneAssetFolder);
	IFileManager::Get().MakeDirectory(*AssetDir, /*Tree=*/true);
	for (const FMjSceneAsset& Asset : Assets)
	{
		if (Asset.Name.IsEmpty() || Asset.Bytes.Num() == 0)
		{
			continue;
		}
		// A mount name keeps whatever subdirectory its reference was authored
		// with, so the sidecar has to carry that directory too or the write
		// lands nowhere.
		const FString Path = FPaths::Combine(AssetDir, Asset.Name);
		IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), /*Tree=*/true);
		if (!FFileHelper::SaveArrayToFile(Asset.Bytes, *Path))
		{
			return Fail(FString::Printf(TEXT("could not write asset '%s'"), *Path));
		}
	}

	// A copy, because what makes the file loadable outside Unreal is a pair of
	// directories pointing at the sidecar, and the compiled scene must keep
	// resolving through the VFS it was built with.
	mjSpec* const Copy = mj_copySpec(Scene.Spec);
	if (Copy == nullptr || Copy->element == nullptr)
	{
		if (Copy != nullptr)
		{
			mj_deleteSpec(Copy);
		}
		return Fail(TEXT("could not copy the scene spec"));
	}
	ON_SCOPE_EXIT
	{
		mj_deleteSpec(Copy);
	};

	const FTCHARToUTF8 Folder(SceneAssetFolder);
	if (Copy->compiler.meshdir == nullptr || Copy->compiler.texturedir == nullptr)
	{
		return Fail(TEXT("the scene spec copy has no asset directories to redirect"));
	}
	mjs_setString(Copy->compiler.meshdir, Folder.Get());
	mjs_setString(Copy->compiler.texturedir, Folder.Get());

	// The copy is compiled before it is serialized. MuJoCo's writer serializes
	// a COMPILED spec, and a copy carries no compile with it; handing it one
	// anyway is an access violation rather than a refusal. The assets go back
	// in for the same reason they went in the first time.
	mjVFS Vfs;
	mj_defaultVFS(&Vfs);
	for (const FMjSceneAsset& Asset : Assets)
	{
		if (!Asset.Name.IsEmpty() && Asset.Bytes.Num() > 0)
		{
			mj_addBufferVFS(&Vfs, TCHAR_TO_UTF8(*Asset.Name), Asset.Bytes.GetData(), Asset.Bytes.Num());
		}
	}
	mjModel* const Recompiled = mj_compile(Copy, &Vfs);
	mj_deleteVFS(&Vfs);
	if (Recompiled == nullptr)
	{
		return Fail(FString::Printf(
			TEXT("the scene spec did not compile for serialization: %s"), *SpecErrorText(*Copy)));
	}
	mj_deleteModel(Recompiled);

	// Sized before the call, then once more against the size it asks for. Two
	// attempts and no loop: a second refusal is a failure to report, not a
	// reason to keep growing a buffer.
	TArray<char> Buffer;
	Buffer.SetNumZeroed(1 << 16);
	char Error[1024] = {0};
	int32 Result = mj_saveXMLString(Copy, Buffer.GetData(), Buffer.Num(), Error, sizeof(Error));
	if (Result > 0)
	{
		// A positive return is the size it wanted rather than a failure.
		Buffer.SetNumZeroed(Result + 1);
		Result = mj_saveXMLString(Copy, Buffer.GetData(), Buffer.Num(), Error, sizeof(Error));
	}
	if (Result != 0)
	{
		return Fail(FString::Printf(TEXT("could not serialize the scene spec: %s"), UTF8_TO_TCHAR(Error)));
	}

	const FString Xml = UTF8_TO_TCHAR(Buffer.GetData());
	const FString XmlPath = FPaths::Combine(Dir, TEXT("scene_compiled.xml"));
	if (!FFileHelper::SaveStringToFile(Xml, *XmlPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
	{
		return Fail(FString::Printf(TEXT("could not write '%s'"), *XmlPath));
	}
	return true;
}

// --- FMjSceneSpecBuilder --------------------------------------------------- //

void FMjSceneSpecBuilder::SetSceneRoot(const FSpecRef& Root)
{
	SceneRoot = Root;
}

void FMjSceneSpecBuilder::AddParticipant(const FMjSceneSpecParticipant& Participant)
{
	Participants.Add(Participant);
}

FMjCompiledScene FMjSceneSpecBuilder::Compile()
{
	FMjCompiledScene Out;

	Out.Scene = BuildSpec(SceneRoot, Out.Errors);
	if (Out.Scene.Spec == nullptr)
	{
		return Out;
	}
	// Which spec is the scene is this builder's answer rather than the walk's,
	// so the name is written here -- but what that name IS is one decision the
	// text writer shares, because the two documents describe one scene.
	if (Out.Scene.Spec->modelname != nullptr)
	{
		const FTCHARToUTF8 Name(*MjSceneModelName(SceneRoot));
		mjs_setString(Out.Scene.Spec->modelname, Name.Get());
	}

	mjsBody* const World = mjs_findBody(Out.Scene.Spec, "world");
	if (World == nullptr)
	{
		Out.Errors.Add(DiagnosticFor(nullptr, TEXT("the scene spec has no world body")));
		return Out;
	}

	// The manager's own assets first, unprefixed. They are collected by the
	// same pass as a participant's because manager-authored content is content:
	// a mesh geom under the manager mounts and binds like any other.
	const auto TakeAssets = [&Out](const FSpecRef& Spec, const FString& Prefix, const FMjBuiltSpec& Built) {
		FMjSceneAssetCollector Collector;
		FMjAssetSink Sink(Collector);
		Sink.VfsPrefix = Prefix;
		Sink.Collect(Spec);
		for (const FMjAssetRequest& Request : Sink.GetRequests())
		{
			if (Request.bMissing)
			{
				Out.Warnings.Add(DiagnosticFor(Request.Element,
					FString::Printf(TEXT("asset '%s' could not be read from '%s'"), *Request.Name,
						*Request.ResolvedPath),
					EMjDiagnosticSeverity::Warning));
			}
		}
		MjNamespaceAssets(Built, Sink.GetRequests());
		Out.Assets.Append(MoveTemp(Collector.Assets));
	};
	TakeAssets(SceneRoot, FString(), Out.Scene);

	for (const FMjSceneSpecParticipant& Participant : Participants)
	{
		FMjBuiltSpec Built = BuildSpec(Participant.Spec, Out.Errors);
		if (Built.Spec == nullptr)
		{
			return Out;
		}

		ReportParticipantGlobals(Participant.Spec, Participant.Prefix, Out.Warnings, Out.Infos);
		TakeAssets(Participant.Spec, Participant.Prefix, Built);

		mjsFrame* const Frame = mjs_addFrame(World, nullptr);
		if (Frame == nullptr)
		{
			Out.Errors.Add(DiagnosticFor(nullptr,
				FString::Printf(TEXT("could not place participant '%s' in the scene"), *Participant.Prefix)));
			return Out;
		}
		Frame->pos[0] = Participant.MjPos.X;
		Frame->pos[1] = Participant.MjPos.Y;
		Frame->pos[2] = Participant.MjPos.Z;
		// MJCF writes a quaternion scalar-first; Unreal stores it scalar-last.
		Frame->quat[0] = Participant.MjQuat.W;
		Frame->quat[1] = Participant.MjQuat.X;
		Frame->quat[2] = Participant.MjQuat.Y;
		Frame->quat[3] = Participant.MjQuat.Z;

		// Where MuJoCo writes the reason an attach failed: a fixed buffer on the
		// target's model, taken here while the spec it hangs off is still whole.
		// A failed attach leaves that spec unusable -- the child's subtree is
		// spliced in and its lists half-merged -- so asking the wreck for its
		// error afterwards puts a dereference of it between the failure and the
		// diagnostic, and the diagnostic is the only thing the caller is going
		// to get. The buffer outlives the damage; the route to it does not.
		const char* const ErrorSlot = mjs_getError(Out.Scene.Spec);

		// MuJoCo reads the conflict policy from the TARGET at each attach
		// (`user_api.cc:476`), so setting it here answers the question for this
		// participant alone. Restored below whether the attach succeeded or not,
		// because the scene's own policy governs every participant that did not
		// ask for something else.
		const mjtConflict ScenePolicy = static_cast<mjtConflict>(Out.Scene.Spec->compiler.conflict);
		if (Participant.Conflict.IsSet())
		{
			Out.Scene.Spec->compiler.conflict = static_cast<mjtConflict>(Participant.Conflict.GetValue());
		}

		// The spec's own element, not its world body: the world body would come
		// across as a body of its own and put an extra link in every chain.
		const FTCHARToUTF8 Prefix(*Participant.Prefix);
		const bool bAttached = Frame->element != nullptr && Built.Spec->element != nullptr
							&& mjs_attach(Frame->element, Built.Spec->element, Prefix.Get(), "") != nullptr;

		Out.Scene.Spec->compiler.conflict = ScenePolicy;
		if (!bAttached)
		{
			// Abandoned, never continued and never retried: a scene with a
			// half-attached participant in it has no compile left in it, and
			// the caller gets the reason instead of a model.
			Out.Errors.Add(DiagnosticFor(nullptr,
				FString::Printf(TEXT("could not attach participant '%s': %s"), *Participant.Prefix,
					*ErrorTextAt(ErrorSlot))));
			return Out;
		}

		// Held for the compiled scene's lifetime. Attach moves the elements
		// rather than copying them, so the participant's recorded handles are
		// the composed spec's elements and keeping the spec is what makes the
		// ownership say so.
		Out.Participants.Add(MoveTemp(Built));
	}

	mjVFS Vfs;
	mj_defaultVFS(&Vfs);
	for (const FMjSceneAsset& Asset : Out.Assets)
	{
		if (!Asset.Name.IsEmpty() && Asset.Bytes.Num() > 0)
		{
			mj_addBufferVFS(&Vfs, TCHAR_TO_UTF8(*Asset.Name), Asset.Bytes.GetData(), Asset.Bytes.Num());
		}
	}
	Out.Model = mj_compile(Out.Scene.Spec, &Vfs);
	mj_deleteVFS(&Vfs);

	if (Out.Model == nullptr)
	{
		Out.Errors.Add(DiagnosticFor(nullptr,
			FString::Printf(TEXT("the scene did not compile: %s"), *SpecErrorText(*Out.Scene.Spec))));
		return Out;
	}

	// Identity comes from the handles the walk recorded, which the compile has
	// just turned into ids. Nothing is looked up by name, so nothing depends on
	// a component having one.
	const auto Bind = [&Out](const FMjBuiltSpec& Built) {
		for (const TPair<TObjectPtr<const UMjNodeComponent>, mjsElement*>& Entry : Built.ElementFor)
		{
			if (Entry.Value == nullptr)
			{
				continue;
			}
			const int32 Id = mjs_getId(Entry.Value);
			if (Id >= 0)
			{
				FMjBoundElement Bound;
				Bound.ObjType = Entry.Value->elemtype;
				Bound.Id = Id;
				Out.BoundIds.Add(Entry.Key, Bound);
			}
		}
	};
	Bind(Out.Scene);
	for (const FMjBuiltSpec& Built : Out.Participants)
	{
		Bind(Built);
	}

	return Out;
}

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
