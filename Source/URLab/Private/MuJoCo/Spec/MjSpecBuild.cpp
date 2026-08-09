// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjSpecBuild.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Gen/MjSpecWrite.gen.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MjSpecBuildContext.h"
#include "MjSpecWriteHooks.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

namespace urlab::spec
{
namespace
{

namespace sw = ps::ue::specwrite;

using ps::mjcf::ElementType;

/** The model name a spec gets when its root authors none. */
const TCHAR* const UnauthoredModelName = TEXT("urlab");

/**
 * The order the sections are written in.
 *
 * Not document order, and it cannot be: a `<geom>` naming a material has to
 * find one on the spec, an element resolving a default class has to find the
 * class, and MuJoCo resolves both by name at compile rather than at write. So
 * the definitions go down first and the things referring to them after, which
 * is the same order MJCF itself is conventionally written in.
 */
int32 PhaseOf(ElementType Type)
{
	switch (Type)
	{
	case ElementType::Default: return 0;
	case ElementType::Compiler:
	case ElementType::Option:
	case ElementType::Size:
	case ElementType::Statistic:
	case ElementType::Visual: return 1;
	case ElementType::Asset: return 2;
	case ElementType::Body: return 3;
	case ElementType::Contact: return 4;
	case ElementType::Deformable: return 5;
	case ElementType::Equality: return 6;
	case ElementType::Tendon: return 7;
	case ElementType::Actuator: return 8;
	case ElementType::Sensor: return 9;
	case ElementType::Custom: return 10;
	case ElementType::Keyframe: return 11;
	case ElementType::Extension: return 12;
	default: return 13;
	}
}

/** A node's authored default class, when its schema gives it one. */
TOptional<FString> DclassOf(const UMjNodeComponent& Node)
{
	TOptional<FString> Out;
	gen::DispatchByType(Node, [&Out](const auto& Element)
	{
		if constexpr (requires { Element.Dclass; })
		{
			Out = Element.Dclass;
		}
		(void)Element;
	});
	return Out;
}

/** The `model` attribute of `<mujoco>`, when the root authored one. */
TOptional<FString> ModelNameOf(const UMjNodeComponent& Root)
{
	TOptional<FString> Out;
	gen::DispatchByType(Root, [&Out](const auto& Element)
	{
		if constexpr (requires { Element.Model; })
		{
			Out = Element.Model;
		}
		(void)Element;
	});
	return Out;
}

/** The class a node imposes on its subtree, when its schema gives it one. */
TOptional<FString> ChildclassOf(const UMjNodeComponent& Node)
{
	TOptional<FString> Out;
	gen::DispatchByType(Node, [&Out](const auto& Element)
	{
		if constexpr (requires { Element.Childclass; })
		{
			Out = Element.Childclass;
		}
		(void)Element;
	});
	return Out;
}

class FBuilder
{
public:
	FBuilder(const FSpecRef& Root, TArray<FMjSpecDiagnostic>& OutDiags)
	{
		Ctx.Source = &Root;
		Ctx.Diagnostics = &OutDiags;
		Ctx.Failed = &bFailed;
		Ctx.Aborted = &bAborted;
		Ctx.Models = &Models;
	}

	FMjBuiltSpec Build();

private:
	void WalkChildren(UMjNodeComponent& Parent, const FMjSpecWriteContext& Inherited);
	void WalkNode(UMjNodeComponent& Node, const FMjSpecWriteContext& Inherited);
	bool ResolveClass(UMjNodeComponent& Node, FMjSpecWriteContext& Local);
	void Identify(UMjNodeComponent& Node, FMjSpecWriteContext& Local);
	bool RunHooks(UMjNodeComponent& Node, FMjSpecWriteContext& Local, bool bCreating);

	FMjSpecWriteContext Ctx;
	FMjBuiltSpec Result;
	FMjModelAssets Models;
	bool bFailed = false;
	bool bAborted = false;
};

bool FBuilder::ResolveClass(UMjNodeComponent& Node, FMjSpecWriteContext& Local)
{
	// The authored class wins, then whatever the nearest enclosing body or frame
	// imposed, then what was inherited. Same resolution MuJoCo's reader does, so
	// a nested class chain lands on mj_compile exactly as it would from a file.
	const TOptional<FString> Authored = DclassOf(Node);
	if (Authored.IsSet() && !Authored.GetValue().IsEmpty())
	{
		Local.ClassName = Authored.GetValue();
	}
	const FTCHARToUTF8 Name(*Local.ClassName);
	Local.Class = mjs_findDefault(Local.Spec, Name.Get());
	if (Local.Class == nullptr)
	{
		return Ctx.Error(Node, FString::Printf(
			TEXT("no default class named '%s'"), *Local.ClassName));
	}
	return true;
}

void FBuilder::Identify(UMjNodeComponent& Node, FMjSpecWriteContext& Local)
{
	ElementType Type;
	if (Local.Element == nullptr || !gen::ElementTypeOfNode(Node, Type))
	{
		return;
	}

	if (!sw::CreationTakesName(Type) && Node.MjName.IsSet())
	{
		const FTCHARToUTF8 Name(*Node.MjName.GetValue());
		mjs_setName(Local.Element, Name.Get());
	}

	// Every element but one resolves THROUGH a class and carries the name of the
	// one it resolved through. A <default> is not one of them: it IS a class, its
	// handle is the class object itself, and that object has no class-name field
	// to write -- mjs_setDefault would put a name over whatever member happens to
	// sit at that offset. Which class it nests in was the argument it was created
	// with, and nothing here can add to that.
	if (Local.Class != nullptr && Type != ElementType::Default)
	{
		mjs_setDefault(Local.Element, Local.Class);
	}

	// Provenance, in the shape MuJoCo's own reader stamps: the compiler appends
	// it verbatim to any error about this element, so it is the difference
	// between "bad geom" and a line the user can open.
	if (Local.Struct != nullptr && (!Node.SourceFile.IsEmpty() || Node.SourceLine > 0))
	{
		const FString Info = FString::Printf(
			TEXT("%s:%d"), *Node.SourceFile, Node.SourceLine);
		const FTCHARToUTF8 Utf8Info(*Info);
		sw::SetInfo(Type, Local.Struct, Utf8Info.Get());
	}

	Result.ElementFor.Add(&Node, Local.Element);
}

bool FBuilder::RunHooks(UMjNodeComponent& Node, FMjSpecWriteContext& Local, bool bCreating)
{
	ElementType Type;
	if (!gen::ElementTypeOfNode(Node, Type))
	{
		return true;
	}
	const sw::FMjHookList Hooks = sw::HooksFor(Type);
	bool bOk = true;
	for (int32 Index = 0; Index < Hooks.Num; ++Index)
	{
		const FMjSpecWriteHookRow* const Row = FindSpecWriteHook(Hooks.Names[Index]);
		if (Row == nullptr)
		{
			bOk = Ctx.Error(Node, FString::Printf(
				TEXT("no hook registered under '%s'"), Hooks.Names[Index]));
			continue;
		}
		const FMjSpecWriteHook Hook = bCreating ? Row->Create : Row->Apply;
		if (Hook == nullptr)
		{
			continue;
		}
		if (!Hook(Local, Node, Local.Element))
		{
			bOk = false;
		}
	}
	return bOk;
}

void FBuilder::WalkNode(UMjNodeComponent& Node, const FMjSpecWriteContext& Inherited)
{
	ElementType Type;
	if (!gen::ElementTypeOfNode(Node, Type))
	{
		Ctx.Error(Node, TEXT("this component is not a spec element"));
		return;
	}

	FMjSpecWriteContext Local = Inherited;
	Local.Struct = nullptr;
	Local.Element = nullptr;
	Local.bChildrenConsumed = false;

	const sw::ECreate Category = sw::CreateOf(Type);
	if (Category != sw::ECreate::Section && !ResolveClass(Node, Local))
	{
		return;  // the subtree is skipped: it would resolve against nothing
	}

	switch (Category)
	{
	case sw::ECreate::BodyScoped:
	case sw::ECreate::SpecScoped:
	{
		// A <default> nested in a <default> is a class of its own, not one of
		// its parent's templates: it is the one creating element that stays
		// creating inside a class, and it chains to the enclosing class below.
		// Everything else here IS a template for its own type.
		if (Local.Partial != nullptr && Type != ElementType::Default)
		{
			Local.Struct = sw::DefaultMember(Type, Local.Partial);
			if (Local.Struct == nullptr)
			{
				Ctx.Error(Node, TEXT("this element cannot be a default class partial"));
				return;
			}
			break;
		}
		// A <default> is the one element whose creation consumes its own name
		// and whose "default" argument is the class it nests inside rather than
		// the class it resolves through.
		const bool bClass = Type == ElementType::Default;

		// The top-level `<default>` names no class because it IS the spec's
		// root class, which every spec already has. MuJoCo's own reader
		// configures "main" from it rather than adding a second one, and
		// mjs_addDefault refuses a class name the spec already holds.
		if (bClass && Local.Partial == nullptr &&
			!(Node.MjName.IsSet() && !Node.MjName.GetValue().IsEmpty()))
		{
			mjsDefault* const Root =
				Local.Spec != nullptr ? mjs_getSpecDefault(Local.Spec) : nullptr;
			if (Root == nullptr)
			{
				Ctx.Error(Node, TEXT("this spec has no root default class"));
				return;
			}
			Local.Element = Root->element;
			Local.Struct = Root;
			break;
		}

		const FTCHARToUTF8 ClassName(bClass && Node.MjName.IsSet()
			? *Node.MjName.GetValue() : *Local.ClassName);
		const mjsDefault* const Parent =
			bClass && Local.Partial != nullptr ? Local.Partial : Local.Class;
		// What an element is created on is the walk's answer, not the schema's,
		// so it is the walk that has to say when it has no answer. MuJoCo's
		// mjs_add* take the owner as a bare pointer and read through it.
		if (Local.Spec == nullptr ||
			(Category == sw::ECreate::BodyScoped && Local.Body == nullptr))
		{
			Ctx.Error(Node, TEXT("this element has nothing to be created on"));
			return;
		}
		const sw::FMjCreated Made = sw::Create(Type, Local.Spec, Local.Body,
			Local.Frame, Parent, ClassName.Get());
		if (Made.Element == nullptr)
		{
			Ctx.Error(Node, TEXT("the spec refused to create this element"));
			return;
		}
		Local.Element = Made.Element;
		Local.Struct = Made.Struct;
		// A body or frame becomes the context its own subtree is created in.
		if (Made.Body != nullptr) { Local.Body = Made.Body; }
		if (Made.Frame != nullptr) { Local.Frame = Made.Frame; }
		if (Made.Default != nullptr) { Local.Struct = Made.Default; }
		break;
	}

	case sw::ECreate::Hook:
		// A hook element is created by its hook, if it is created at all: which
		// shorthand an actuator is decides how it is made, and a macro element
		// or a fold has no element of its own.
		if (!RunHooks(Node, Local, /*bCreating=*/true))
		{
			return;
		}
		break;

	case sw::ECreate::ParentEmbedded:
	case sw::ECreate::SpecEmbedded:
		Local.Struct = sw::EmbeddedTarget(Type, Local.Spec, Local.Parent);
		if (Local.Struct == nullptr)
		{
			Ctx.Error(Node, TEXT("this element has no struct to write onto"));
			return;
		}
		break;

	case sw::ECreate::Section:
		break;
	}

	Identify(Node, Local);

	if (Category == sw::ECreate::ParentEmbedded || Category == sw::ECreate::SpecEmbedded)
	{
		sw::ApplyEmbedded(Node, Local.Spec, Local.Parent);
	}
	else if (Local.Struct != nullptr)
	{
		sw::ApplyFields(Node, Local.Struct);
	}

	if (!RunHooks(Node, Local, /*bCreating=*/false))
	{
		return;  // the element is malformed; its children would be worse
	}

	if (Local.bChildrenConsumed)
	{
		return;
	}

	// The subtree is created inside whatever this node established.
	FMjSpecWriteContext Child = Local;
	Child.Parent = Local.Struct;
	Child.ParentNode = &Node;
	// A class is a class only for its immediate children: a nested <default>
	// replaces it, and nothing else under it opens one.
	Child.Partial = Type == ElementType::Default
		? static_cast<mjsDefault*>(Local.Struct) : nullptr;
	const TOptional<FString> Childclass = ChildclassOf(Node);
	if (Childclass.IsSet() && !Childclass.GetValue().IsEmpty())
	{
		Child.ClassName = Childclass.GetValue();
	}
	WalkChildren(Node, Child);
}

void FBuilder::WalkChildren(UMjNodeComponent& Parent, const FMjSpecWriteContext& Inherited)
{
	TArray<FMjOrderedChild> Children = MjOrderedChildrenOf(*Ctx.Source, Parent);

	// Inside a class, the nested classes go last.
	//
	// mjs_addDefault copies the enclosing class's templates into the new one as
	// it creates it, so whatever the enclosing class has not been given yet is
	// not inherited at all. Schema order puts `<default>` first among a class's
	// children, which would open every nested class before the templates it is
	// supposed to inherit had been written.
	ElementType ParentType{};
	if (gen::ElementTypeOfNode(Parent, ParentType) && ParentType == ElementType::Default)
	{
		Children.StableSort([](const FMjOrderedChild& A, const FMjOrderedChild& B)
		{
			ElementType Left{};
			ElementType Right{};
			const bool bLeft = gen::ElementTypeOfNode(*A.Node, Left)
				&& Left == ElementType::Default;
			const bool bRight = gen::ElementTypeOfNode(*B.Node, Right)
				&& Right == ElementType::Default;
			return bLeft < bRight;
		});
	}

	for (const FMjOrderedChild& Child : Children)
	{
		// An abort is not a skipped element: the spec it would be written onto
		// is already unusable, so there is nothing left for the rest of the
		// document to be written into.
		if (bAborted)
		{
			return;
		}
		if (Child.Node != nullptr)
		{
			WalkNode(*Child.Node, Inherited);
		}
	}
}

FMjBuiltSpec FBuilder::Build()
{
	UMjNodeComponent* const Root = Ctx.Source->GetRoot();
	const auto Fail = [this](const TCHAR* Message)
	{
		bFailed = true;
		FMjSpecDiagnostic& Diagnostic = Ctx.Diagnostics->AddDefaulted_GetRef();
		Diagnostic.Message = Message;
	};

	if (Root == nullptr)
	{
		Fail(TEXT("the spec has no root component"));
		return FMjBuiltSpec();
	}

	Ctx.Spec = mj_makeSpec();
	if (Ctx.Spec == nullptr)
	{
		Fail(TEXT("could not allocate an mjSpec"));
		return FMjBuiltSpec();
	}
	Result.Spec = Ctx.Spec;
	Ctx.Body = mjs_findBody(Ctx.Spec, "world");
	if (Ctx.Body == nullptr)
	{
		Fail(TEXT("a fresh mjSpec has no world body"));
		mj_deleteSpec(Result.Spec);
		Result.Spec = nullptr;
		return FMjBuiltSpec();
	}

	// The model name is the spec's rather than any element's, and it is the
	// root's `model` FIELD rather than the identity attribute every other
	// element carries its name in. Reading the wrong one is silent: mj_makeSpec
	// names a fresh spec "MuJoCo Model", so the name reaches the compiled name
	// table either way and only its content says which was read.
	//
	// Written whether or not the root authored one, because the name buffer is
	// part of the model a comparison sees and leaving it to mj_makeSpec makes
	// the same document compile differently depending on which path built it.
	// The stand-in is a constant rather than anything derived from a file or an
	// asset, so the artefact stays reproducible, and it is deliberately not
	// MuJoCo's own default, so that reading a compiled model still says whether
	// a name was authored.
	const TOptional<FString> ModelName = ModelNameOf(*Root);
	const FString Named = ModelName.IsSet() && !ModelName.GetValue().IsEmpty()
		? ModelName.GetValue() : FString(UnauthoredModelName);
	const FTCHARToUTF8 Name(*Named);
	mjs_setString(Ctx.Spec->modelname, Name.Get());

	// Sections in write order, siblings within a section in authored order.
	TArray<FMjOrderedChild> Sections = MjOrderedChildrenOf(*Ctx.Source, *Root);
	Sections.StableSort([](const FMjOrderedChild& A, const FMjOrderedChild& B)
	{
		ElementType Left{};
		ElementType Right{};
		gen::ElementTypeOfNode(*A.Node, Left);
		gen::ElementTypeOfNode(*B.Node, Right);
		return PhaseOf(Left) < PhaseOf(Right);
	});

	FMjSpecWriteContext Top = Ctx;
	Top.ParentNode = Root;
	for (const FMjOrderedChild& Section : Sections)
	{
		if (bAborted)
		{
			break;
		}
		if (Section.Node == nullptr)
		{
			continue;
		}
		ElementType Type{};
		if (gen::ElementTypeOfNode(*Section.Node, Type) && Type == ElementType::Body)
		{
			// The root's body IS the world body, which the spec already has, so
			// its children are written onto that rather than into a new one.
			FMjSpecWriteContext World = Top;
			World.ParentNode = Section.Node;
			Result.ElementFor.Add(Section.Node, Ctx.Body->element);
			WalkChildren(*Section.Node, World);
			continue;
		}
		WalkNode(*Section.Node, Top);
	}

	if (bFailed)
	{
		// A spec that failed halfway is worse than none: it compiles into a
		// model missing exactly the elements nobody was told about.
		mj_deleteSpec(Result.Spec);
		Result.Spec = nullptr;
		Result.ElementFor.Empty();
	}
	return MoveTemp(Result);
}

}  // namespace

bool FMjSpecWriteContext::Error(const UMjNodeComponent& Node, const FString& Message)
{
	if (Failed != nullptr)
	{
		*Failed = true;
	}
	if (Diagnostics != nullptr)
	{
		FMjSpecDiagnostic& Diagnostic = Diagnostics->AddDefaulted_GetRef();
		Diagnostic.Message = Message;
		Diagnostic.File = Node.SourceFile;
		Diagnostic.Line = Node.SourceLine;
	}
	return false;
}

bool FMjSpecWriteContext::Abort(const UMjNodeComponent& Node, const FString& Message)
{
	if (Aborted != nullptr)
	{
		*Aborted = true;
	}
	return Error(Node, Message);
}

bool FMjSpecWriteContext::Warn(const UMjNodeComponent& Node, const FString& Message)
{
	if (Diagnostics != nullptr)
	{
		FMjSpecDiagnostic& Diagnostic = Diagnostics->AddDefaulted_GetRef();
		Diagnostic.Message = Message;
		Diagnostic.File = Node.SourceFile;
		Diagnostic.Line = Node.SourceLine;
	}
	return true;
}

FMjModelAssets::~FMjModelAssets()
{
	for (mjSpec* const Child : Owned)
	{
		mj_deleteSpec(Child);
	}
}

void FMjModelAssets::Add(const FString& Name, mjSpec* Child)
{
	if (Child == nullptr)
	{
		return;
	}
	Owned.Add(Child);
	if (!ByName.Contains(Name))
	{
		ByName.Add(Name, Child);
	}
}

mjSpec* FMjModelAssets::Find(const FString& Name) const
{
	mjSpec* const* const Found = ByName.Find(Name);
	return Found != nullptr ? *Found : nullptr;
}

FMjBuiltSpec::FMjBuiltSpec(FMjBuiltSpec&& Other)
	: Spec(Other.Spec)
	, ElementFor(MoveTemp(Other.ElementFor))
{
	Other.Spec = nullptr;
}

FMjBuiltSpec& FMjBuiltSpec::operator=(FMjBuiltSpec&& Other)
{
	if (this != &Other)
	{
		if (Spec != nullptr)
		{
			mj_deleteSpec(Spec);
		}
		Spec = Other.Spec;
		ElementFor = MoveTemp(Other.ElementFor);
		Other.Spec = nullptr;
	}
	return *this;
}

FMjBuiltSpec::~FMjBuiltSpec()
{
	if (Spec != nullptr)
	{
		mj_deleteSpec(Spec);
		Spec = nullptr;
	}
}

FMjBuiltSpec BuildSpec(const FSpecRef& Root, TArray<FMjSpecDiagnostic>& OutDiags)
{
	if (!Root.IsValid())
	{
		FMjSpecDiagnostic& Diagnostic = OutDiags.AddDefaulted_GetRef();
		Diagnostic.Message = TEXT("the spec handle names no component tree");
		return FMjBuiltSpec();
	}
	return FBuilder(Root, OutDiags).Build();
}

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
