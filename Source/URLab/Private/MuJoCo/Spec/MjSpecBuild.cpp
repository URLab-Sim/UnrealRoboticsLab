// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjSpecBuild.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Gen/MjEnums.gen.h"
#include "MuJoCo/Gen/MjKeywords.gen.h"
#include "MuJoCo/Gen/MjSpecWrite.gen.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MjReservedNames.h"
#include "MjSpecBuildContext.h"
#include "MjSpecNodes.h"
#include "MjSpecWriteHooks.h"
#include "MuJoCo/Spec/MjScalePolicy.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

namespace urlab::spec
{
namespace
{

namespace sw = ps::ue::specwrite;

using ps::mjcf::ElementType;

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

/** How many `size` values a node authored on its own storage, if any. */
TOptional<int32> AuthoredSizeNumOf(const UMjNodeComponent& Node)
{
	TOptional<int32> Out;
	gen::DispatchByType(Node, [&Out](const auto& Element)
	{
		// A `size` the schema gives a length to. An element whose `size` is a
		// fixed arity has nothing to count and nothing to truncate.
		if constexpr (requires { Element.Size.IsSet(); Element.Size.GetValue().Num(); })
		{
			if (Element.Size.IsSet())
			{
				Out = Element.Size.GetValue().Num();
			}
		}
		(void)Element;
	});
	return Out;
}

/**
 * How a diagnostic says which element it is about.
 *
 * Both names, because they answer different questions: the component name is
 * what the user selects in the editor, and the MJCF name is what the compiled
 * model, the debug artefact and a bridge client all know the element by. An
 * element the document left unnamed is carrying its transient reserved name
 * here, which is the name that leaves the engine, so quoting it is right.
 */
FString IdentityOf(const UMjNodeComponent& Node)
{
	const FString Component = Node.GetName();
	const FString MjName = Node.MjName.Get(FString());
	if (MjName.IsEmpty() || MjName == Component)
	{
		return Component;
	}
	return FString::Printf(TEXT("%s '%s'"), *Component, *MjName);
}

/** The element a diagnostic is about, and what it was being written under. */
FString DiagnosticSubject(const UMjNodeComponent& Node, const UMjNodeComponent* Walked)
{
	if (Walked == nullptr || Walked == &Node)
	{
		return IdentityOf(Node);
	}
	return FString::Printf(TEXT("%s under %s"), *IdentityOf(Node), *IdentityOf(*Walked));
}

/** An MJCF keyword as text. The tables are ASCII by construction. */
FString KeywordOf(EMjGeomType Type)
{
	FString Out;
	for (const char Character : ps::ue::ToMjcf(Type))
	{
		Out.AppendChar(static_cast<TCHAR>(Character));
	}
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
	void ReportOverLongSizes();

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

void FBuilder::ReportOverLongSizes()
{
	// One pass over what the walk built, rather than a check inside each
	// generated Apply. The gizmo, the array widget and a hand-edited MJCF file
	// all arrive at this same build, so the question is asked once where they
	// meet; scattered through generated code it would be a rule nobody reviews.
	//
	// REPORTING ONLY, and the values are left exactly as authored. MuJoCo reads
	// `mjGEOMINFO[type]` values out of `size` and keeps the rest: `checksize`
	// bounds its loop by the arity and never looks further (user_objects.cc:163),
	// and `mjCModel::CopyObjects` copies all three slots into `geom_size` and
	// `site_size` whatever the type reads (user_model.cc:3055, :3101). Dropping
	// them would compile a different model from the one stock MuJoCo compiles
	// out of the same document, which is a worse fault than the silence: this
	// project's correctness rests on those two models being the same one.
	//
	// So what is wrong is not the values, it is that nobody is told. A sphere
	// authored with three radii is a sphere of the first and the other two decide
	// nothing, and until now no compile said so.
	//
	// The count is the element's own authored one and the type is read off the
	// built element, because creation copies the resolved class's template in:
	// that is the type the compile will read even where the element authored none
	// of it, and it means a shape whose tail came from its class, or a site
	// sitting on MuJoCo's own 0.005 default, is not accused of authoring it.
	//
	// Reported to two audiences on purpose. The build's own diagnostic array is
	// read when a build FAILS, and this does not fail one; a user who authored a
	// size that decides nothing has to be told where they are looking, which is
	// the editor's message log.
	TArray<FMjSizeViolation> Violations;
	for (const TPair<TObjectPtr<const UMjNodeComponent>, mjsElement*>& Entry : Result.ElementFor)
	{
		const UMjNodeComponent* const Node = Entry.Key.Get();
		if (Node == nullptr || Entry.Value == nullptr)
		{
			continue;
		}
		const TOptional<int32> Authored = AuthoredSizeNumOf(*Node);
		if (!Authored.IsSet() || Authored.GetValue() <= 0)
		{
			continue;
		}

		mjtGeom Shape = mjGEOM_SPHERE;
		if (const mjsGeom* const Geom = mjs_asGeom(Entry.Value))
		{
			Shape = Geom->type;
		}
		else if (const mjsSite* const Site = mjs_asSite(Entry.Value))
		{
			Shape = Site->type;
		}
		else
		{
			continue;  // a `size` that is not a shape's: a composite, an hfield
		}

		const EMjGeomType Type = static_cast<EMjGeomType>(Shape);
		const int32 Allowed = MjSizeArityFor(Type);
		if (Authored.GetValue() <= Allowed)
		{
			continue;
		}
		const FString Message = FString::Printf(
			TEXT("authors %d size values where a %s reads %d; the rest are carried into the compiled model, "
				 "as MuJoCo carries them, and decide nothing"),
			Authored.GetValue(), *KeywordOf(Type), Allowed);
		Ctx.Warn(*Node, Message);

		FMjSizeViolation& Violation = Violations.AddDefaulted_GetRef();
		Violation.Name = Node->MjName.Get(Node->GetName());
		Violation.File = Node->SourceFile;
		Violation.Line = Node->SourceLine;
		Violation.Authored = Authored.GetValue();
		Violation.Allowed = Allowed;
		Violation.Message = FString::Printf(TEXT("%s: %s"), *IdentityOf(*Node), *Message);
	}

	MjReportSizeArity(Violations);
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
	Local.Node = &Node;
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
	// Written only when the root authored one. MuJoCo's own reader writes it
	// under exactly that condition (`xml_native_reader.cc:211`), so a document
	// with no `model` attribute keeps the name mj_makeSpec left on the fresh
	// spec and compiles to the same name buffer whichever path built it.
	const TOptional<FString> ModelName = ModelNameOf(*Root);
	if (ModelName.IsSet() && !ModelName.GetValue().IsEmpty())
	{
		const FTCHARToUTF8 Name(*ModelName.GetValue());
		mjs_setString(Ctx.Spec->modelname, Name.Get());
	}

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
	Top.Node = Root;
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
			World.Node = Section.Node;
			World.ParentNode = Section.Node;
			Result.ElementFor.Add(Section.Node, Ctx.Body->element);
			WalkChildren(*Section.Node, World);
			continue;
		}
		WalkNode(*Section.Node, Top);
	}

	// The whole document is written, so every element carries the type its class
	// resolved to and nothing more will be added: the last thing before the spec
	// is handed to a compile.
	ReportOverLongSizes();

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

/**
 * Nothing under `Root` still carries a name the reservation minted.
 *
 * The reservation is transient by contract and the tree is the user's document,
 * so a name that survived is authored identity nobody asked for. It would not
 * fail anything here: it fails later, in a saved package and in the next diff
 * of their MJCF, which is why it is checked at the one point the contract says
 * it must already be true.
 */
void CheckReservationLifted(const FSpecRef& Root)
{
#if DO_CHECK
	const FMjSpecNodes Tree = MjSpecNodesOf(Root);
	for (const UMjNodeComponent* const Node : Tree.Nodes)
	{
		if (Node == nullptr || !Node->MjName.IsSet())
		{
			continue;
		}
		checkf(!Node->MjName.GetValue().StartsWith(MjReservedNamePrefix),
			TEXT("the reserved name '%s' survived the spec build; the reservation scope is meant to take "
				 "every one of them off again before the tree is handed back"),
			*Node->MjName.GetValue());
	}
#else
	(void)Root;
#endif
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
		Diagnostic.Message = FString::Printf(
			TEXT("%s: %s"), *DiagnosticSubject(Node, this->Node), *Message);
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
		Diagnostic.Message = FString::Printf(
			TEXT("%s: %s"), *DiagnosticSubject(Node, this->Node), *Message);
		Diagnostic.File = Node.SourceFile;
		Diagnostic.Line = Node.SourceLine;
		// The build carried on and produced a spec. A caller asking "did this
		// work" by the array's length would be told no, over a remark.
		Diagnostic.Severity = EMjDiagnosticSeverity::Warning;
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
	FMjBuiltSpec Built;
	{
		// Held across the whole walk: the elements are named from their
		// components, and a macro's subtree is serialized back out to MJCF from
		// them as well.
		const FMjReservedNames Reserved(Root);
		Built = FBuilder(Root, OutDiags).Build();
	}
	CheckReservationLifted(Root);
	return Built;
}

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
