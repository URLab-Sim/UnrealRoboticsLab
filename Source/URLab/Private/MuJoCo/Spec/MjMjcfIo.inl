// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The reader and writer bodies, shared by both profile instantiation units.
//
// Included exactly twice, once per translation unit, with URLAB_MJ_IO_PROFILE
// and URLAB_MJ_IO_FACTORY naming the profile and its construction seam. The
// bodies are identical because the profile seam is what absorbs the difference:
// that identity is the point of the seam, so it is stated as one file rather
// than copied.

#if !defined(URLAB_MJ_IO_PROFILE) || !defined(URLAB_MJ_IO_FACTORY) || !defined(URLAB_MJ_IO_PARSE) || \
	!defined(URLAB_MJ_IO_WRITE) || !defined(URLAB_MJ_IO_WRITE_ELEMENT)
#error "MjMjcfIo.inl needs URLAB_MJ_IO_{PROFILE,FACTORY,PARSE,WRITE,WRITE_ELEMENT}"
#endif

namespace urlab::spec::io
{

FMjSpecParseResult URLAB_MJ_IO_PARSE(const FString& Xml, const FString& Filename,
	const FMjDocParseOptions& Options)
{
	ps::mjcf::io::ParseOptions PsOptions;
	PsOptions.allow_external_includes = Options.bAllowExternalIncludes;

	auto Result = ps::mjcf::io::ParseMjcfStringT<URLAB_MJ_IO_PROFILE, URLAB_MJ_IO_FACTORY>(
		gen::FMjStrPolicy::ToUtf8(gen::FMjStrPolicy::View(Xml)),
		gen::FMjStrPolicy::ToUtf8(gen::FMjStrPolicy::View(Filename)), PsOptions);

	FMjSpecParseResult Out;
	Out.Root = Result.model;
	CollectDiagnostics(Result.errors, Out.Errors);
	CollectDiagnostics(Result.warnings, Out.Warnings);
	if (Out.Errors.Num() > 0)
	{
		Out.Root = nullptr;
	}
	return Out;
}

FString URLAB_MJ_IO_WRITE(const UMjNodeComponent& Root, TArray<FMjSpecDiagnostic>* OutErrors)
{
	using Doc = pssdk::DocOf<URLAB_MJ_IO_PROFILE>;
	const Doc* Model = Cast<Doc>(&Root);
	if (Model == nullptr)
	{
		if (OutErrors != nullptr)
		{
			FMjSpecDiagnostic Diagnostic;
			Diagnostic.Message = TEXT("spec root is not a <mujoco> element");
			OutErrors->Add(MoveTemp(Diagnostic));
		}
		return FString();
	}

	std::vector<ps::Diagnostic> Errors;
	const std::string Text =
		ps::mjcf::io::WriteMjcfT<URLAB_MJ_IO_PROFILE>(*Model, /*names=*/nullptr, &Errors);
	if (OutErrors != nullptr)
	{
		CollectDiagnostics(Errors, *OutErrors);
	}
	return gen::FMjStrPolicy::FromUtf8(Text);
}

FString URLAB_MJ_IO_WRITE_ELEMENT(const UMjNodeComponent& Node, TArray<FMjSpecDiagnostic>* OutErrors)
{
	std::vector<ps::Diagnostic> Errors;
	std::string Text;

	// The tag is the element's own, never the parent's slot. A caller reaching
	// here has no parent to derive a contextual alias from, and letting it pass
	// one would be an invitation to pass the wrong one.
	const bool bDispatched = gen::DispatchByType(Node, [&](const auto& Element)
	{
		using E = std::decay_t<decltype(Element)>;
		const psm::ElementType Type = pssdk::ElementTypeOf<URLAB_MJ_IO_PROFILE, E>;
		ps::mjcf::io::WriteElement<URLAB_MJ_IO_PROFILE>(Element, ps::mjcf::xmlbind::Bind(Type).tag,
			/*depth=*/0, Text, /*names=*/nullptr, &Errors);
	});

	if (!bDispatched)
	{
		if (OutErrors != nullptr)
		{
			FMjSpecDiagnostic Diagnostic;
			Diagnostic.Message = TEXT("this component is not a spec element");
			OutErrors->Add(MoveTemp(Diagnostic));
		}
		return FString();
	}

	// Same contract as the whole-document write: a partial document is worse
	// than none, because it parses.
	if (!Errors.empty())
	{
		if (OutErrors != nullptr)
		{
			CollectDiagnostics(Errors, *OutErrors);
		}
		return FString();
	}
	return gen::FMjStrPolicy::FromUtf8(Text);
}

}  // namespace urlab::spec::io
