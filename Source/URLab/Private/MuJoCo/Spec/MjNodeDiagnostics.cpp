// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjNodeDiagnostics.h"

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "Utils/URLabLogging.h"

#if WITH_EDITOR
#include "Logging/MessageLog.h"
#endif

namespace
{
/**
 * Elements already told about, by identity and rule rather than by object.
 *
 * The same reasoning as `GReportedIllegalPlacement`: a component is not the
 * same object from one Blueprint reconstruct to the next, so a message keyed on
 * the object is said again on every recompile, while `Serial` survives a
 * reconstruct and a genuinely new element mints a fresh one. The rule is part of
 * the key because an element can break two of them, and clearing one must not
 * silence the other.
 */
TSet<TPair<uint64, uint8>> GReportedPreviewProblems;
} // namespace

namespace urlab::spec
{
void MjNodeNotePreviewProblem(UMjNodeComponent& Node, EMjPreviewProblem Problem, const FString& Message)
{
	// A class default object is not an element: it has no spec, and nothing the
	// user can see is derived from its picture.
	if (Node.HasAnyFlags(RF_ClassDefaultObject))
	{
		return;
	}

	// The keys and the rows are one table in two arrays, and a mismatch between
	// them could only come from something writing the rows directly. Rebuilding
	// is the recovery, because the rows are re-derived anyway.
	if (Node.PreviewProblemKeys.Num() != Node.PreviewProblems.Num())
	{
		Node.PreviewProblemKeys.Reset();
		Node.PreviewProblems.Reset();
	}

	const int32 Existing = Node.PreviewProblemKeys.Find(Problem);
	if (Existing != INDEX_NONE)
	{
		Node.PreviewProblems[Existing] = Message;
	}
	else
	{
		Node.PreviewProblemKeys.Add(Problem);
		Node.PreviewProblems.Add(Message);
	}

	// The row above is the persistent surface and is rewritten every time. The
	// two logs are a transition: said when the element starts breaking the rule,
	// not once per registration for the rest of the session.
	const TPair<uint64, uint8> Key(Node.Serial, static_cast<uint8>(Problem));
	if (GReportedPreviewProblems.Contains(Key))
	{
		return;
	}
	GReportedPreviewProblems.Add(Key);

	const FString Line = FString::Printf(TEXT("%s: %s"), *Node.MjName.Get(Node.GetName()), *Message);
	UE_LOG(LogURLab, Warning, TEXT("%s"), *Line);
#if WITH_EDITOR
	FMessageLog(TEXT("URLab")).Warning(FText::FromString(Line));
#endif
}

void MjNodeClearPreviewProblem(UMjNodeComponent& Node, EMjPreviewProblem Problem)
{
	const int32 Existing = Node.PreviewProblemKeys.Find(Problem);
	if (Existing != INDEX_NONE && Node.PreviewProblems.IsValidIndex(Existing))
	{
		Node.PreviewProblemKeys.RemoveAt(Existing);
		Node.PreviewProblems.RemoveAt(Existing);
	}

	// Breaking the same rule a second time is a new mistake, and is said again.
	GReportedPreviewProblems.Remove(TPair<uint64, uint8>(Node.Serial, static_cast<uint8>(Problem)));
}
} // namespace urlab::spec
