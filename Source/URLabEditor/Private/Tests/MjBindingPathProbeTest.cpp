// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Empirical probe: does the id an element is told after a compile agree with
// the id its compiled name resolves to?
//
// There is one binding path now. Binding is by name, and the element retains
// nothing but the integer it was given, so the only thing that can be wrong is
// that integer. For each element, after the compile:
//   - Name id  = mj_name2id(model, type, prefix + name)
//   - Bound id = the element's own GetBoundId()
// The test logs both and reports agreement per element. Its job is to report,
// not to gate, so it always passes.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjGeom.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMjBindingPathProbeTest,
	"URLab.MuJoCo.Binding.PathProbe",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

namespace
{
enum class EPathResult : uint8
{
	Success,
	Stale,
	Failed,
	Unbound
};

struct FProbeResult
{
	FString Element;
	FString MjType;
	int NameId;
	int BoundId;
	EPathResult NameOutcome;
	EPathResult BoundOutcome;
};

const TCHAR* PathResultName(EPathResult R)
{
	switch (R)
	{
		case EPathResult::Success:
			return TEXT("SUCCESS");
		case EPathResult::Stale:
			return TEXT("STALE_ID");
		case EPathResult::Failed:
			return TEXT("FAILED");
		case EPathResult::Unbound:
			return TEXT("UNBOUND");
	}
	return TEXT("?");
}

int GetMjCountFor(const mjModel* m, mjtObj Type)
{
	switch (Type)
	{
		case mjOBJ_BODY:
			return m->nbody;
		case mjOBJ_JOINT:
			return m->njnt;
		case mjOBJ_GEOM:
			return m->ngeom;
		case mjOBJ_SITE:
			return m->nsite;
		case mjOBJ_CAMERA:
			return m->ncam;
		case mjOBJ_LIGHT:
			return m->nlight;
		case mjOBJ_MESH:
			return m->nmesh;
		case mjOBJ_HFIELD:
			return m->nhfield;
		case mjOBJ_TEXTURE:
			return m->ntex;
		case mjOBJ_MATERIAL:
			return m->nmat;
		case mjOBJ_PAIR:
			return m->npair;
		case mjOBJ_EXCLUDE:
			return m->nexclude;
		case mjOBJ_EQUALITY:
			return m->neq;
		case mjOBJ_TENDON:
			return m->ntendon;
		case mjOBJ_ACTUATOR:
			return m->nu;
		case mjOBJ_SENSOR:
			return m->nsensor;
		case mjOBJ_NUMERIC:
			return m->nnumeric;
		case mjOBJ_TEXT:
			return m->ntext;
		case mjOBJ_TUPLE:
			return m->ntuple;
		case mjOBJ_KEY:
			return m->nkey;
		case mjOBJ_PLUGIN:
			return m->nplugin;
		default:
			return 0;
	}
}

/**
 * A bound id out of range of its family is worse than a missing one: it indexes
 * a live array and reads someone else's state rather than refusing.
 */
EPathResult ClassifyBound(const TOptional<int32>& Id, int MaxCount)
{
	if (!Id.IsSet())
		return EPathResult::Unbound;
	if (Id.GetValue() < 0)
		return EPathResult::Failed;
	if (Id.GetValue() >= MaxCount)
		return EPathResult::Stale;
	return EPathResult::Success;
}

FProbeResult Probe(
	const TCHAR* Label,
	mjtObj Type,
	const UMjNodeComponent* Element,
	const FMjUESession& Session)
{
	FProbeResult R;
	R.Element = Label;
	R.MjType = FString::Printf(TEXT("%d"), (int)Type);

	const int MaxCount = GetMjCountFor(Session.Model(), Type);
	R.NameId = Session.MjId(Type, Label);
	R.NameOutcome = R.NameId >= 0 ? EPathResult::Success : EPathResult::Failed;

	R.BoundId = Element->GetBoundId().Get(-1);
	R.BoundOutcome = ClassifyBound(Element->GetBoundId(), MaxCount);

	return R;
}
} // namespace

bool FMjBindingPathProbeTest::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	const mjModel* Model = S.Model();
	if (!Model)
	{
		AddError(TEXT("Compile produced no model. Probe aborted."));
		return false;
	}
	if (Model->nbody < 2 || Model->ngeom < 1 || Model->njnt < 1)
	{
		AddError(FString::Printf(
			TEXT("Compiled model is missing expected entities (nbody=%d, ngeom=%d, njnt=%d). Expected nbody>=2, ngeom>=1, njnt>=1. Probe aborted."),
			Model->nbody, Model->ngeom, Model->njnt));
		return false;
	}

	const FString Prefix = S.Robot->GetCompiledPrefix();
	UE_LOG(LogTemp, Display, TEXT("[PathProbe] Articulation prefix: '%s'"), *Prefix);
	UE_LOG(LogTemp, Display, TEXT("[PathProbe] Model counts: nbody=%d  ngeom=%d  njnt=%d  (mj_compile completed)"),
		Model->nbody, Model->ngeom, Model->njnt);

	TArray<FProbeResult> Results;
	Results.Add(Probe(TEXT("RootBody"), mjOBJ_BODY, S.Body, S));
	Results.Add(Probe(TEXT("TestGeom"), mjOBJ_GEOM, S.Geom, S));
	Results.Add(Probe(TEXT("TestJoint"), mjOBJ_JOINT, S.Joint, S));

	int NameSuccesses = 0, NameFailures = 0;
	int BoundSuccesses = 0, BoundStales = 0, BoundFailures = 0, BoundUnbound = 0;
	int Agreements = 0;

	for (const FProbeResult& R : Results)
	{
		UE_LOG(LogTemp, Display,
			TEXT("[PathProbe] [%s] NameId=%d (%s)  BoundId=%d (%s)  Name==Bound:%d"),
			*R.Element, R.NameId, PathResultName(R.NameOutcome),
			R.BoundId, PathResultName(R.BoundOutcome),
			(int)(R.NameId == R.BoundId && R.BoundId >= 0));

		switch (R.BoundOutcome)
		{
			case EPathResult::Success:
				++BoundSuccesses;
				break;
			case EPathResult::Stale:
				++BoundStales;
				break;
			case EPathResult::Failed:
				++BoundFailures;
				break;
			case EPathResult::Unbound:
				++BoundUnbound;
				break;
		}
		if (R.NameOutcome == EPathResult::Success)
			++NameSuccesses;
		else
			++NameFailures;

		if (R.NameOutcome == EPathResult::Success && R.NameId == R.BoundId)
			++Agreements;
	}

	UE_LOG(LogTemp, Display, TEXT("[PathProbe] === SUMMARY (n=%d) ==="), Results.Num());
	UE_LOG(LogTemp, Display, TEXT("[PathProbe] Name lookup: success=%d  failed=%d"),
		NameSuccesses, NameFailures);
	UE_LOG(LogTemp, Display, TEXT("[PathProbe] Bound id: success=%d  stale=%d  failed=%d  unbound=%d"),
		BoundSuccesses, BoundStales, BoundFailures, BoundUnbound);
	UE_LOG(LogTemp, Display, TEXT("[PathProbe] Name==Bound: %d/%d"), Agreements, Results.Num());

	if (Agreements == Results.Num())
	{
		UE_LOG(LogTemp, Display, TEXT("[PathProbe] VERDICT: every element bound to the id its compiled name resolves to."));
	}
	else
	{
		UE_LOG(LogTemp, Display, TEXT("[PathProbe] VERDICT: %d/%d elements disagree with their compiled name."),
			Results.Num() - Agreements, Results.Num());
	}

	S.Cleanup();
	return true;
}
