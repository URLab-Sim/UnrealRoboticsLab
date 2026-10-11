// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjModelSemanticCompare.h"

#if WITH_EDITOR

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBodyInertiaSemanticComparisonTest,
	"URLab.Parity.BodyInertiaSemanticComparison",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBodyInertiaSemanticComparisonTest::RunTest(const FString&)
{
	using MjModelSemanticCompare::EquivalentBodyInertia;
	const double RootHalf = FMath::Sqrt(0.5);
	const double Identity[4] = {1, 0, 0, 0};
	const double AroundZ[4] = {RootHalf, 0, 0, RootHalf};
	const double AroundX[4] = {RootHalf, RootHalf, 0, 0};
	const double Degenerate[3] = {2, 2, 1};
	const double Distinct[3] = {1, 2, 3};

	TestTrue(TEXT("rotation within degenerate principal plane preserves tensor"),
		EquivalentBodyInertia(Degenerate, Identity, Degenerate, AroundZ));
	TestFalse(TEXT("rotating distinct principal moments changes tensor"),
		EquivalentBodyInertia(Distinct, Identity, Distinct, AroundX));

	// Gymnasium humanoid right_lower_arm: fromto normalization leaves the same
	// physical inertia, while MuJoCo selects a different degenerate eigenbasis.
	const double OriginalMoment[3] = {
		0.013667228332107468, 0.013667228332107467, 0.0006215610823377786};
	const double WrittenMoment[3] = {
		0.013667228332107473, 0.013667228332107472, 0.00062156108233777946};
	const double OriginalQuat[4] = {
		0.88047623921714935, -0.36470519963100095, 0.27984814233312122, -0.11591689595929512};
	const double WrittenQuat[4] = {
		0.4247082002778671, 0.82047323857028398, -0.3398511429799877, -0.17591989660616131};
	double MaxError = 0;
	TestTrue(TEXT("humanoid inertial frames reconstruct the same tensor"),
		EquivalentBodyInertia(OriginalMoment, OriginalQuat, WrittenMoment, WrittenQuat, &MaxError));
	TestTrue(TEXT("humanoid tensor difference is roundoff"), MaxError < 1e-14);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBodyBvhSemanticValidationTest,
	"URLab.Parity.BodyBvhSemanticValidation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBodyBvhSemanticValidationTest::RunTest(const FString&)
{
	using MjModelSemanticCompare::ValidateBodyBvh;
	mjModel Model{};
	Model.nbody = 2;
	Model.ngeom = 2;
	Model.nbvh = 3;
	Model.nbvhstatic = 3;
	int BodyAdr[2] = {-1, 0};
	int BodyNum[2] = {0, 3};
	double BodyIpos[6] = {};
	double BodyIquat[8] = {1, 0, 0, 0, 1, 0, 0, 0};
	int GeomBody[2] = {1, 1};
	int GeomContype[2] = {1, 1};
	int GeomConaffinity[2] = {1, 1};
	double GeomPos[6] = {-1, 0, 0, 1, 0, 0};
	double GeomQuat[8] = {1, 0, 0, 0, 1, 0, 0, 0};
	double GeomAabb[12] = {0, 0, 0, .1, .1, .1, 0, 0, 0, .1, .1, .1};
	int BvhDepth[3] = {0, 1, 1};
	int BvhChild[6] = {1, 2, -1, -1, -1, -1};
	int BvhNodeid[3] = {-1, 0, 1};
	double BvhAabb[18] = {
		0, 0, 0, 1.1, .1, .1,
		-1, 0, 0, .1, .1, .1,
		1, 0, 0, .1, .1, .1};
	Model.body_bvhadr = BodyAdr;
	Model.body_bvhnum = BodyNum;
	Model.body_ipos = BodyIpos;
	Model.body_iquat = BodyIquat;
	Model.geom_bodyid = GeomBody;
	Model.geom_contype = GeomContype;
	Model.geom_conaffinity = GeomConaffinity;
	Model.geom_pos = GeomPos;
	Model.geom_quat = GeomQuat;
	Model.geom_aabb = GeomAabb;
	Model.bvh_depth = BvhDepth;
	Model.bvh_child = BvhChild;
	Model.bvh_nodeid = BvhNodeid;
	Model.bvh_aabb = BvhAabb;
	std::string Error;
	TestTrue(TEXT("well formed body tree covers both collidable geoms"), ValidateBodyBvh(Model, &Error));

	BvhChild[0] = 3;
	TestFalse(TEXT("out of range child is rejected"), ValidateBodyBvh(Model, &Error));
	BvhChild[0] = 1;
	BvhNodeid[2] = 0;
	TestFalse(TEXT("valid-looking duplicate geom leaf is rejected"), ValidateBodyBvh(Model, &Error));
	BvhNodeid[2] = 1;
	BvhAabb[3] = .5;
	TestFalse(TEXT("undersized parent bound is rejected"), ValidateBodyBvh(Model, &Error));
	BvhAabb[3] = 1.1;
	BvhAabb[6 + 3] = .05;
	TestFalse(TEXT("undersized leaf bound is rejected"), ValidateBodyBvh(Model, &Error));
	BvhAabb[6 + 3] = .1;

	MjModelSemanticCompare::FBounds HugeOuter;
	MjModelSemanticCompare::FBounds HugeChild;
	for (int Axis = 0; Axis < 3; ++Axis)
	{
		HugeOuter.Min[Axis] = -1e10;
		HugeOuter.Max[Axis] = 1e10;
		HugeChild.Min[Axis] = -1e10;
		HugeChild.Max[Axis] = 1e10;
	}
	// Center/half-extent reconstruction can make a child endpoint exceed its
	// parent's reconstructed endpoint by sub-micrometre roundoff at this scale.
	HugeOuter.Max[2] = 0.39999961853027344;
	HugeChild.Min[2] = -0.40000000000000036;
	HugeChild.Max[2] = 0.40000000000000036;
	TestTrue(TEXT("large BVH bounds allow endpoint reconstruction roundoff"),
		MjModelSemanticCompare::Encloses(HugeOuter, HugeChild));
	HugeOuter.Max[2] -= 0.01;
	TestFalse(TEXT("a materially clipped large BVH bound is still rejected"),
		MjModelSemanticCompare::Encloses(HugeOuter, HugeChild));

	TestFalse(TEXT("wrong geom size remains a physical field mismatch"),
		MjModelSemanticCompare::SameFieldValue(0.5, 0.501));
	TestFalse(TEXT("wrong body mass remains a physical field mismatch"),
		MjModelSemanticCompare::SameFieldValue(1.0, 1.01));
	TestFalse(TEXT("wrong contact mask remains a structural field mismatch"),
		MjModelSemanticCompare::SameFieldValue(1, 0));
	TestTrue(TEXT("one-ulp geometry roundoff stays within established float tolerance"),
		MjModelSemanticCompare::SameFieldValue(0.031, 0.031000000000000003));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjContactPhysicsSemanticComparisonTest,
	"URLab.Parity.ContactPhysicsSemanticComparison",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjContactPhysicsSemanticComparisonTest::RunTest(const FString&)
{
	const FString Path = FPaths::Combine(FPaths::ProjectIntermediateDir(), TEXT("MjSemanticContactProbe.xml"));
	const FString Xml = TEXT(R"(<mujoco model="contact_probe">
  <worldbody>
    <body name="left"><freejoint/><geom name="left_ball" type="sphere" size="0.5"/></body>
    <body name="right" pos="0.8 0 0"><freejoint/><geom name="right_ball" type="sphere" size="0.5"/></body>
  </worldbody>
</mujoco>)");
	if (!FFileHelper::SaveStringToFile(Xml, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
	{
		AddError(TEXT("could not write contact probe XML"));
		return false;
	}
	char Error[1024] = {};
	mjModel* Original = mj_loadXML(TCHAR_TO_UTF8(*Path), nullptr, Error, sizeof(Error));
	IFileManager::Get().Delete(*Path);
	if (!TestNotNull(TEXT("compiled contact probe model"), Original))
	{
		AddError(UTF8_TO_TCHAR(Error));
		return false;
	}
	mjModel* WrongMask = mj_copyModel(nullptr, Original);
	if (!TestNotNull(TEXT("copied contact probe model"), WrongMask))
	{
		mj_deleteModel(Original);
		return false;
	}
	WrongMask->geom_contype[1] = 0;
	WrongMask->geom_conaffinity[1] = 0;
	std::string ProbeError;
	TestFalse(TEXT("changed collision mask changes contact behavior"),
		MjModelSemanticCompare::EquivalentContactPhysics(*Original, *WrongMask, &ProbeError));
	mj_deleteModel(WrongMask);
	mj_deleteModel(Original);
	return true;
}

#endif // WITH_EDITOR
