// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

// Test-only comparison of a body's physical inertia. MuJoCo's eigensolver may
// choose different axes when principal moments coincide, even when the inertia
// tensor in body coordinates is identical.

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

#include <mujoco/mujoco.h>

namespace MjModelSemanticCompare
{

template <typename T>
inline bool SameFieldValue(T A, T B)
{
	if constexpr (std::is_floating_point_v<T>)
	{
		const double AValue = static_cast<double>(A);
		const double BValue = static_cast<double>(B);
		if ((std::isnan(AValue) && std::isnan(BValue)) || AValue == BValue) return true;
		return std::isfinite(AValue) && std::isfinite(BValue)
			&& std::abs(AValue - BValue) <= 1e-9 * (1.0 + std::max(std::abs(AValue), std::abs(BValue)));
	}
	else
	{
		return A == B;
	}
}

struct FBounds
{
	double Min[3]{};
	double Max[3]{};
};

inline double BoundsTolerance(double A, double B, double C, double D)
{
	return 64.0 * std::numeric_limits<double>::epsilon()
		* std::max(1.0, std::max(std::max(std::abs(A), std::abs(B)),
			std::max(std::abs(C), std::abs(D))));
}

inline bool ReadBounds(const mjModel& Model, int Node, FBounds& Out)
{
	for (int Axis = 0; Axis < 3; ++Axis)
	{
		const double Center = Model.bvh_aabb[6 * Node + Axis];
		const double Extent = Model.bvh_aabb[6 * Node + Axis + 3];
		if (!std::isfinite(Center) || !std::isfinite(Extent) || Extent < 0.0) return false;
		Out.Min[Axis] = Center - Extent;
		Out.Max[Axis] = Center + Extent;
	}
	return true;
}

inline bool Encloses(const FBounds& Outer, const FBounds& Inner)
{
	for (int Axis = 0; Axis < 3; ++Axis)
	{
		// Native BVH nodes are stored as center + half extent. Converting that
		// representation back to endpoints can lose a few ULPs at the scale of a
		// very large extent (for example a world plane spanning 1e10 m). Scale this
		// containment check by every operand, including the opposite endpoint;
		// model fields and small bounds retain their existing strict tolerances.
		const double MinTolerance = BoundsTolerance(
			Outer.Min[Axis], Outer.Max[Axis], Inner.Min[Axis], Inner.Max[Axis]);
		const double MaxTolerance = MinTolerance;
		if (Outer.Min[Axis] > Inner.Min[Axis] + MinTolerance
			|| Outer.Max[Axis] < Inner.Max[Axis] - MaxTolerance)
		{
			return false;
		}
	}
	return true;
}

inline bool ExpectedBodyGeomBounds(const mjModel& Model, int Body, int Geom, FBounds& Out)
{
	const double InverseInertialQuat[4] = {
		Model.body_iquat[4 * Body], -Model.body_iquat[4 * Body + 1],
		-Model.body_iquat[4 * Body + 2], -Model.body_iquat[4 * Body + 3]};
	for (int Axis = 0; Axis < 3; ++Axis)
	{
		Out.Min[Axis] = std::numeric_limits<double>::infinity();
		Out.Max[Axis] = -std::numeric_limits<double>::infinity();
	}
	for (int Corner = 0; Corner < 8; ++Corner)
	{
		double Local[3];
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			const double Center = Model.geom_aabb[6 * Geom + Axis];
			const double Extent = Model.geom_aabb[6 * Geom + Axis + 3];
			if (!std::isfinite(Center) || !std::isfinite(Extent) || Extent < 0.0) return false;
			Local[Axis] = Center + ((Corner & (1 << Axis)) ? Extent : -Extent);
		}
		double Rotated[3];
		mju_rotVecQuat(Rotated, Local, Model.geom_quat + 4 * Geom);
		double Relative[3];
		for (int Axis = 0; Axis < 3; ++Axis)
			Relative[Axis] = Rotated[Axis] + Model.geom_pos[3 * Geom + Axis] - Model.body_ipos[3 * Body + Axis];
		double Inertial[3];
		mju_rotVecQuat(Inertial, Relative, InverseInertialQuat);
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			Out.Min[Axis] = std::min(Out.Min[Axis], Inertial[Axis]);
			Out.Max[Axis] = std::max(Out.Max[Axis], Inertial[Axis]);
		}
	}
	return true;
}

// Validate the rigid-body BVHs independently of their serialized leaf order.
// MuJoCo stores these boxes in each body's inertial frame. Mesh and flex trees
// have separate leaf domains and remain strict in the compiled-model comparator.
inline bool ValidateBodyBvh(const mjModel& Model, std::string* OutError = nullptr)
{
	auto Fail = [&](const std::string& Message) {
		if (OutError != nullptr) *OutError = Message;
		return false;
	};
	if (Model.nbody <= 0 || Model.nbvhstatic < 0) return Fail("invalid body or static-BVH count");
	std::vector<bool> Claimed(static_cast<size_t>(Model.nbvhstatic), false);
	for (int Body = 0; Body < Model.nbody; ++Body)
	{
		const int Start = Model.body_bvhadr[Body];
		const int Count = Model.body_bvhnum[Body];
		if (Count < 0) return Fail("negative body BVH node count");
		std::vector<int> ExpectedGeoms;
		for (int Geom = 0; Geom < Model.ngeom; ++Geom)
		{
			if (Model.geom_bodyid[Geom] == Body && (Model.geom_contype[Geom] || Model.geom_conaffinity[Geom]))
				ExpectedGeoms.push_back(Geom);
		}
		if (Count == 0)
		{
			if (Start != -1) return Fail("body without a BVH has a nonnegative address");
			if (!ExpectedGeoms.empty()) return Fail("collidable body geoms have no BVH");
			continue;
		}
		if (ExpectedGeoms.empty()) return Fail("body BVH has no collidable geoms");
		if (Start < 0 || Start > Model.nbvhstatic - Count)
			return Fail("body BVH range is outside static node storage");
		for (int Local = 0; Local < Count; ++Local)
		{
			const int Global = Start + Local;
			if (Claimed[Global]) return Fail("body BVH ranges overlap");
			Claimed[Global] = true;
		}

		std::vector<int> FoundGeoms;
		std::vector<unsigned char> State(static_cast<size_t>(Count), 0);
		std::vector<FBounds> NodeBounds(static_cast<size_t>(Count));
		std::function<bool(int, int)> Visit = [&](int Local, int ExpectedDepth) {
			if (Local < 0 || Local >= Count) return false;
			if (State[Local] != 0) return false;
			State[Local] = 1;
			const int Global = Start + Local;
			if (Model.bvh_depth[Global] != ExpectedDepth || !ReadBounds(Model, Global, NodeBounds[Local])) return false;
			const int Left = Model.bvh_child[2 * Global];
			const int Right = Model.bvh_child[2 * Global + 1];
			const int Geom = Model.bvh_nodeid[Global];
			if (Geom >= 0)
			{
				if (Left != -1 || Right != -1 || Geom >= Model.ngeom
					|| Model.geom_bodyid[Geom] != Body
					|| !(Model.geom_contype[Geom] || Model.geom_conaffinity[Geom])) return false;
				FBounds GeomBounds;
				if (!ExpectedBodyGeomBounds(Model, Body, Geom, GeomBounds)
					|| !Encloses(NodeBounds[Local], GeomBounds)) return false;
				FoundGeoms.push_back(Geom);
			}
			else
			{
				if (Geom != -1 || Left < 0 || Right < 0 || Left == Right
					|| !Visit(Left, ExpectedDepth + 1) || !Visit(Right, ExpectedDepth + 1)
					|| !Encloses(NodeBounds[Local], NodeBounds[Left])
					|| !Encloses(NodeBounds[Local], NodeBounds[Right])) return false;
			}
			State[Local] = 2;
			return true;
		};
		if (!Visit(0, 0)) return Fail("body BVH has invalid node links, depths, leaf IDs or bounds");
		if (std::find(State.begin(), State.end(), 0) != State.end())
			return Fail("body BVH contains unreachable nodes");
		std::sort(ExpectedGeoms.begin(), ExpectedGeoms.end());
		std::sort(FoundGeoms.begin(), FoundGeoms.end());
		if (ExpectedGeoms != FoundGeoms) return Fail("body BVH leaf IDs do not cover its collidable geoms exactly once");
	}
	return true;
}

struct FContactSnapshot
{
	int Geom1 = -1;
	int Geom2 = -1;
	double Distance = 0.0;
	double Position[3]{};
	double Force[6]{};
};

inline bool Close(double A, double B, double Tolerance = 1e-9)
{
	return std::isfinite(A) && std::isfinite(B)
		&& std::abs(A - B) <= Tolerance * (1.0 + std::max(std::abs(A), std::abs(B)));
}

// Small deterministic state sweep used only when two valid body BVHs have
// different derived nodes/bounds. Compare physical outputs at identical qpos/qvel.
inline bool EquivalentContactPhysics(const mjModel& A, const mjModel& B, std::string* OutError = nullptr)
{
	auto Fail = [&](const std::string& Message) {
		if (OutError != nullptr) *OutError = Message;
		return false;
	};
	if (A.nq != B.nq || A.nv != B.nv || A.ngeom != B.ngeom || A.nbody != B.nbody)
		return Fail("contact probe requires matching model dimensions");
	mjData* DataA = mj_makeData(&A);
	mjData* DataB = mj_makeData(&B);
	if (DataA == nullptr || DataB == nullptr)
	{
		mj_deleteData(DataA);
		mj_deleteData(DataB);
		return Fail("could not allocate contact-probe data");
	}
	std::vector<mjtNum> DenseA(static_cast<size_t>(A.nv) * A.nv);
	std::vector<mjtNum> DenseB(static_cast<size_t>(B.nv) * B.nv);
	std::vector<mjtNum> Velocity(static_cast<size_t>(A.nv));
	bool Equivalent = true;
	std::string Failure;
	for (int Sample = 0; Sample < 9 && Equivalent; ++Sample)
	{
		mju_copy(DataA->qpos, A.qpos0, A.nq);
		mju_copy(DataB->qpos, B.qpos0, B.nq);
		for (int V = 0; V < A.nv; ++V)
			Velocity[V] = Sample == 0 ? 0.0 : 0.01 * std::sin((V + 1) * (Sample + 3) * 0.6180339887498948);
		if (Sample != 0)
		{
			mj_integratePos(&A, DataA->qpos, Velocity.data(), 0.015 * Sample);
			mj_integratePos(&B, DataB->qpos, Velocity.data(), 0.015 * Sample);
		}
		mju_copy(DataA->qvel, Velocity.data(), A.nv);
		mju_copy(DataB->qvel, Velocity.data(), B.nv);
		mj_forward(&A, DataA);
		mj_forward(&B, DataB);
		mj_fullM(&A, DataA, DenseA.data());
		mj_fullM(&B, DataB, DenseB.data());
		for (int I = 0; I < A.nv * A.nv && Equivalent; ++I)
			if (!Close(DenseA[I], DenseB[I])) { Equivalent = false; Failure = "mass matrix differs in deterministic state sweep"; }
		for (int I = 0; I < A.nv && Equivalent; ++I)
		{
			if (!Close(DataA->qfrc_bias[I], DataB->qfrc_bias[I])
				|| !Close(DataA->qfrc_passive[I], DataB->qfrc_passive[I])
				|| !Close(DataA->qfrc_actuator[I], DataB->qfrc_actuator[I])
				|| !Close(DataA->qfrc_constraint[I], DataB->qfrc_constraint[I]))
			{
				Equivalent = false;
				Failure = "generalized or constraint force differs in deterministic state sweep";
			}
		}
		for (int I = 0; I < A.ngeom * 3 && Equivalent; ++I)
			if (!Close(DataA->geom_xpos[I], DataB->geom_xpos[I])) { Equivalent = false; Failure = "geom world position differs"; }
		for (int I = 0; I < A.ngeom * 9 && Equivalent; ++I)
			if (!Close(DataA->geom_xmat[I], DataB->geom_xmat[I])) { Equivalent = false; Failure = "geom world rotation differs"; }
		std::vector<FContactSnapshot> ContactsA;
		std::vector<FContactSnapshot> ContactsB;
		auto Capture = [](const mjModel& Model, const mjData& Data, std::vector<FContactSnapshot>& Contacts) {
			Contacts.reserve(Data.ncon);
			for (int I = 0; I < Data.ncon; ++I)
			{
				const mjContact& Contact = Data.contact[I];
				FContactSnapshot Item;
				Item.Geom1 = Contact.geom[0];
				Item.Geom2 = Contact.geom[1];
				Item.Distance = Contact.dist;
				mju_copy(Item.Position, Contact.pos, 3);
				mj_contactForce(&Model, &Data, I, Item.Force);
				Contacts.push_back(Item);
			}
			auto Less = [](const FContactSnapshot& L, const FContactSnapshot& R) {
				return std::tie(L.Geom1, L.Geom2, L.Position[0], L.Position[1], L.Position[2], L.Distance)
					< std::tie(R.Geom1, R.Geom2, R.Position[0], R.Position[1], R.Position[2], R.Distance);
			};
			std::sort(Contacts.begin(), Contacts.end(), Less);
		};
		Capture(A, *DataA, ContactsA);
		Capture(B, *DataB, ContactsB);
		if (ContactsA.size() != ContactsB.size()) { Equivalent = false; Failure = "contact count differs in deterministic state sweep"; }
		for (size_t I = 0; I < ContactsA.size() && Equivalent; ++I)
		{
			if (ContactsA[I].Geom1 != ContactsB[I].Geom1 || ContactsA[I].Geom2 != ContactsB[I].Geom2
				|| !Close(ContactsA[I].Distance, ContactsB[I].Distance)
				|| !Close(ContactsA[I].Position[0], ContactsB[I].Position[0])
				|| !Close(ContactsA[I].Position[1], ContactsB[I].Position[1])
				|| !Close(ContactsA[I].Position[2], ContactsB[I].Position[2]))
			{
				Equivalent = false;
				Failure = "contact pair, distance or position differs in deterministic state sweep";
			}
			for (int Component = 0; Component < 6 && Equivalent; ++Component)
				if (!Close(ContactsA[I].Force[Component], ContactsB[I].Force[Component]))
				{
					Equivalent = false;
					Failure = "contact force differs in deterministic state sweep";
				}
		}
	}
	mj_deleteData(DataA);
	mj_deleteData(DataB);
	if (!Equivalent) return Fail(Failure);
	return true;
}

inline void InertiaTensor(const double* Diagonal, const double* Quaternion, double Out[9])
{
	const double W = Quaternion[0];
	const double X = Quaternion[1];
	const double Y = Quaternion[2];
	const double Z = Quaternion[3];
	const double Rotation[9] = {
		1 - 2 * (Y * Y + Z * Z), 2 * (X * Y - Z * W), 2 * (X * Z + Y * W),
		2 * (X * Y + Z * W), 1 - 2 * (X * X + Z * Z), 2 * (Y * Z - X * W),
		2 * (X * Z - Y * W), 2 * (Y * Z + X * W), 1 - 2 * (X * X + Y * Y),
	};
	for (int Row = 0; Row < 3; ++Row)
	{
		for (int Column = 0; Column < 3; ++Column)
		{
			Out[3 * Row + Column] = 0;
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				Out[3 * Row + Column] += Rotation[3 * Row + Axis] * Diagonal[Axis]
					* Rotation[3 * Column + Axis];
			}
		}
	}
}

inline bool EquivalentBodyInertia(const double* DiagonalA, const double* QuaternionA,
	const double* DiagonalB, const double* QuaternionB, double* OutMaxError = nullptr)
{
	double A[9];
	double B[9];
	InertiaTensor(DiagonalA, QuaternionA, A);
	InertiaTensor(DiagonalB, QuaternionB, B);
	double MaxError = 0;
	double Scale = 0;
	for (int Index = 0; Index < 9; ++Index)
	{
		if (!std::isfinite(A[Index]) || !std::isfinite(B[Index])) return false;
		MaxError = std::max(MaxError, std::abs(A[Index] - B[Index]));
		Scale = std::max(Scale, std::max(std::abs(A[Index]), std::abs(B[Index])));
	}
	if (OutMaxError != nullptr) *OutMaxError = MaxError;
	return MaxError <= 1e-12 + 1e-10 * Scale;
}

} // namespace MjModelSemanticCompare
