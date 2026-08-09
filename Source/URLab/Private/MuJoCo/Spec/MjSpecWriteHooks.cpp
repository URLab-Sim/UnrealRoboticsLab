// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MjSpecWriteHooks.h"

#if URLAB_MJ_GEN

#include <string>
#include <type_traits>

#include "MuJoCo/Gen/MjElements.gen.h"
#include "MuJoCo/Gen/MjKeywords.gen.h"
#include "MuJoCo/Gen/MjSpecWrite.gen.h"
#include "MuJoCo/Gen/MjStorage.gen.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

namespace urlab::spec
{
namespace
{

namespace sw = ps::ue::specwrite;

using ps::mjcf::ElementType;

/** The children of the node that owns this one, in authored order. */
TArray<FMjOrderedChild> Siblings(const FMjSpecWriteContext& Ctx)
{
	if (Ctx.Source == nullptr || Ctx.ParentNode == nullptr)
	{
		return {};
	}
	return MjOrderedChildrenOf(*Ctx.Source,
		*const_cast<UMjNodeComponent*>(Ctx.ParentNode));
}

template <class T> struct TIsEnumList : std::false_type {};
template <class T> struct TIsEnumList<TArray<T>> : std::true_type {};

/** One candidate operand spelling, and the object kind electing it implies. */
struct FOperand
{
	const TOptional<FString>* Value = nullptr;
	mjtObj Kind = mjOBJ_UNKNOWN;
};

// --- Small conversions ---------------------------------------------------- //

const char* Utf8(const FString& Value)
{
	// A ring rather than one buffer: several of these appear as separate
	// arguments of a single mjs_* call, so each conversion has to survive until
	// that call runs, and a shared buffer would leave all but the last dangling.
	constexpr int32 Slots = 8;
	static thread_local std::string Ring[Slots];
	static thread_local int32 Next = 0;
	std::string& Slot = Ring[Next];
	Next = (Next + 1) % Slots;
	Slot = ps::ue::FMjStrPolicy::ToUtf8(FStringView(Value));
	return Slot.c_str();
}

/** The keyword set as MuJoCo's bit field: one bit per member, by index. */
template <class TEnum>
int DataSpec(const TArray<TEnum>& Keywords, int Fallback)
{
	if (Keywords.IsEmpty())
	{
		return Fallback;
	}
	int Bits = 0;
	for (const TEnum Keyword : Keywords)
	{
		Bits |= 1 << sw::KeywordC(Keyword);
	}
	return Bits;
}

/** True when the keywords are strictly ascending, which the engine requires. */
template <class TEnum>
bool Ascending(const TArray<TEnum>& Keywords)
{
	for (int32 I = 1; I < Keywords.Num(); ++I)
	{
		if (sw::KeywordC(Keywords[I]) <= sw::KeywordC(Keywords[I - 1]))
		{
			return false;
		}
	}
	return true;
}

/** Copy up to N authored values into a C array, leaving the rest alone. */
void CopySome(const TOptional<TArray<double>>& Values, double* Out, int32 N)
{
	if (!Values.IsSet())
	{
		return;
	}
	const int32 Count = FMath::Min<int32>(Values.GetValue().Num(), N);
	for (int32 I = 0; I < Count; ++I)
	{
		Out[I] = Values.GetValue()[I];
	}
}

// --- H1, H2: actuators ---------------------------------------------------- //
// An actuator is not created by the generated dispatch: which shorthand it is
// decides gain, bias and dynamics together, and mjs_setTo* derives all three
// from parameters that INHERIT from whatever the default class already put on
// the struct. So the order is the reader's -- create, apply the shared
// attributes, elect the transmission, then configure -- and it is why the hook
// contract has a create phase at all.

mjsActuator* Actuator(FMjSpecWriteContext& Ctx)
{
	return static_cast<mjsActuator*>(Ctx.Struct);
}

bool CreateActuator(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	if (Ctx.Partial != nullptr)
	{
		// An actuator spelling under a <default> configures the class's own
		// actuator template, which is what makes a class-authored gain reach
		// the shorthands that inherit from it.
		Ctx.Struct = Ctx.Partial->actuator;
		return Ctx.Struct != nullptr
			|| Ctx.Error(Node, TEXT("this default class carries no actuator template"));
	}
	mjsActuator* const Made = mjs_addActuator(Ctx.Spec, Ctx.Class);
	if (Made == nullptr)
	{
		return Ctx.Error(Node, TEXT("could not add an actuator to the spec"));
	}
	Ctx.Struct = Made;
	Ctx.Element = Made->element;
	return true;
}

/**
 * Elect the transmission target and type.
 *
 * Exactly one of the operand attributes decides both, which is why none of them
 * can be a plain field write: `target` and `trntype` move together. Mirrors the
 * reader, with its two legality checks kept as diagnostics.
 */
bool ApplyTransmission(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	mjsActuator* const Act = Actuator(Ctx);
	if (Act == nullptr)
	{
		return Ctx.Error(Node, TEXT("transmission on something that is not an actuator"));
	}

	bool bElected = false;
	// Matched on TYPE as well as name. The walk instantiates this over every
	// element the schema declares, not just the actuators it is called for, and
	// `<visual><rgba joint=...>` is a colour that happens to share the spelling
	// -- so a name-only match binds a transmission to a quaternion's worth of
	// floats and only says so at the point of use.
	const auto Elect = [&](const auto& Target, mjtTrn Type)
	{
		if constexpr (std::is_same_v<std::decay_t<decltype(Target)>, TOptional<FString>>)
		{
			if (!Target.IsSet())
			{
				return;
			}
			mjs_setString(Act->target, Utf8(Target.GetValue()));
			Act->trntype = Type;
			bElected = true;
		}
	};

	TOptional<FString> SliderSite;
	TOptional<double> CrankLength;
	TOptional<FString> RefSite;

	gen::DispatchByType(Node, [&](const auto& Element)
	{
		if constexpr (requires { Element.Joint; }) { Elect(Element.Joint, mjTRN_JOINT); }
		if constexpr (requires { Element.Jointinparent; })
		{
			Elect(Element.Jointinparent, mjTRN_JOINTINPARENT);
		}
		if constexpr (requires { Element.Tendon; }) { Elect(Element.Tendon, mjTRN_TENDON); }
		if constexpr (requires { Element.Cranksite; })
		{
			Elect(Element.Cranksite, mjTRN_SLIDERCRANK);
		}
		if constexpr (requires { Element.Site; }) { Elect(Element.Site, mjTRN_SITE); }
		if constexpr (requires { Element.Body; }) { Elect(Element.Body, mjTRN_BODY); }
		if constexpr (requires { Element.Slidersite; }) { SliderSite = Element.Slidersite; }
		if constexpr (requires { Element.Cranklength; }) { CrankLength = Element.Cranklength; }
		if constexpr (requires { Element.Refsite; }) { RefSite = Element.Refsite; }
		(void)Element;
	});
	(void)bElected;

	if (SliderSite.IsSet())
	{
		mjs_setString(Act->slidersite, Utf8(SliderSite.GetValue()));
	}
	if (CrankLength.IsSet())
	{
		Act->cranklength = CrankLength.GetValue();
	}
	if ((CrankLength.IsSet() || SliderSite.IsSet()) &&
		Act->trntype != mjTRN_SLIDERCRANK && Act->trntype != mjTRN_UNDEFINED)
	{
		return Ctx.Error(Node,
			TEXT("cranklength and slidersite need a slidercrank transmission"));
	}

	if (RefSite.IsSet())
	{
		mjs_setString(Act->refsite, Utf8(RefSite.GetValue()));
		if (Act->trntype != mjTRN_SITE && Act->trntype != mjTRN_UNDEFINED)
		{
			return Ctx.Error(Node, TEXT("refsite needs a site transmission"));
		}
	}
	return true;
}

/** The servo input keyword or token set, folded into ctrlspec. */
template <class T>
void ApplyInput(const T& Element, mjsActuator& Act)
{
	if constexpr (requires { Element.Input; })
	{
		if (!Element.Input.IsSet())
		{
			return;
		}
		using InputType = std::decay_t<decltype(Element.Input.GetValue())>;
		if constexpr (TIsEnumList<InputType>::value)
		{
			int Bits = 0;
			for (const auto Keyword : Element.Input.GetValue())
			{
				Bits |= sw::KeywordC(Keyword);
			}
			Act.ctrlspec = Bits;
		}
		else
		{
			Act.ctrlspec = sw::KeywordC(Element.Input.GetValue());
		}
	}
}

/** A nullable scalar argument: the reader's "unset means inherit" spelling. */
double* Opt(const TOptional<double>& Value, double& Storage)
{
	if (!Value.IsSet())
	{
		return nullptr;
	}
	Storage = Value.GetValue();
	return &Storage;
}

bool ApplyShorthand(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	mjsActuator* const Act = Actuator(Ctx);
	if (Act == nullptr)
	{
		return Ctx.Error(Node, TEXT("actuator shorthand on something that is not an actuator"));
	}

	ElementType Type;
	if (!gen::ElementTypeOfNode(Node, Type))
	{
		return Ctx.Error(Node, TEXT("unrecognised actuator element"));
	}

	// Storage for the nullable arguments, which must outlive the call.
	double Kv = 0.0;
	double DampRatio = 0.0;
	double TimeConst = 0.0;
	const char* Error = nullptr;

	switch (Type)
	{
	case ElementType::ActuatorGeneral:
		ApplyInput(static_cast<const UMjActuatorGeneral&>(Node), *Act);
		break;

	case ElementType::Motor:
		Error = mjs_setToMotor(Act);
		break;

	case ElementType::Position:
	case ElementType::IntVelocity:
	{
		// kp inherits from whatever the default class left in gainprm[0], which
		// is exactly what makes a class-authored gain survive the shorthand.
		double Kp = Act->gainprm[0];
		double InheritRange = Act->inheritrange;
		const auto Configure = [&](const auto& Element)
		{
			if (Element.Kp.IsSet()) { Kp = Element.Kp.GetValue(); }
			if (Element.Inheritrange.IsSet()) { InheritRange = Element.Inheritrange.GetValue(); }
		};
		double* Timing = nullptr;
		if (Type == ElementType::Position)
		{
			const UMjPosition& Element = static_cast<const UMjPosition&>(Node);
			Configure(Element);
			Timing = Opt(Element.Timeconst, TimeConst);
			Error = mjs_setToPosition(Act, Kp, Opt(Element.Kv, Kv),
				Opt(Element.Dampratio, DampRatio), Timing, InheritRange);
		}
		else
		{
			const UMjIntVelocity& Element = static_cast<const UMjIntVelocity&>(Node);
			Configure(Element);
			Error = mjs_setToIntVelocity(Act, Kp, Opt(Element.Kv, Kv),
				Opt(Element.Dampratio, DampRatio), nullptr, InheritRange);
		}
		break;
	}

	case ElementType::OrientationActuator:
	{
		const UMjOrientationActuator& Element = static_cast<const UMjOrientationActuator&>(Node);
		double Kp = Act->gainprm[0];
		if (Element.Kp.IsSet()) { Kp = Element.Kp.GetValue(); }
		ApplyInput(Element, *Act);
		Error = mjs_setToOrientation(Act, Kp, Opt(Element.Kv, Kv),
			Opt(Element.Dampratio, DampRatio), Act->ctrlspec);
		break;
	}

	case ElementType::Pid:
	{
		const UMjPid& Element = static_cast<const UMjPid&>(Node);
		// pid's kp inherits from the bias term rather than the gain, because
		// that is where its affine bias put it.
		double Kp = -Act->biasprm[1];
		if (Element.Kp.IsSet()) { Kp = Element.Kp.GetValue(); }
		const bool bInherited = Act->dyntype == mjDYN_PID;
		double Ki = bInherited ? Act->gainprm[0] : 0.0;
		double IMax = bInherited ? Act->dynprm[0] : 0.0;
		double SlewMax = bInherited ? Act->dynprm[1] : 0.0;
		if (Element.Ki.IsSet()) { Ki = Element.Ki.GetValue(); }
		if (Element.Imax.IsSet()) { IMax = Element.Imax.GetValue(); }
		if (Element.Slewmax.IsSet()) { SlewMax = Element.Slewmax.GetValue(); }
		ApplyInput(Element, *Act);
		double InheritRange = Act->inheritrange;
		if (Element.Inheritrange.IsSet()) { InheritRange = Element.Inheritrange.GetValue(); }
		Error = mjs_setToPID(Act, Kp, Opt(Element.Kv, Kv),
			Opt(Element.Dampratio, DampRatio), &Ki, &IMax, &SlewMax, InheritRange,
			Act->ctrlspec);
		break;
	}

	case ElementType::Velocity:
	{
		const UMjVelocity& Element = static_cast<const UMjVelocity&>(Node);
		double Gain = Act->gainprm[0];
		if (Element.Kv.IsSet()) { Gain = Element.Kv.GetValue(); }
		Error = mjs_setToVelocity(Act, Gain);
		break;
	}

	case ElementType::Damper:
	{
		const UMjDamper& Element = static_cast<const UMjDamper&>(Node);
		const bool bInherited = Act->gaintype == mjGAIN_AFFINE;
		double Gain = bInherited ? -Act->gainprm[2] : 0.0;
		if (Element.Kv.IsSet()) { Gain = Element.Kv.GetValue(); }
		Error = mjs_setToDamper(Act, Gain);
		break;
	}

	case ElementType::Cylinder:
	{
		const UMjCylinder& Element = static_cast<const UMjCylinder&>(Node);
		double Time = Act->dynprm[0];
		double Bias[3] = { Act->biasprm[0], Act->biasprm[1], Act->biasprm[2] };
		double Area = Act->gainprm[0];
		// -1 is the engine's "not given"; a diameter overrides the area.
		const double Diameter = -1.0;
		if (Element.Timeconst.IsSet()) { Time = Element.Timeconst.GetValue(); }
		if (Element.Area.IsSet()) { Area = Element.Area.GetValue(); }
		if (Element.Bias.IsSet())
		{
			Bias[0] = Element.Bias.GetValue().X;
			Bias[1] = Element.Bias.GetValue().Y;
			Bias[2] = Element.Bias.GetValue().Z;
		}
		Error = mjs_setToCylinder(Act, Time, Bias[0], Area, Diameter);
		Act->biasprm[1] = Bias[1];
		Act->biasprm[2] = Bias[2];
		break;
	}

	case ElementType::Muscle:
	{
		const UMjMuscle& Element = static_cast<const UMjMuscle&>(Node);
		double TauSmooth = Act->dynprm[2];
		if (Element.Tausmooth.IsSet()) { TauSmooth = Element.Tausmooth.GetValue(); }
		double Range[2] = { -1.0, -1.0 };
		double Timing[2] = { -1.0, -1.0 };
		if (Element.Range.IsSet())
		{
			Range[0] = Element.Range.GetValue().X;
			Range[1] = Element.Range.GetValue().Y;
		}
		if (Element.Timeconst.IsSet())
		{
			Timing[0] = Element.Timeconst.GetValue().X;
			Timing[1] = Element.Timeconst.GetValue().Y;
		}
		const auto Or = [](const TOptional<double>& Value) { return Value.Get(-1.0); };
		Error = mjs_setToMuscle(Act, Timing, TauSmooth, Range, Or(Element.Force),
			Or(Element.Scale), Or(Element.Lmin), Or(Element.Lmax), Or(Element.Vmax),
			Or(Element.Fpmax), Or(Element.Fvmax));
		break;
	}

	case ElementType::Adhesion:
	{
		const UMjAdhesion& Element = static_cast<const UMjAdhesion&>(Node);
		double Gain = Act->gainprm[0];
		if (Element.Gain.IsSet()) { Gain = Element.Gain.GetValue(); }
		if (Element.Body.IsSet())
		{
			mjs_setString(Act->target, Utf8(Element.Body.GetValue()));
			Act->trntype = mjTRN_BODY;
		}
		Error = mjs_setToAdhesion(Act, Gain);
		break;
	}

	case ElementType::DcMotor:
	{
		const UMjDcMotor& Element = static_cast<const UMjDcMotor&>(Node);
		const bool bInherited = Act->gaintype == mjGAIN_DCMOTOR;
		double MotorConst[2] = { bInherited ? Act->gainprm[1] : 0.0, 0.0 };
		double Resistance = bInherited ? Act->gainprm[0] : 0.0;
		double Nominal[3] = { 0.0, 0.0, 0.0 };
		double Saturation[3] = { 0.0, 0.0, bInherited ? Act->dynprm[1] : 0.0 };
		double Controller[6] = {
			bInherited ? Act->gainprm[4] : 0.0, bInherited ? Act->gainprm[5] : 0.0,
			bInherited ? Act->gainprm[6] : 0.0, bInherited ? Act->dynprm[7] : 0.0,
			bInherited ? Act->dynprm[8] : 0.0, bInherited ? Act->gainprm[7] : 0.0 };
		double Inductance[2] = { 0.0, bInherited ? Act->dynprm[0] : 0.0 };
		double Cogging[3] = {
			bInherited ? Act->biasprm[0] : 0.0, bInherited ? Act->biasprm[1] : 0.0,
			bInherited ? Act->biasprm[2] : 0.0 };
		double Thermal[6] = {
			bInherited ? Act->dynprm[2] : 0.0, bInherited ? Act->dynprm[3] : 0.0, 0.0,
			bInherited ? Act->gainprm[2] : 0.0, bInherited ? Act->gainprm[3] : 0.0,
			bInherited ? Act->dynprm[4] : 0.0 };
		double LuGre[5] = {
			bInherited ? Act->dynprm[5] : 0.0, bInherited ? Act->dynprm[6] : 0.0,
			bInherited ? Act->biasprm[3] : 0.0, bInherited ? Act->biasprm[4] : 0.0,
			bInherited ? Act->biasprm[5] : 0.0 };
		int InputMode = bInherited ? static_cast<int>(Act->gainprm[8]) : 0;
		if (Element.Resistance.IsSet()) { Resistance = Element.Resistance.GetValue(); }
		if (Element.Input.IsSet()) { InputMode = sw::KeywordC(Element.Input.GetValue()); }
		CopySome(Element.Motorconst, MotorConst, 2);
		CopySome(Element.Nominal, Nominal, 3);
		CopySome(Element.Saturation, Saturation, 3);
		CopySome(Element.Inductance, Inductance, 2);
		CopySome(Element.Cogging, Cogging, 3);
		CopySome(Element.Controller, Controller, 6);
		CopySome(Element.Thermal, Thermal, 6);
		CopySome(Element.Lugre, LuGre, 5);
		Error = mjs_setToDCMotor(Act, MotorConst, Resistance, Nominal, Saturation,
			Inductance, Cogging, Controller, Thermal, LuGre, InputMode);
		break;
	}

	case ElementType::ActuatorPlugin:
		// The plugin hook configures the mjsPlugin member; nothing about the
		// gain family is this element's to decide.
		break;

	default:
		return Ctx.Error(Node, TEXT("unrecognised actuator shorthand"));
	}

	if (Error != nullptr && *Error != '\0')
	{
		return Ctx.Error(Node, FString(UTF8_TO_TCHAR(Error)));
	}
	return true;
}

// --- H3: tendon path ------------------------------------------------------ //
// A wrap is not an element with fields: mjs_wrap* takes its attributes as
// constructor arguments and appends to the owning tendon's path, so authored
// order IS the path order.

bool ApplyTendonPath(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	mjsTendon* const Tendon = static_cast<mjsTendon*>(Ctx.Parent);
	if (Tendon == nullptr)
	{
		return Ctx.Error(Node, TEXT("a tendon path item outside a tendon"));
	}

	ElementType Type;
	if (!gen::ElementTypeOfNode(Node, Type))
	{
		return Ctx.Error(Node, TEXT("unrecognised tendon path item"));
	}

	const mjsWrap* Made = nullptr;
	switch (Type)
	{
	case ElementType::SpatialSite:
		Made = mjs_wrapSite(Tendon, Utf8(static_cast<const UMjSpatialSite&>(Node).Site));
		break;
	case ElementType::SpatialGeom:
	{
		const UMjSpatialGeom& Element = static_cast<const UMjSpatialGeom&>(Node);
		// The empty string rather than a null pointer for an unauthored side
		// site: MuJoCo takes it by value into a std::string, so null is not
		// "there isn't one", it is an access violation.
		const FString Side = Element.Sidesite.Get(FString());
		Made = mjs_wrapGeom(Tendon, Utf8(Element.Geom), Utf8(Side));
		break;
	}
	case ElementType::Pulley:
		Made = mjs_wrapPulley(Tendon,
			static_cast<const UMjPulley&>(Node).Divisor.Get(0.0));
		break;
	case ElementType::FixedJoint:
	{
		const UMjFixedJoint& Element = static_cast<const UMjFixedJoint&>(Node);
		Made = mjs_wrapJoint(Tendon, Utf8(Element.Joint), Element.Coef.Get(0.0));
		break;
	}
	default:
		return Ctx.Error(Node, TEXT("unrecognised tendon path item"));
	}

	if (Made == nullptr)
	{
		return Ctx.Error(Node, TEXT("the tendon rejected this path item"));
	}
	return true;
}

// --- H4: material layers -------------------------------------------------- //
// A material's textures are one vector indexed by role, so both the `texture=`
// shorthand and the `<layer>` children are writes into slots of it rather than
// fields of their own.

bool ApplyMaterialLayers(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	ElementType Type;
	if (!gen::ElementTypeOfNode(Node, Type))
	{
		return false;
	}

	if (Type == ElementType::Material)
	{
		// Nothing, and that is the point: `<material texture=...>` is the
		// reader's shorthand for `<layer role="rgb">`, so it is folded into a
		// layer child when the document is read and the material itself carries
		// no texture slot to write. The layer branch below does the work.
		return true;
	}

	mjsMaterial* const Owner = static_cast<mjsMaterial*>(Ctx.Parent);
	const UMjMaterialLayer& Layer = static_cast<const UMjMaterialLayer&>(Node);
	if (Owner == nullptr)
	{
		return Ctx.Error(Node, TEXT("a texture layer outside a material"));
	}
	EMjTexRole Role;
	if (!ps::ue::FromMjcf(ps::ue::FMjStrPolicy::ToUtf8(FStringView(Layer.Role)), Role))
	{
		return Ctx.Error(Node,
			FString::Printf(TEXT("unknown texture role '%s'"), *Layer.Role));
	}
	if (!Layer.Texture.IsSet())
	{
		return Ctx.Error(Node, TEXT("a texture layer with no texture"));
	}
	mjs_setInStringVec(Owner->textures, sw::KeywordC(Role),
		Utf8(Layer.Texture.GetValue()));
	return true;
}

// --- H5: skin bones ------------------------------------------------------- //
// A bone is a row across five parallel vectors on the skin, so it is written by
// appending to each rather than by filling a struct of its own.

bool ApplySkinBone(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	mjsSkin* const Skin = static_cast<mjsSkin*>(Ctx.Parent);
	if (Skin == nullptr)
	{
		return Ctx.Error(Node, TEXT("a bone outside a skin"));
	}

	// Written once, for the whole run: the bind vectors are flat and the id and
	// weight vectors are append-only, so writing one bone at a time would need
	// state carried between calls. The first bone writes them all, and the rest
	// find the work already done.
	const TArray<FMjOrderedChild> Bones = Siblings(Ctx);
	const UMjSkinBone* const First = Bones.IsEmpty()
		? nullptr : Cast<UMjSkinBone>(Bones[0].Node);
	if (First != &Node)
	{
		return true;
	}

	TArray<float> Positions;
	TArray<float> Rotations;
	for (const FMjOrderedChild& Child : Bones)
	{
		const UMjSkinBone* const Bone = Cast<UMjSkinBone>(Child.Node);
		if (Bone == nullptr)
		{
			continue;
		}
		mjs_appendString(Skin->bodyname, Utf8(Bone->Body));

		const FMjPosition3 BindPos = Bone->Bindpos.Get(FMjPosition3());
		Positions.Add(static_cast<float>(BindPos.X));
		Positions.Add(static_cast<float>(BindPos.Y));
		Positions.Add(static_cast<float>(BindPos.Z));

		const FMjQuatRot BindQuat = Bone->Bindquat.Get(FMjQuatRot());
		Rotations.Add(static_cast<float>(BindQuat.W));
		Rotations.Add(static_cast<float>(BindQuat.X));
		Rotations.Add(static_cast<float>(BindQuat.Y));
		Rotations.Add(static_cast<float>(BindQuat.Z));

		TArray<int> Ids;
		if (Bone->Vertid.IsSet())
		{
			Ids.Reserve(Bone->Vertid.GetValue().Num());
			for (const double Value : Bone->Vertid.GetValue())
			{
				Ids.Add(static_cast<int>(Value));
			}
		}
		mjs_appendIntVec(Skin->vertid, Ids.GetData(), Ids.Num());

		TArray<float> Weights;
		if (Bone->Vertweight.IsSet())
		{
			Weights.Reserve(Bone->Vertweight.GetValue().Num());
			for (const double Value : Bone->Vertweight.GetValue())
			{
				Weights.Add(static_cast<float>(Value));
			}
		}
		mjs_appendFloatVec(Skin->vertweight, Weights.GetData(), Weights.Num());
	}

	mjs_setFloat(Skin->bindpos, Positions.GetData(), Positions.Num());
	mjs_setFloat(Skin->bindquat, Rotations.GetData(), Rotations.Num());
	return true;
}

// --- H6: tuple elements --------------------------------------------------- //

bool ApplyTupleElement(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	mjsTuple* const Tuple = static_cast<mjsTuple*>(Ctx.Parent);
	if (Tuple == nullptr)
	{
		return Ctx.Error(Node, TEXT("a tuple entry outside a tuple"));
	}
	// Written once for the whole run, for the same reason a skin's bones are:
	// the three vectors are parallel and only two of the three can be appended
	// to one entry at a time.
	const TArray<FMjOrderedChild> Entries = Siblings(Ctx);
	const UMjTupleElement* const First = Entries.IsEmpty()
		? nullptr : Cast<UMjTupleElement>(Entries[0].Node);
	if (First != &Node)
	{
		return true;
	}

	TArray<int> Kinds;
	TArray<double> Parameters;
	for (const FMjOrderedChild& Child : Entries)
	{
		const UMjTupleElement* const Entry = Cast<UMjTupleElement>(Child.Node);
		if (Entry == nullptr)
		{
			continue;
		}
		const int Kind = mju_str2Type(Utf8(Entry->Objtype));
		if (Kind == mjOBJ_UNKNOWN)
		{
			return Ctx.Error(*Entry,
				FString::Printf(TEXT("unknown object kind '%s'"), *Entry->Objtype));
		}
		Kinds.Add(Kind);
		Parameters.Add(Entry->Prm.Get(0.0));
		mjs_appendString(Tuple->objname, Utf8(Entry->Objname));
	}
	mjs_setInt(Tuple->objtype, Kinds.GetData(), Kinds.Num());
	mjs_setDouble(Tuple->objprm, Parameters.GetData(), Parameters.Num());
	return true;
}

// --- H7: plugins ---------------------------------------------------------- //

bool ApplyPlugin(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	ElementType Type;
	if (!gen::ElementTypeOfNode(Node, Type))
	{
		return false;
	}

	const auto Configure = [&](mjsPlugin& Plugin, const TOptional<FString>& Name,
		const TOptional<FString>& Instance)
	{
		Plugin.active = true;
		mjs_setString(Plugin.plugin_name, Utf8(Name.Get(FString())));
		mjs_setString(Plugin.name, Utf8(Instance.Get(FString())));
		if (!Instance.IsSet() || Instance.GetValue().IsEmpty())
		{
			if (mjsPlugin* const Made = mjs_addPlugin(Ctx.Spec))
			{
				Plugin.element = Made->element;
			}
		}
	};

	switch (Type)
	{
	case ElementType::PluginDef:
	{
		const UMjPluginDef& Element = static_cast<const UMjPluginDef&>(Node);
		if (Element.Plugin.IsSet() &&
			mjs_activatePlugin(Ctx.Spec, Utf8(Element.Plugin.GetValue())) != 0)
		{
			return Ctx.Error(Node, FString::Printf(
				TEXT("could not activate plugin '%s'"), *Element.Plugin.GetValue()));
		}
		return true;
	}

	case ElementType::PluginInstance:
	{
		mjsPlugin* const Made = mjs_addPlugin(Ctx.Spec);
		if (Made == nullptr)
		{
			return Ctx.Error(Node, TEXT("could not add a plugin instance"));
		}
		Made->active = true;
		Ctx.Struct = Made;
		Ctx.Element = Made->element;
		if (Node.MjName.IsSet())
		{
			mjs_setString(Made->name, Utf8(Node.MjName.GetValue()));
		}
		return true;
	}

	case ElementType::PluginRef:
	{
		const UMjPluginRef& Element = static_cast<const UMjPluginRef&>(Node);
		// A <plugin> child configures the element that encloses it, and which
		// member that is depends on the enclosing struct: only a handful carry
		// an mjsPlugin, so the parent's element type is what selects it.
		ElementType Owner;
		mjsPlugin* Slot = nullptr;
		if (Ctx.Parent != nullptr && Ctx.ParentNode != nullptr &&
			gen::ElementTypeOfNode(*Ctx.ParentNode, Owner))
		{
			switch (Owner)
			{
			case ElementType::Body:
				Slot = &static_cast<mjsBody*>(Ctx.Parent)->plugin;
				break;
			case ElementType::Geom:
				Slot = &static_cast<mjsGeom*>(Ctx.Parent)->plugin;
				break;
			case ElementType::Mesh:
				Slot = &static_cast<mjsMesh*>(Ctx.Parent)->plugin;
				break;
			default:
				break;
			}
		}
		if (Slot == nullptr)
		{
			return Ctx.Error(Node, TEXT("a <plugin> child on an element with no plugin slot"));
		}
		Configure(*Slot, Element.Plugin, Element.Instance);
		return true;
	}

	case ElementType::ActuatorPlugin:
	{
		mjsActuator* const Act = Actuator(Ctx);
		if (Act == nullptr)
		{
			return Ctx.Error(Node, TEXT("a plugin actuator that was never created"));
		}
		const UMjActuatorPlugin& Element = static_cast<const UMjActuatorPlugin&>(Node);
		Configure(Act->plugin, Element.Plugin, Element.Instance);
		return true;
	}

	case ElementType::SensorPlugin:
	{
		mjsSensor* const Sensor = static_cast<mjsSensor*>(Ctx.Struct);
		if (Sensor == nullptr)
		{
			return Ctx.Error(Node, TEXT("a plugin sensor that was never created"));
		}
		const UMjSensorPlugin& Element = static_cast<const UMjSensorPlugin&>(Node);
		Sensor->type = mjSENS_PLUGIN;
		Configure(Sensor->plugin, Element.Plugin, Element.Instance);
		const auto Kind = [](const TOptional<FString>& Text)
		{
			return Text.IsSet() ? static_cast<mjtObj>(mju_str2Type(Utf8(Text.GetValue())))
								: mjOBJ_UNKNOWN;
		};
		Sensor->objtype = Kind(Element.Objtype);
		Sensor->reftype = Kind(Element.Reftype);
		if (Element.Objname.IsSet())
		{
			mjs_setString(Sensor->objname, Utf8(Element.Objname.GetValue()));
		}
		if (Element.Refname.IsSet())
		{
			mjs_setString(Sensor->refname, Utf8(Element.Refname.GetValue()));
		}
		if ((Sensor->objtype != mjOBJ_UNKNOWN) != Element.Objname.IsSet())
		{
			return Ctx.Error(Node, TEXT("objtype and objname must be given together"));
		}
		if ((Sensor->reftype != mjOBJ_UNKNOWN) != Element.Refname.IsSet())
		{
			return Ctx.Error(Node, TEXT("reftype and refname must be given together"));
		}
		return true;
	}

	case ElementType::Config:
		// Plugin configuration is a key/value pair the engine reads through its
		// own attribute map, which mjs_setPluginAttributes owns.
		return true;

	case ElementType::Extension:
		return true;

	default:
		return Ctx.Error(Node, TEXT("unrecognised plugin element"));
	}
}

// --- H8: option bitmask folds --------------------------------------------- //

bool ApplyOptionFlags(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	ElementType Type;
	if (!gen::ElementTypeOfNode(Node, Type))
	{
		return false;
	}

	if (Type == ElementType::Option)
	{
		const UMjOption& Element = static_cast<const UMjOption&>(Node);
		if (!Element.Actuatorgroupdisable.IsSet())
		{
			return true;
		}
		// One bit per group id: mjOption carries a mask, not a list, so the
		// fold IS the write.
		for (const int32 Group : Element.Actuatorgroupdisable.GetValue())
		{
			if (Group < 0)
			{
				return Ctx.Error(Node, TEXT("a disabled actuator group must not be negative"));
			}
			if (Group > 30)
			{
				return Ctx.Error(Node, TEXT("a disabled actuator group must not exceed 30"));
			}
			Ctx.Spec->option.disableactuator |= 1 << Group;
		}
		return true;
	}

	const UMjFlag& Flag = static_cast<const UMjFlag&>(Node);
	mjOption& Option = Ctx.Spec->option;

	// Two families with opposite polarity: a disable flag is stored as the
	// ABSENCE of its bit when enabled, and an enable flag as its presence.
	const auto Disable = [&](const TOptional<EMjEnable>& Value, int Mask)
	{
		if (!Value.IsSet()) { return; }
		Option.disableflags &= ~Mask;
		Option.disableflags |= Value.GetValue() == EMjEnable::enable ? 0 : Mask;
	};
	const auto Enable = [&](const TOptional<EMjEnable>& Value, int Mask)
	{
		if (!Value.IsSet()) { return; }
		Option.enableflags &= ~Mask;
		Option.enableflags |= Value.GetValue() == EMjEnable::enable ? Mask : 0;
	};

	Disable(Flag.Constraint, mjDSBL_CONSTRAINT);
	Disable(Flag.Equality, mjDSBL_EQUALITY);
	Disable(Flag.Frictionloss, mjDSBL_FRICTIONLOSS);
	Disable(Flag.Limit, mjDSBL_LIMIT);
	Disable(Flag.Contact, mjDSBL_CONTACT);
	Disable(Flag.Spring, mjDSBL_SPRING);
	Disable(Flag.Damper, mjDSBL_DAMPER);
	Disable(Flag.Gravity, mjDSBL_GRAVITY);
	Disable(Flag.Clampctrl, mjDSBL_CLAMPCTRL);
	Disable(Flag.Warmstart, mjDSBL_WARMSTART);
	Disable(Flag.Filterparent, mjDSBL_FILTERPARENT);
	Disable(Flag.Actuation, mjDSBL_ACTUATION);
	Disable(Flag.Refsafe, mjDSBL_REFSAFE);
	Disable(Flag.Sensor, mjDSBL_SENSOR);
	Disable(Flag.Midphase, mjDSBL_MIDPHASE);
	Disable(Flag.Eulerdamp, mjDSBL_EULERDAMP);
	Disable(Flag.Autoreset, mjDSBL_AUTORESET);
	Disable(Flag.Nativeccd, mjDSBL_NATIVECCD);
	Disable(Flag.Island, mjDSBL_ISLAND);
	Disable(Flag.Multiccd, mjDSBL_MULTICCD);

	Enable(Flag.Override, mjENBL_OVERRIDE);
	Enable(Flag.Energy, mjENBL_ENERGY);
	Enable(Flag.Fwdinv, mjENBL_FWDINV);
	Enable(Flag.Invdiscrete, mjENBL_INVDISCRETE);
	Enable(Flag.Sleep, mjENBL_SLEEP);
	Enable(Flag.Diagexact, mjENBL_DIAGEXACT);
	return true;
}

// --- H9: size memory ------------------------------------------------------ //

bool ApplySizeMemory(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	const UMjSize& Element = static_cast<const UMjSize&>(Node);
	if (!Element.Memory.IsSet())
	{
		return true;
	}
	FString Text = Element.Memory.GetValue().TrimStartAndEnd();
	if (Text == TEXT("-1"))
	{
		return true;  // the engine's own "unset" spelling
	}

	// An unsigned count with an optional binary-multiple suffix, which is the
	// only place in MJCF a number is written that way.
	static const TCHAR* const Suffixes = TEXT("KMGTPE");
	int32 Shift = 0;
	if (!Text.IsEmpty())
	{
		int32 Index = INDEX_NONE;
		if (FString(Suffixes).FindChar(Text[Text.Len() - 1], Index))
		{
			Shift = 10 * (Index + 1);
			Text.LeftChopInline(1);
		}
	}
	if (Text.IsEmpty() || !Text.IsNumeric() || Text.StartsWith(TEXT("-")))
	{
		return Ctx.Error(Node, FString::Printf(
			TEXT("'%s' is not an unsigned byte count with an optional {K,M,G,T,P,E} suffix"),
			*Element.Memory.GetValue()));
	}
	const uint64 Base = FCString::Strtoui64(*Text, nullptr, 10);
	if (Shift >= 64 || Base > (TNumericLimits<uint64>::Max() >> Shift))
	{
		return Ctx.Error(Node, TEXT("the memory size given is too big"));
	}
	Ctx.Spec->memory = static_cast<mjtSize>(Base << Shift);
	return true;
}

// --- Input folds ---------------------------------------------------------- //
// Nothing to do, and that is the point: these attributes are the reader's
// alternative spellings of a canonical field, so the fold already happened when
// the document was read and they carry no storage of their own to write.

bool ApplyInputFold(FMjSpecWriteContext&, const UMjNodeComponent&, mjsElement*)
{
	return true;
}

// --- Equality subtype folds ----------------------------------------------- //
// mjsEquality carries name1, name2, objtype and one data array, so each subtype
// elects the object kind and packs its operands into them.

bool ApplyEqualityFold(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	mjsEquality* const Equality = static_cast<mjsEquality*>(Ctx.Struct);
	if (Equality == nullptr)
	{
		return Ctx.Error(Node, TEXT("an equality fold on something that is not an equality"));
	}

	ElementType Type;
	if (!gen::ElementTypeOfNode(Node, Type))
	{
		return false;
	}

	FString Name1;
	FString Name2;
	const auto Anchor = [&](const TOptional<FMjPosition3>& Value, int Offset)
	{
		if (!Value.IsSet()) { return false; }
		Equality->data[Offset + 0] = Value.GetValue().X;
		Equality->data[Offset + 1] = Value.GetValue().Y;
		Equality->data[Offset + 2] = Value.GetValue().Z;
		return true;
	};

	switch (Type)
	{
	case ElementType::Connect:
	{
		const UMjConnect& Element = static_cast<const UMjConnect&>(Node);
		Equality->type = mjEQ_CONNECT;
		const bool bHasAnchor = Anchor(Element.Anchor, 0);
		// Body semantics need an anchor; site semantics name two sites and
		// carry the offset in the sites themselves.
		if (Element.Body1.IsSet() && bHasAnchor)
		{
			Name1 = Element.Body1.GetValue();
			Name2 = Element.Body2.Get(FString());
			Equality->objtype = mjOBJ_BODY;
		}
		else
		{
			Name1 = Element.Site1.Get(FString());
			Name2 = Element.Site2.Get(FString());
			Equality->objtype = mjOBJ_SITE;
		}
		break;
	}

	case ElementType::Weld:
	{
		const UMjWeld& Element = static_cast<const UMjWeld&>(Node);
		Equality->type = mjEQ_WELD;
		const bool bHasAnchor = Anchor(Element.Anchor, 0);
		if (Element.Relpose.IsSet())
		{
			const TArray<double>& Pose = Element.Relpose.GetValue();
			const int32 Count = FMath::Min<int32>(Pose.Num(), 7);
			for (int32 I = 0; I < Count; ++I)
			{
				Equality->data[3 + I] = Pose[I];
			}
		}
		if (Element.Body1.IsSet())
		{
			Name1 = Element.Body1.GetValue();
			Name2 = Element.Body2.Get(FString());
			Equality->objtype = mjOBJ_BODY;
			if (!bHasAnchor)
			{
				Equality->data[0] = Equality->data[1] = Equality->data[2] = 0.0;
			}
		}
		else
		{
			Name1 = Element.Site1.Get(FString());
			Name2 = Element.Site2.Get(FString());
			Equality->objtype = mjOBJ_SITE;
		}
		if (Element.Torquescale.IsSet())
		{
			Equality->data[10] = Element.Torquescale.GetValue();
		}
		break;
	}

	case ElementType::EqualityJoint:
	{
		const UMjEqualityJoint& Element = static_cast<const UMjEqualityJoint&>(Node);
		Equality->type = mjEQ_JOINT;
		Name1 = Element.Joint1;
		Name2 = Element.Joint2.Get(FString());
		CopySome(Element.Polycoef, Equality->data, 5);
		break;
	}

	case ElementType::EqualityTendon:
	{
		const UMjEqualityTendon& Element = static_cast<const UMjEqualityTendon&>(Node);
		Equality->type = mjEQ_TENDON;
		Name1 = Element.Tendon1;
		Name2 = Element.Tendon2.Get(FString());
		CopySome(Element.Polycoef, Equality->data, 5);
		break;
	}

	case ElementType::EqualityFlex:
		Equality->type = mjEQ_FLEX;
		Name1 = static_cast<const UMjEqualityFlex&>(Node).Flex;
		break;

	case ElementType::Flexvert:
		Equality->type = mjEQ_FLEXVERT;
		Name1 = static_cast<const UMjFlexvert&>(Node).Flex;
		break;

	case ElementType::Flexstrain:
	{
		const UMjFlexstrain& Element = static_cast<const UMjFlexstrain&>(Node);
		Equality->type = mjEQ_FLEXSTRAIN;
		Name1 = Element.Flex;
		if (Element.Cell.IsSet())
		{
			Equality->data[0] = Element.Cell.GetValue().X;
			Equality->data[1] = Element.Cell.GetValue().Y;
			Equality->data[2] = Element.Cell.GetValue().Z;
		}
		break;
	}

	default:
		return Ctx.Error(Node, TEXT("unrecognised equality constraint"));
	}

	mjs_setString(Equality->name1, Utf8(Name1));
	if (!Name2.IsEmpty())
	{
		mjs_setString(Equality->name2, Utf8(Name2));
	}
	return true;
}

// --- Irregular sensor folds ----------------------------------------------- //
// The same shape one level over: mjsSensor stores its operands in one name pair
// with a kind each, and its keyword sets in intprm.

bool ApplySensorFold(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	mjsSensor* const Sensor = static_cast<mjsSensor*>(Ctx.Struct);
	if (Sensor == nullptr)
	{
		return Ctx.Error(Node, TEXT("a sensor fold on something that is not a sensor"));
	}

	ElementType Type;
	if (!gen::ElementTypeOfNode(Node, Type))
	{
		return false;
	}

	FString ObjName;
	FString RefName;

	/** The first operand spelling that is present decides the object kind. */
	const auto Elect = [](std::initializer_list<FOperand> Options,
		FString& OutName, mjtObj& OutType)
	{
		for (const FOperand& Option : Options)
		{
			if (Option.Value->IsSet())
			{
				OutName = Option.Value->GetValue();
				OutType = Option.Kind;
				return;
			}
		}
		OutType = mjOBJ_UNKNOWN;
	};

	switch (Type)
	{
	case ElementType::Rangefinder:
	{
		const UMjRangefinder& Element = static_cast<const UMjRangefinder&>(Node);
		Sensor->type = mjSENS_RANGEFINDER;
		Elect({ { &Element.Site, mjOBJ_SITE }, { &Element.Camera, mjOBJ_CAMERA } },
			ObjName, Sensor->objtype);
		if (Element.Data.IsSet() && !Ascending(Element.Data.GetValue()))
		{
			return Ctx.Error(Node, TEXT("rangefinder data keywords must be in schema order"));
		}
		Sensor->intprm[0] = DataSpec(Element.Data.Get(TArray<EMjRayData>()),
			1 << mjRAYDATA_DIST);
		break;
	}

	case ElementType::Distance:
	case ElementType::Normal:
	case ElementType::Fromto:
	{
		// The three share a shape, so they share the fold and differ only in
		// which sensor kind they name.
		const auto Fold = [&](const auto& Element)
		{
			Elect({ { &Element.Body1, mjOBJ_BODY }, { &Element.Geom1, mjOBJ_GEOM } },
				ObjName, Sensor->objtype);
			Elect({ { &Element.Body2, mjOBJ_BODY }, { &Element.Geom2, mjOBJ_GEOM } },
				RefName, Sensor->reftype);
		};
		if (Type == ElementType::Distance)
		{
			Fold(static_cast<const UMjDistance&>(Node));
			Sensor->type = mjSENS_GEOMDIST;
		}
		else if (Type == ElementType::Normal)
		{
			Fold(static_cast<const UMjNormal&>(Node));
			Sensor->type = mjSENS_GEOMNORMAL;
		}
		else
		{
			Fold(static_cast<const UMjFromto&>(Node));
			Sensor->type = mjSENS_GEOMFROMTO;
		}
		break;
	}

	case ElementType::SensorContact:
	{
		const UMjSensorContact& Element = static_cast<const UMjSensorContact&>(Node);
		Sensor->type = mjSENS_CONTACT;
		Elect({ { &Element.Site, mjOBJ_SITE }, { &Element.Body1, mjOBJ_BODY },
				{ &Element.Subtree1, mjOBJ_XBODY }, { &Element.Geom1, mjOBJ_GEOM } },
			ObjName, Sensor->objtype);
		Elect({ { &Element.Body2, mjOBJ_BODY }, { &Element.Subtree2, mjOBJ_XBODY },
				{ &Element.Geom2, mjOBJ_GEOM } },
			RefName, Sensor->reftype);
		if (Element.Data.IsSet() && !Ascending(Element.Data.GetValue()))
		{
			return Ctx.Error(Node, TEXT("contact data keywords must be in schema order"));
		}
		Sensor->intprm[0] = DataSpec(Element.Data.Get(TArray<EMjContactData>()),
			1 << mjCONDATA_FOUND);
		Sensor->intprm[1] = Element.Reduce.IsSet()
			? sw::KeywordC(Element.Reduce.GetValue()) : 0;
		Sensor->intprm[2] = Element.Num.Get(1);
		if (Sensor->intprm[2] <= 0)
		{
			return Ctx.Error(Node, TEXT("a contact sensor's num must be positive"));
		}
		break;
	}

	case ElementType::Tactile:
	{
		const UMjTactile& Element = static_cast<const UMjTactile&>(Node);
		Sensor->type = mjSENS_TACTILE;
		// The mesh is the sensorized object and the geom is the reference,
		// which is the opposite of how the tag reads.
		Sensor->objtype = mjOBJ_MESH;
		ObjName = Element.Mesh;
		Sensor->reftype = mjOBJ_GEOM;
		RefName = Element.Geom;
		if (Element.User.IsSet())
		{
			mjs_setDouble(Sensor->userdata,
				const_cast<double*>(Element.User.GetValue().GetData()),
				Element.User.GetValue().Num());
		}
		break;
	}

	case ElementType::SensorUser:
	{
		const UMjSensorUser& Element = static_cast<const UMjSensorUser&>(Node);
		Sensor->type = mjSENS_USER;
		if (Element.Objtype.IsSet())
		{
			Sensor->objtype = static_cast<mjtObj>(
				mju_str2Type(Utf8(Element.Objtype.GetValue())));
			if (Sensor->objtype == mjOBJ_UNKNOWN)
			{
				return Ctx.Error(Node, FString::Printf(
					TEXT("unknown object kind '%s'"), *Element.Objtype.GetValue()));
			}
		}
		ObjName = Element.Objname.Get(FString());
		if (Element.User.IsSet())
		{
			mjs_setDouble(Sensor->userdata,
				const_cast<double*>(Element.User.GetValue().GetData()),
				Element.User.GetValue().Num());
		}
		break;
	}

	default:
		return Ctx.Error(Node, TEXT("unrecognised sensor"));
	}

	if (!ObjName.IsEmpty())
	{
		mjs_setString(Sensor->objname, Utf8(ObjName));
	}
	if (!RefName.IsEmpty())
	{
		mjs_setString(Sensor->refname, Utf8(RefName));
	}
	return true;
}

// --- Compiler placement --------------------------------------------------- //

bool ApplyCompilerPlacement(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	const UMjCompiler& Element = static_cast<const UMjCompiler&>(Node);

	// strippath is on the spec, not on the compiler options it is authored with.
	if (Element.Strippath.IsSet())
	{
		Ctx.Spec->strippath = Element.Strippath.GetValue() ? 1 : 0;
	}

	// The engine reads `coordinate` only to reject the removed global form, so
	// the check is the whole of the write.
	if (Element.Coordinate.IsSet() && Element.Coordinate.GetValue() != EMjCoordinate::local)
	{
		return Ctx.Error(Node,
			TEXT("global coordinates are no longer supported; convert the model in "
				 "MuJoCo 2.3.3 or older"));
	}

	// assetdir has no field: it is the fallback both asset directories take,
	// and an explicit meshdir or texturedir overrides it.
	if (Element.Assetdir.IsSet())
	{
		const char* const Directory = Utf8(Element.Assetdir.GetValue());
		if (!Element.Meshdir.IsSet())
		{
			mjs_setString(Ctx.Spec->compiler.meshdir, Directory);
		}
		if (!Element.Texturedir.IsSet())
		{
			mjs_setString(Ctx.Spec->compiler.texturedir, Directory);
		}
	}
	return true;
}

// --- Asset builtins ------------------------------------------------------- //

bool ApplyAssetBuiltin(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	ElementType Type;
	if (!gen::ElementTypeOfNode(Node, Type))
	{
		return false;
	}

	switch (Type)
	{
	case ElementType::Mesh:
	{
		const UMjMeshBase& Element = static_cast<const UMjMeshBase&>(Node);
		if (!Element.Builtin.IsSet())
		{
			return true;
		}
		// Generated geometry is not stored on the mesh: mjs_makeMesh consumes
		// the keyword and its parameters and produces vertices.
		mjsMesh* const Mesh = static_cast<mjsMesh*>(Ctx.Struct);
		if (Mesh == nullptr)
		{
			return Ctx.Error(Node, TEXT("a builtin on something that is not a mesh"));
		}
		TArray<double> Parameters = Element.Params.Get(TArray<double>());
		if (mjs_makeMesh(Mesh, static_cast<mjtMeshBuiltin>(sw::KeywordC(Element.Builtin.GetValue())),
			Parameters.GetData(), Parameters.Num()) != 0)
		{
			return Ctx.Error(Node, FString(UTF8_TO_TCHAR(mjs_getError(Ctx.Spec))));
		}
		return true;
	}

	case ElementType::Hfield:
	{
		const UMjHfield& Element = static_cast<const UMjHfield&>(Node);
		mjsHField* const Field = static_cast<mjsHField*>(Ctx.Struct);
		if (Field == nullptr)
		{
			return Ctx.Error(Node, TEXT("elevation on something that is not a heightfield"));
		}
		// A file supplies the elevation, and authored rows beside one are the
		// reader's no-op rather than a second source
		// (`xml_native_reader.cc:2263`).
		const bool bFromFile = Element.File.IsSet() && !Element.File.GetValue().IsEmpty();
		const int32 Rows = Element.Nrow.Get(0);
		const int32 Columns = Element.Ncol.Get(0);
		if (!Element.Elevation.IsSet() || bFromFile || Rows <= 0 || Columns <= 0)
		{
			return true;
		}

		const TArray<double>& Values = Element.Elevation.GetValue();
		if (Values.Num() != Rows * Columns)
		{
			return Ctx.Error(Node, TEXT("elevation data length must match nrow*ncol"));
		}

		// Rows go in bottom-to-top, so that the authored string reads
		// top-to-bottom. The compiler copies the vector across verbatim
		// (`user_objects.cc:4773`), so this row order is the stored row order
		// and reversing it here is the whole of the convention
		// (`xml_native_reader.cc:2277`).
		TArray<float> Elevation;
		Elevation.SetNumUninitialized(Values.Num());
		for (int32 Row = 0; Row < Rows; ++Row)
		{
			const int32 Flipped = Rows - 1 - Row;
			for (int32 Column = 0; Column < Columns; ++Column)
			{
				Elevation[Flipped * Columns + Column] = static_cast<float>(Values[Row * Columns + Column]);
			}
		}
		mjs_setFloat(Field->userdata, Elevation.GetData(), Elevation.Num());
		return true;
	}

	case ElementType::Texture:
	{
		const UMjTextureBase& Element = static_cast<const UMjTextureBase&>(Node);
		mjsTexture* const Texture = static_cast<mjsTexture*>(Ctx.Struct);
		if (Texture == nullptr)
		{
			return Ctx.Error(Node, TEXT("cube faces on something that is not a texture"));
		}
		// The six faces are positional slots of one vector, in the engine's own
		// right-left-up-down-front-back order.
		const TOptional<FString>* const Faces[] = {
			&Element.Fileright, &Element.Fileleft, &Element.Fileup,
			&Element.Filedown, &Element.Filefront, &Element.Fileback };
		for (int32 Face = 0; Face < UE_ARRAY_COUNT(Faces); ++Face)
		{
			if (Faces[Face]->IsSet())
			{
				mjs_setInStringVec(Texture->cubefiles, Face,
					Utf8(Faces[Face]->GetValue()));
			}
		}
		return true;
	}

	default:
		return Ctx.Error(Node, TEXT("unrecognised asset builtin"));
	}
}

// --- Flex dof layout ------------------------------------------------------ //

bool ApplyFlexLayout(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	const UMjFlex& Element = static_cast<const UMjFlex&>(Node);
	if (!Element.Dof.IsSet())
	{
		return true;
	}
	mjsFlex* const Flex = static_cast<mjsFlex*>(Ctx.Struct);
	if (Flex == nullptr)
	{
		return Ctx.Error(Node, TEXT("a flex dof layout on something that is not a flex"));
	}
	// The keyword lowers to an interpolation order rather than being stored,
	// and it lowers by keyword rather than by constant: mjFCOMPDOF_* is a
	// parse-time enum MuJoCo does not ship a header for. Only the two reduced
	// layouts raise the order; full, radial and 2d all leave it at zero.
	switch (Element.Dof.GetValue())
	{
	case EMjFlexDof::quadratic: Flex->order = 2; break;
	case EMjFlexDof::trilinear: Flex->order = 1; break;
	default: Flex->order = 0; break;
	}
	return true;
}

// --- Numeric data --------------------------------------------------------- //

bool ApplyNumericData(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	const UMjNumeric& Element = static_cast<const UMjNumeric&>(Node);
	if (!Element.Data.IsSet())
	{
		return true;
	}
	// `<numeric data="1 2 3">` spells a double vector as one string, so the
	// parse is the write.
	mjsNumeric* const Numeric = static_cast<mjsNumeric*>(Ctx.Struct);
	if (Numeric == nullptr)
	{
		return Ctx.Error(Node, TEXT("numeric data on something that is not a numeric"));
	}
	TArray<FString> Tokens;
	Element.Data.GetValue().ParseIntoArrayWS(Tokens);
	TArray<double> Values;
	Values.Reserve(Tokens.Num());
	for (const FString& Token : Tokens)
	{
		if (!Token.IsNumeric())
		{
			return Ctx.Error(Node, FString::Printf(
				TEXT("'%s' is not a number in this numeric's data"), *Token));
		}
		Values.Add(FCString::Atod(*Token));
	}
	mjs_setDouble(Numeric->data, Values.GetData(), Values.Num());
	return true;
}

// --- The macro bridge ----------------------------------------------------- //
// `<composite>`, `<flexcomp>` and `<replicate>` are expanded by MuJoCo's XML
// READER, not by its compiler, so there is no mjs_* call that performs one. The
// bridge is therefore the reader itself: serialize the macro subtree into a
// small document, parse it, and attach the expansion where the macro was
// authored.
//
// What the document has to carry beyond the macro is what the expansion depends
// on. The participant's `<compiler>` governs its subtree's angle and coordinate
// semantics regardless of the spec it is attached into, and its `<default>` tree
// has to be present or a `childclass` reference fails to parse. Assets are the
// one thing it must NOT carry wholesale: the expansion is attached with an empty
// prefix and the target already holds every one of them, and mjs_attach rejects
// a repeated asset name. So the wrapper carries the asset section minus the
// names the target holds, which is a set difference rather than a walk over what
// the macro references -- anything omitted is omitted BECAUSE the target has it,
// so no omission can break a reference.
//
// The macro sits under a `<frame>` carrying the class the macro resolved
// through, which is the class its enclosing body imposed. A frame rather than a
// body because a frame is flattened at compile and a body is not: a body carrier
// would put an extra link in the chain and shift every name after it.

/**
 * The three elements MuJoCo's reader expands.
 *
 * Their inner components route to this handler too, and reaching it means one
 * was authored outside the macro that owns it: the macro carries its own
 * subtree, so nothing inside one is ever bridged on its own.
 */
bool IsMacroRoot(ElementType Type)
{
	return Type == ElementType::Composite || Type == ElementType::Flexcomp ||
		Type == ElementType::Replicate;
}

/** The object kind an asset element compiles into, or mjOBJ_UNKNOWN. */
mjtObj AssetKind(ElementType Type)
{
	switch (Type)
	{
	case ElementType::Mesh: return mjOBJ_MESH;
	case ElementType::Hfield: return mjOBJ_HFIELD;
	case ElementType::Skin: return mjOBJ_SKIN;
	case ElementType::Texture: return mjOBJ_TEXTURE;
	case ElementType::Material: return mjOBJ_MATERIAL;
	case ElementType::ModelAsset: return mjOBJ_MODEL;
	default: return mjOBJ_UNKNOWN;
	}
}

/**
 * True when the wrapper has to carry this asset element.
 *
 * False means the target spec already holds it, which is the only reason an
 * asset is ever left out. An unnamed element is one the walk created before the
 * body sections and whose compiled name MuJoCo derives from its file, so a
 * second copy in the wrapper would collide at compile rather than at attach --
 * the same "already there" for a different reason.
 */
bool WrapperNeedsAsset(const FMjSpecWriteContext& Ctx, const UMjNodeComponent& Asset)
{
	ElementType Type;
	if (!gen::ElementTypeOfNode(Asset, Type))
	{
		return false;
	}
	const mjtObj Kind = AssetKind(Type);
	if (Kind == mjOBJ_UNKNOWN)
	{
		return false;
	}
	if (!Asset.MjName.IsSet() || Asset.MjName.GetValue().IsEmpty())
	{
		return false;
	}
	const FTCHARToUTF8 Name(*Asset.MjName.GetValue());
	return mjs_findElement(Ctx.Spec, Kind, Name.Get()) == nullptr;
}

/** Serialize one node's subtree, reporting rather than returning a partial. */
bool AppendElement(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Macro,
	const UMjNodeComponent& Node, FString& Out)
{
	TArray<FMjSpecDiagnostic> Errors;
	const FString Text = Ctx.Source->WriteMjcfElement(Node, &Errors);
	if (Text.IsEmpty())
	{
		TArray<FString> Lines;
		for (const FMjSpecDiagnostic& Diagnostic : Errors)
		{
			Lines.Add(Diagnostic.ToString());
		}
		return Ctx.Error(Macro, FString::Printf(
			TEXT("could not serialize '%s' into this macro's wrapper document: %s"),
			*Node.GetName(), Lines.IsEmpty() ? TEXT("no output") : *FString::Join(Lines, TEXT("; "))));
	}
	Out += Text;
	return true;
}

/** The participant's compiler blocks, class tree and the assets not yet shared. */
bool AppendContext(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Macro, FString& Out)
{
	UMjNodeComponent* const Root = Ctx.Source->GetRoot();
	if (Root == nullptr)
	{
		return Ctx.Error(Macro, TEXT("this spec has no root component"));
	}

	FString Compilers;
	FString Defaults;
	FString Assets;
	for (const FMjOrderedChild& Section : MjOrderedChildrenOf(*Ctx.Source, *Root))
	{
		ElementType Type;
		if (Section.Node == nullptr || !gen::ElementTypeOfNode(*Section.Node, Type))
		{
			continue;
		}
		if (Type == ElementType::Compiler && !AppendElement(Ctx, Macro, *Section.Node, Compilers))
		{
			return false;
		}
		if (Type == ElementType::Default && !AppendElement(Ctx, Macro, *Section.Node, Defaults))
		{
			return false;
		}
		if (Type != ElementType::Asset)
		{
			continue;
		}
		for (const FMjOrderedChild& Asset : MjOrderedChildrenOf(*Ctx.Source, *Section.Node))
		{
			if (Asset.Node != nullptr && WrapperNeedsAsset(Ctx, *Asset.Node) &&
				!AppendElement(Ctx, Macro, *Asset.Node, Assets))
			{
				return false;
			}
		}
	}

	Out += Compilers;
	Out += Defaults;
	if (!Assets.IsEmpty())
	{
		Out += TEXT("<asset>\n") + Assets + TEXT("</asset>\n");
	}
	return true;
}

/**
 * Point the expansion's body references at the body the macro was authored in.
 *
 * A `<flexcomp>` pins its vertices to the enclosing body BY NAME
 * (`user_flexcomp.cc:517`). The wrapper has no such body -- the macro sits
 * directly in its world body -- so an expansion parsed on its own names
 * `world`, and after the attach that is the target spec's world rather than the
 * body the user authored the macro in. The pins would land on the wrong body,
 * and one attach later, under a participant prefix, the name resolves to
 * nothing and MuJoCo drops the whole flex without a word
 * (`user_model.cc:263-284` skips an element whose references do not resolve).
 *
 * So the substitution happens here, while the expansion is still its own spec
 * and its `world` still means "wherever this macro was authored".
 */
void PointExpansionAtOwner(mjSpec& Expansion, const mjsBody& Owner)
{
	const mjString* const OwnerName = mjs_getName(const_cast<mjsBody&>(Owner).element);
	const char* const Name = OwnerName != nullptr ? mjs_getString(OwnerName) : nullptr;
	if (Name == nullptr || Name[0] == '\0' || std::string(Name) == "world")
	{
		// The macro was authored in the spec's own world body, which is what
		// the wrapper already reproduces.
		return;
	}

	const auto Repoint = [Name](mjStringVec* Names) {
		if (Names == nullptr)
		{
			return;
		}
		for (int Index = 0; Index < static_cast<int>(Names->size()); ++Index)
		{
			if ((*Names)[Index] == "world")
			{
				mjs_setInStringVec(Names, Index, Name);
			}
		}
	};

	for (mjsElement* Element = mjs_firstElement(&Expansion, mjOBJ_FLEX); Element != nullptr;
		 Element = mjs_nextElement(&Expansion, Element))
	{
		if (mjsFlex* const Flex = mjs_asFlex(Element))
		{
			Repoint(Flex->vertbody);
			Repoint(Flex->nodebody);
		}
	}
}

bool ApplyMacroBridge(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	if (Ctx.Source == nullptr || Ctx.Spec == nullptr || Ctx.Body == nullptr)
	{
		return Ctx.Error(Node, TEXT("this macro element has nothing to be expanded onto"));
	}
	if (Ctx.Partial != nullptr)
	{
		return Ctx.Error(Node, TEXT("a macro element cannot be a default class partial"));
	}
	ElementType Type;
	if (!gen::ElementTypeOfNode(Node, Type) || !IsMacroRoot(Type))
	{
		return Ctx.Error(Node,
			TEXT("this element belongs inside a macro element, which carries it already"));
	}

	FString Wrapper = TEXT("<mujoco>\n");
	if (!AppendContext(Ctx, Node, Wrapper))
	{
		return false;
	}

	FString Macro;
	if (!AppendElement(Ctx, Node, Node, Macro))
	{
		return false;
	}
	Wrapper += FString::Printf(TEXT("<worldbody>\n<frame childclass=\"%s\">\n"), *Ctx.ClassName);
	Wrapper += Macro;
	Wrapper += TEXT("</frame>\n</worldbody>\n</mujoco>\n");

	char Error[1024] = {0};
	const FTCHARToUTF8 Utf8Wrapper(*Wrapper);
	mjSpec* const Expansion =
		mj_parseXMLString(Utf8Wrapper.Get(), nullptr, Error, sizeof(Error));
	if (Expansion == nullptr)
	{
		return Ctx.Error(Node, FString::Printf(TEXT("this macro did not expand: %s"),
			UTF8_TO_TCHAR(Error)));
	}

	PointExpansionAtOwner(*Expansion, *Ctx.Body);

	// The enclosing frame is the parent, so a macro authored inside a `<frame>`
	// expands under that frame's transform rather than under the body's.
	mjsFrame* const At = mjs_addFrame(Ctx.Body, Ctx.Frame);
	const bool bAttached = At != nullptr && Expansion->element != nullptr &&
		mjs_attach(At->element, Expansion->element, "", "") != nullptr;
	const FString AttachError = bAttached ? FString() : FString(UTF8_TO_TCHAR(mjs_getError(Ctx.Spec)));
	mj_deleteSpec(Expansion);

	if (!bAttached)
	{
		// A rejected attach has already inserted its element and moved the
		// counts, so the target spec is finished with rather than merely short
		// of one macro.
		return Ctx.Abort(Node, FString::Printf(
			TEXT("this macro's expansion could not be attached and the model cannot be built: %s"),
			*AttachError));
	}

	// The subtree went into the wrapper whole; the walk must not create it again
	// beside the expansion it produced.
	Ctx.bChildrenConsumed = true;
	return true;
}

// --- Nested models -------------------------------------------------------- //
// `<model>` parses a file into a child spec and `<attach>` splices one in, both
// on the reader's own terms (`src/xml/xml_native_reader.cc:2296` and `:2598`).
// Neither produces an element of the current spec: a child spec is held beside
// the walk, and an attach leaves behind only whatever MuJoCo copied across.

/**
 * The `<model>` asset's file, parsed and registered under its model name.
 *
 * The file resolves against the element's own source directory and through no
 * asset directory, which is the rule the reader applies to a nested model and
 * only to a nested model.
 */
bool ApplyModelAsset(FMjSpecWriteContext& Ctx, const UMjModelAsset& Asset)
{
	if (Ctx.Models == nullptr)
	{
		return Ctx.Error(Asset, TEXT("this build has nowhere to register a model asset"));
	}
	const FString File = Asset.File.Get(FString());
	if (File.IsEmpty())
	{
		return Ctx.Error(Asset, TEXT("this model asset names no file"));
	}
	const FString Path = MjResolveAssetPath(Asset, FString(), File);

	char Error[1024] = {0};
	mjSpec* const Child =
		mj_parse(Utf8(Path), Utf8(Asset.ContentType.Get(FString())), nullptr, Error, sizeof(Error));
	if (Child == nullptr)
	{
		return Ctx.Error(Asset, FString::Printf(TEXT("could not parse the model file '%s': %s"), *Path,
			UTF8_TO_TCHAR(Error)));
	}

	// An authored name renames the child, and the renamed child is what an
	// `<attach model=>` finds: the lookup is by model name either way.
	if (Asset.MjName.IsSet() && !Asset.MjName.GetValue().IsEmpty())
	{
		mjs_setString(Child->modelname, Utf8(Asset.MjName.GetValue()));
	}
	const char* const Registered = mjs_getString(Child->modelname);
	Ctx.Models->Add(FString(UTF8_TO_TCHAR(Registered != nullptr ? Registered : "")), Child);
	return true;
}

/** The name of a body or frame in a diagnostic. */
FString KindName(mjtObj Kind)
{
	const char* const Text = mju_type2Str(Kind);
	return FString(UTF8_TO_TCHAR(Text != nullptr ? Text : "element"));
}

/** Splice a registered model, or a body or frame of this one, onto the current body. */
bool ApplyAttach(FMjSpecWriteContext& Ctx, const UMjAttach& Attach)
{
	if (Ctx.Spec == nullptr || Ctx.Body == nullptr)
	{
		return Ctx.Error(Attach, TEXT("this attach has no body to be placed on"));
	}
	if (Ctx.Partial != nullptr)
	{
		return Ctx.Error(Attach, TEXT("an attach cannot be a default class partial"));
	}

	// Read as the reader reads it: `body` and `frame` share one name slot, and
	// `body` decides the kind when a document authors both.
	FString ChildName;
	mjtObj Kind = mjOBJ_UNKNOWN;
	if (Attach.Body.IsSet())
	{
		ChildName = Attach.Body.GetValue();
		Kind = mjOBJ_BODY;
	}
	if (Attach.Frame.IsSet())
	{
		ChildName = Attach.Frame.GetValue();
		Kind = Kind == mjOBJ_UNKNOWN ? mjOBJ_FRAME : Kind;
	}
	const FString& Prefix = Attach.Prefix;

	// Refused before anything is inserted, because a rejected attach cannot be
	// unwound and a prefixed name the spec already holds is exactly what it
	// rejects on.
	if (!ChildName.IsEmpty() && Kind != mjOBJ_UNKNOWN &&
		mjs_findElement(Ctx.Spec, Kind, Utf8(Prefix + ChildName)) != nullptr)
	{
		return Ctx.Error(Attach, FString::Printf(TEXT("this model already has a %s named '%s%s'"),
			*KindName(Kind), *Prefix, *ChildName));
	}

	mjsElement* Source = nullptr;
	if (!Attach.Model.IsSet())
	{
		if (Kind == mjOBJ_UNKNOWN)
		{
			return Ctx.Error(Attach, TEXT("an attach naming no model must name a body or a frame"));
		}
		Source = mjs_findElement(Ctx.Spec, Kind, Utf8(ChildName));
		if (Source == nullptr)
		{
			return Ctx.Error(Attach, FString::Printf(TEXT("this model has no %s named '%s' to attach to itself"),
				*KindName(Kind), *ChildName));
		}
	}
	else
	{
		const FString& ModelName = Attach.Model.GetValue();
		mjSpec* const Model = Ctx.Models != nullptr ? Ctx.Models->Find(ModelName) : nullptr;
		if (Model == nullptr)
		{
			return Ctx.Error(Attach, FString::Printf(TEXT("no model asset named '%s' was registered"), *ModelName));
		}
		// Naming neither a body nor a frame attaches the model itself, which is
		// what puts its whole world body's contents in.
		Source = Kind == mjOBJ_UNKNOWN ? Model->element : mjs_findElement(Model, Kind, Utf8(ChildName));
		if (Source == nullptr)
		{
			return Ctx.Error(Attach, FString::Printf(TEXT("model asset '%s' has no %s named '%s'"), *ModelName,
				*KindName(Kind), *ChildName));
		}
	}

	// mjs_attach splices onto a frame rather than onto a body, so an attach
	// authored outside one needs a frame of its own; inside one it uses that
	// frame and its transform.
	mjsFrame* Frame = Ctx.Frame;
	if (Frame == nullptr)
	{
		Frame = mjs_addFrame(Ctx.Body, nullptr);
		if (Frame == nullptr)
		{
			return Ctx.Error(Attach, TEXT("could not add the frame this attach hangs from"));
		}
		mjs_setDefault(Frame->element, Ctx.Class);
		if (!Attach.SourceFile.IsEmpty() || Attach.SourceLine > 0)
		{
			mjs_setString(Frame->info,
				Utf8(FString::Printf(TEXT("%s:%d"), *Attach.SourceFile, Attach.SourceLine)));
		}
	}

	// Authored attach copies, where composition moves. The reader raises this
	// flag for exactly the span in which an `<attach>` can appear and lowers it
	// again (`src/xml/xml_native_reader.cc:309,316`), and both halves matter: a
	// model asset spliced in twice needs a copy each time, a self-attach without
	// one puts the same body in the tree twice, and leaving it raised would
	// deep-copy the participant attach that Gate A settled on not copying. A
	// fresh spec has it lowered, and nothing else in the walk raises it, so
	// lowering it again restores what was there.
	mjs_setDeepCopy(Ctx.Spec, 1);
	mjsElement* const Attached = mjs_attach(Frame->element, Source, Utf8(Prefix), "");
	mjs_setDeepCopy(Ctx.Spec, 0);
	if (Attached == nullptr)
	{
		return Ctx.Abort(Attach, FString::Printf(TEXT("this attach failed and the model cannot be built: %s"),
			UTF8_TO_TCHAR(mjs_getError(Ctx.Spec))));
	}
	return true;
}

bool ApplyNestedModel(FMjSpecWriteContext& Ctx, const UMjNodeComponent& Node, mjsElement*)
{
	if (const UMjModelAsset* const Asset = Cast<UMjModelAsset>(&Node))
	{
		return ApplyModelAsset(Ctx, *Asset);
	}
	if (const UMjAttach* const Attach = Cast<UMjAttach>(&Node))
	{
		return ApplyAttach(Ctx, *Attach);
	}
	return Ctx.Error(Node, TEXT("this element is neither a model asset nor an attach"));
}

// --- Registry ------------------------------------------------------------- //

const FMjSpecWriteHookRow Rows[] = {
	{ TEXT("actuator_shorthand"), &CreateActuator, &ApplyShorthand },
	{ TEXT("transmission"), nullptr, &ApplyTransmission },
	{ TEXT("tendon_path"), nullptr, &ApplyTendonPath },
	{ TEXT("material_layers"), nullptr, &ApplyMaterialLayers },
	{ TEXT("skin_bones"), nullptr, &ApplySkinBone },
	{ TEXT("tuple_elements"), nullptr, &ApplyTupleElement },
	{ TEXT("plugins"), nullptr, &ApplyPlugin },
	{ TEXT("option_flags"), nullptr, &ApplyOptionFlags },
	{ TEXT("size_memory"), nullptr, &ApplySizeMemory },
	{ TEXT("macro_bridge"), nullptr, &ApplyMacroBridge },
	{ TEXT("input_fold"), nullptr, &ApplyInputFold },
	{ TEXT("nested_model"), nullptr, &ApplyNestedModel },
	{ TEXT("equality_fold"), nullptr, &ApplyEqualityFold },
	{ TEXT("sensor_fold"), nullptr, &ApplySensorFold },
	{ TEXT("compiler_placement"), nullptr, &ApplyCompilerPlacement },
	{ TEXT("asset_builtin"), nullptr, &ApplyAssetBuiltin },
	{ TEXT("flex_layout"), nullptr, &ApplyFlexLayout },
	{ TEXT("numeric_data"), nullptr, &ApplyNumericData },
};

}  // namespace

const FMjSpecWriteHookRow* FindSpecWriteHook(const TCHAR* Name)
{
	if (Name == nullptr)
	{
		return nullptr;
	}
	for (const FMjSpecWriteHookRow& Row : Rows)
	{
		if (FCString::Strcmp(Row.Name, Name) == 0)
		{
			return &Row;
		}
	}
	return nullptr;
}

bool SpecWriteRegistryComplete(TArray<FString>* OutMissing)
{
	bool bComplete = true;
	const sw::FMjHookList Wanted = sw::AllHooks();
	for (int32 Index = 0; Index < Wanted.Num; ++Index)
	{
		if (FindSpecWriteHook(Wanted.Names[Index]) == nullptr)
		{
			bComplete = false;
			if (OutMissing != nullptr)
			{
				OutMissing->AddUnique(FString(Wanted.Names[Index]));
			}
		}
	}
	return bComplete;
}

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
