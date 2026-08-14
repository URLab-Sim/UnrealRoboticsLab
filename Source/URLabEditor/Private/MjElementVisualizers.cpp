// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MjElementVisualizers.h"

#include "Editor/UnrealEdEngine.h"
#include "Misc/CoreDelegates.h"
#include "PrimitiveDrawInterface.h"
#include "PrimitiveDrawingUtils.h"
#include "SceneManagement.h"
#include "SceneView.h"
#include "UnrealEdGlobals.h"

#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Cameras/MjCamera.gen.h"
#include "MuJoCo/Gen/Elements/Cameras/MjLight.gen.h"
#include "MuJoCo/Gen/Elements/Geometry/MjGeom.gen.h"
#include "MuJoCo/Gen/Elements/Geometry/MjSite.gen.h"
#include "MuJoCo/Gen/Elements/Joints/MjFreeJoint.gen.h"
#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"
#include "MuJoCo/Gen/Elements/Options/MjCompiler.gen.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjElementIdentity.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

#endif // URLAB_MJ_GEN

namespace
{

#if URLAB_MJ_GEN

using namespace urlab::spec;

/** MJCF lengths are metres; Unreal's are centimetres. */
constexpr double MetresToUnreal = 100.0;

/** The palette the simulation's debug view uses, so the two read as one thing. */
const FLinearColor ArcColor(0.16f, 0.78f, 0.16f);
const FLinearColor LimitColor(0.86f, 0.24f, 0.24f);
const FLinearColor RefColor(0.70f, 0.70f, 0.86f);
const FLinearColor AxisColor(0.95f, 0.85f, 0.12f);
const FLinearColor MarkerColor(0.55f, 0.55f, 0.60f);

/** The range arc's radius, matching MjUtils::DrawDebugJoint's default. */
constexpr double ArcRadius = 10.0;

/**
 * Read an element's attributes the way the compiler will see them.
 *
 * `Function` is handed a reader: `Read(&UMjJoint::Axis)` gives the value from
 * the nearest layer that authored it -- the element itself, then its default
 * class chain -- as a `TOptional` that is unset when no layer did.
 *
 * ONE effective context per call, never one per attribute. Building a context
 * indexes every element and every default class in the spec, so an element that
 * reads five attributes through five contexts pays five whole-spec walks to
 * draw one joint, which is the shape of the editor lag this project has already
 * paid for once. Everything an element needs is read inside a single
 * `WithEffectiveDoc`, and the fallback path (no spec root reachable: a detached
 * component, or a class default object) reads authored values only.
 */
template <class E, class Fn>
void WithElementReader(const E& Element, Fn&& Function)
{
	const bool bResolved = WithEffectiveDoc(Element, [&Element, &Function](auto& Effective) {
		Function([&Element, &Effective](auto Field) {
			auto Out = Element.*Field;
			Out.Reset();
			Effective.ForEachLayer(Element, [&Out, Field](const auto& Layer) {
				if ((Layer.*Field).IsSet())
				{
					Out = Layer.*Field;
					return true;
				}
				return false;
			});
			return Out;
		});
	});
	if (!bResolved)
	{
		Function([&Element](auto Field) { return Element.*Field; });
	}
}

/**
 * The angle unit the spec's `<compiler>` declares.
 *
 * A joint `range` is authored in this unit, not in radians, and MJCF's default
 * is degrees -- so drawing an authored range as radians is wrong by 57x on
 * every model that does not say `angle="radian"`. The compiled model has no
 * such ambiguity, which is exactly why the ported drawing needs this and the
 * original did not.
 *
 * A property of the DOCUMENT, so it is asked once per sweep and not once per
 * hinge: the walk below is a whole-graph walk, and a model of a hundred joints
 * was paying a hundred of them per frame to draw one number that is the same
 * for all of them.
 */
double RadiansPerAuthoredAngle(const FSpecRef& Spec)
{
	UMjNodeComponent* const Root = Spec.GetRoot();
	if (Root == nullptr)
	{
		return UE_DOUBLE_PI / 180.0;
	}
	for (const FMjOrderedChild& Child : MjOrderedChildrenOf(Spec, *Root))
	{
		const UMjCompiler* const Compiler = Cast<UMjCompiler>(Child.Node);
		if (Compiler == nullptr)
		{
			continue;
		}
		return Compiler->Angle.Get(EMjAngleUnit::degree) == EMjAngleUnit::radian
				 ? 1.0
				 : UE_DOUBLE_PI / 180.0;
	}
	return UE_DOUBLE_PI / 180.0;
}

/** An MJCF rgb or rgba triple as a colour, or `Fallback` when it is not one. */
template <class T>
FLinearColor ColorOf(const TOptional<TArray<T>>& Rgb, const FLinearColor& Fallback)
{
	if (!Rgb.IsSet() || Rgb.GetValue().Num() < 3)
	{
		return Fallback;
	}
	const TArray<T>& Values = Rgb.GetValue();
	return FLinearColor(static_cast<float>(Values[0]), static_cast<float>(Values[1]),
		static_cast<float>(Values[2]), 1.0f);
}

/** The `Index`th entry of an MJCF size-like array in Unreal units. */
double SizeAt(const TOptional<TArray<double>>& Size, int32 Index, double Fallback)
{
	const double Metres = Size.IsSet() && Size.GetValue().IsValidIndex(Index)
							? Size.GetValue()[Index]
							: Fallback;
	return Metres * MetresToUnreal;
}

/** A unit vector perpendicular to `Axis`, however `Axis` is oriented. */
FVector PerpendicularTo(const FVector& Axis)
{
	FVector Radial = FVector::CrossProduct(Axis, FVector::UpVector);
	if (Radial.SizeSquared() < UE_SMALL_NUMBER)
	{
		Radial = FVector::CrossProduct(Axis, FVector::RightVector);
	}
	return Radial.GetSafeNormal();
}

// --- The element families --------------------------------------------- //

/**
 * A hinge's or a slide's axis and travel.
 *
 * Ported from `AMjArticulation::DrawDebugJoints`, with two differences forced
 * by there being no compiled model: the angles come from the authored `range`
 * through the compiler's angle unit, and there is no current position to mark
 * because nothing has stepped. The reference position is drawn when authored,
 * which is what `qpos0` would have shown.
 */
void DrawJoint(const UMjJoint& Joint, FPrimitiveDrawInterface* PDI, double RadiansPerAngle)
{
	EMjJointType Type = EMjJointType::hinge;
	FVector LocalAxis(0.0, 0.0, 1.0);
	FVector2D Range = FVector2D::ZeroVector;
	bool bLimited = false;
	double Ref = 0.0;

	WithElementReader(Joint, [&](auto Read) {
		Type = Read(&UMjJoint::Type).Get(EMjJointType::hinge);
		LocalAxis = Read(&UMjJoint::Axis).Get(FMjDirection3(0.0, 0.0, 1.0)).ToUnreal();
		Range = Read(&UMjJoint::Range).Get(FVector2D::ZeroVector);
		bLimited = Read(&UMjJoint::Limited).Get(EMjTriState::auto_) == EMjTriState::true_
				|| Range != FVector2D::ZeroVector;
		Ref = Read(&UMjJoint::Ref).Get(0.0);
	});

	if (Type != EMjJointType::hinge && Type != EMjJointType::slide)
	{
		return;
	}

	const FTransform Frame = Joint.GetComponentTransform();
	const FVector Anchor = Frame.GetLocation();
	const FVector Axis = Frame.TransformVectorNoScale(LocalAxis).GetSafeNormal();
	if (Axis.IsNearlyZero())
	{
		return;
	}

	// The axis itself, always: a joint with no range still has a direction, and
	// the direction is the thing a user gets wrong.
	PDI->DrawLine(Anchor - Axis * ArcRadius, Anchor + Axis * ArcRadius, AxisColor, SDPG_Foreground, 1.5f);

	if (Type == EMjJointType::hinge)
	{
		// Negated because the axis was Y-negated crossing into Unreal's
		// left-handed frame, so MuJoCo's right-hand rule reverses with it. The
		// limits swap for the same reason.
		const double Scale = -RadiansPerAngle * 180.0 / UE_DOUBLE_PI;
		const FVector Radial = PerpendicularTo(Axis);
		const FVector Bitangent = FVector::CrossProduct(Axis, Radial).GetSafeNormal();

		if (bLimited && Range != FVector2D::ZeroVector)
		{
			const double DrawMin = Scale * Range.Y;
			const double DrawMax = Scale * Range.X;
			DrawArc(PDI, Anchor, Radial, Bitangent, static_cast<float>(DrawMin), static_cast<float>(DrawMax),
				ArcRadius, 48, ArcColor, SDPG_Foreground);

			const FQuat MinRot(Axis, FMath::DegreesToRadians(DrawMin));
			const FQuat MaxRot(Axis, FMath::DegreesToRadians(DrawMax));
			PDI->DrawLine(Anchor, Anchor + MinRot.RotateVector(Radial) * ArcRadius, LimitColor,
				SDPG_Foreground, 1.5f);
			PDI->DrawLine(Anchor, Anchor + MaxRot.RotateVector(Radial) * ArcRadius, LimitColor,
				SDPG_Foreground, 1.5f);
		}

		const FQuat RefRot(Axis, FMath::DegreesToRadians(Scale * Ref));
		PDI->DrawLine(Anchor, Anchor + RefRot.RotateVector(Radial) * ArcRadius * 0.9, RefColor,
			SDPG_Foreground, 1.0f);
	}
	else
	{
		// A slide's range is a length, so it is metres whatever the angle unit
		// says.
		if (bLimited && Range != FVector2D::ZeroVector)
		{
			const FVector MinPoint = Anchor + Axis * Range.X * MetresToUnreal;
			const FVector MaxPoint = Anchor + Axis * Range.Y * MetresToUnreal;
			const FVector Tick = PerpendicularTo(Axis) * 4.0;

			PDI->DrawLine(MinPoint, MaxPoint, ArcColor, SDPG_Foreground, 1.5f);
			PDI->DrawLine(MinPoint - Tick, MinPoint + Tick, LimitColor, SDPG_Foreground, 1.5f);
			PDI->DrawLine(MaxPoint - Tick, MaxPoint + Tick, LimitColor, SDPG_Foreground, 1.5f);
		}

		const FVector RefPoint = Anchor + Axis * Ref * MetresToUnreal;
		PDI->DrawPoint(RefPoint, RefColor, 6.0f, SDPG_Foreground);
	}
}

/** A free joint has no axis and no range: what it has is six degrees of freedom. */
void DrawFreeJoint(const UMjFreeJoint& Joint, FPrimitiveDrawInterface* PDI)
{
	const FTransform Frame = Joint.GetComponentTransform();
	DrawCoordinateSystem(PDI, Frame.GetLocation(), Frame.Rotator(), static_cast<float>(ArcRadius),
		SDPG_Foreground, 1.5f);
}

/**
 * A locator marker's size in Unreal units, so a tiny site stays locatable.
 *
 * MuJoCo's own 5 mm default site floors the SHAPE at 0.5 UU (`DrawSite`
 * leaves that alone), but an authored 1 mm site draws a shape smaller still
 * and reads as nothing at ordinary zoom. This is a second, independent floor
 * on a marker added next to the shape: `MarkerFloor` off-screen or with no
 * view to measure against, and -- when a view is available -- the standard
 * editor-gizmo scale factor, so the marker holds a constant size in pixels
 * rather than shrinking at a distance or swamping the shape up close.
 *
 * `View` is null off both of this drawing's existing tests
 * (`FMjElementVisualizerDrawsInheritedSize`,
 * `FMjElementVisualizerReadsRangeInAuthoredAngleUnit`, `MjVisualizerTests.cpp`),
 * so every read of it is guarded rather than assumed present.
 */
double LocatorMarkerSize(const FSceneView* View, const FVector& Location)
{
	constexpr double MarkerFloor = 2.0;
	if (View == nullptr)
	{
		return MarkerFloor;
	}
	const double ScreenScale = View->WorldToScreen(Location).W
							 * (4.0 / View->UnscaledViewRect.Width() / View->ViewMatrices.GetProjectionMatrix().M[0][0]);
	return FMath::Max(ScreenScale, MarkerFloor);
}

/**
 * A site's shape, in wireframe.
 *
 * A site is a geom's geometry without a geom's physics, and MuJoCo sizes it by
 * the same rules, so the shape table is the geom's: sphere by one radius,
 * capsule and cylinder by radius and half-length along local Z, box and
 * ellipsoid by three half-extents.
 */
void DrawSite(const UMjSite& Site, const FSceneView* View, FPrimitiveDrawInterface* PDI)
{
	EMjGeomType Type = EMjGeomType::sphere;
	TOptional<TArray<double>> Size;
	FLinearColor Color(0.5f, 0.5f, 0.5f, 1.0f);

	WithElementReader(Site, [&](auto Read) {
		Type = Read(&UMjSite::Type).Get(EMjGeomType::sphere);
		Size = Read(&UMjSite::Size);
		Color = Read(&UMjSite::Rgba).Get(FLinearColor(0.5f, 0.5f, 0.5f, 1.0f));
	});
	Color.A = 1.0f;

	FTransform Frame = Site.GetComponentTransform();
	Frame.RemoveScaling();
	const FVector Base = Frame.GetLocation();
	const FVector AxisX = Frame.GetUnitAxis(EAxis::X);
	const FVector AxisY = Frame.GetUnitAxis(EAxis::Y);
	const FVector AxisZ = Frame.GetUnitAxis(EAxis::Z);

	// MuJoCo's own default site size, so an unsized site is visible rather than
	// invisible.
	const double Radius = FMath::Max(SizeAt(Size, 0, 0.005), 0.5);

	switch (Type)
	{
		case EMjGeomType::sphere:
			DrawWireSphere(PDI, Base, Color, Radius, 16, SDPG_Foreground);
			break;

		case EMjGeomType::ellipsoid:
		{
			const double RadiusY = FMath::Max(SizeAt(Size, 1, 0.005), 0.5);
			const double RadiusZ = FMath::Max(SizeAt(Size, 2, 0.005), 0.5);
			DrawCircle(PDI, Base, AxisX * Radius, AxisY * RadiusY, Color, 1.0, 24, SDPG_Foreground);
			DrawCircle(PDI, Base, AxisX * Radius, AxisZ * RadiusZ, Color, 1.0, 24, SDPG_Foreground);
			DrawCircle(PDI, Base, AxisY * RadiusY, AxisZ * RadiusZ, Color, 1.0, 24, SDPG_Foreground);
			break;
		}

		case EMjGeomType::capsule:
		{
			// MJCF sizes a capsule by its cylinder's half-length; Unreal's helper
			// wants the half-height of the whole thing, hemispheres included.
			const double HalfLength = SizeAt(Size, 1, 0.005);
			DrawWireCapsule(PDI, Base, AxisX, AxisY, AxisZ, Color, Radius, HalfLength + Radius, 16,
				SDPG_Foreground);
			break;
		}

		case EMjGeomType::cylinder:
			DrawWireCylinder(PDI, Base, AxisX, AxisY, AxisZ, Color, Radius, SizeAt(Size, 1, 0.005), 16,
				SDPG_Foreground);
			break;

		case EMjGeomType::box:
		{
			const FVector Extent(Radius, FMath::Max(SizeAt(Size, 1, 0.005), 0.5),
				FMath::Max(SizeAt(Size, 2, 0.005), 0.5));
			DrawWireBox(PDI, Frame.ToMatrixNoScale(), FBox(-Extent, Extent), Color, SDPG_Foreground);
			break;
		}

		default:
			// A plane, a height field or a mesh site: the cross the simulation
			// draws, which locates it without claiming a shape it cannot size.
			PDI->DrawLine(Base - AxisX * Radius, Base + AxisX * Radius, Color, SDPG_Foreground, 1.0f);
			PDI->DrawLine(Base - AxisY * Radius, Base + AxisY * Radius, Color, SDPG_Foreground, 1.0f);
			PDI->DrawLine(Base - AxisZ * Radius, Base + AxisZ * Radius, Color, SDPG_Foreground, 1.0f);
			break;
	}

	// The locator marker: independent of the shape and its own floor, so a
	// well-sized site is unaffected (the marker sits inside the shape it
	// already drew) and a millimetre-scale one is not simply undetectable.
	DrawWireDiamond(PDI, Frame.ToMatrixNoScale(), static_cast<float>(LocatorMarkerSize(View, Base)), MarkerColor,
		SDPG_Foreground);
}

/**
 * A light's position, direction and reach.
 *
 * Drawn rather than previewed with a real `ULightComponent`: a light component
 * parented to a Blueprint template is a component the Blueprint can serialise,
 * and an editor-only preview that ships inside the asset is a worse bug than no
 * preview at all. The wireframe carries the same three facts a preview light
 * would -- where it is, which way it points, how wide it opens.
 */
void DrawLight(const UMjLight& Light, FPrimitiveDrawInterface* PDI)
{
	EMjLightType Type = EMjLightType::spot;
	FVector LocalDirection(0.0, 0.0, -1.0);
	FLinearColor Color(0.7f, 0.7f, 0.7f, 1.0f);
	double CutoffDegrees = 45.0;

	WithElementReader(Light, [&](auto Read) {
		Type = Read(&UMjLight::Type).Get(EMjLightType::spot);
		LocalDirection = Read(&UMjLight::Dir).Get(FMjDirection3(0.0, 0.0, -1.0)).ToUnreal();
		Color = ColorOf(Read(&UMjLight::Diffuse), FLinearColor(0.7f, 0.7f, 0.7f, 1.0f));
		CutoffDegrees = Read(&UMjLight::Cutoff).Get(45.0f);
	});

	const FTransform Frame = Light.GetComponentTransform();
	const FVector Origin = Frame.GetLocation();
	const FVector Direction = Frame.TransformVectorNoScale(LocalDirection).GetSafeNormal();
	if (Direction.IsNearlyZero())
	{
		return;
	}

	DrawWireSphere(PDI, Origin, Color, 3.0, 12, SDPG_Foreground);

	const double Reach = ArcRadius * 3.0;
	if (Type == EMjLightType::directional)
	{
		// Parallel rays: a directional light has a direction and no origin, and
		// a cone would claim it falls off.
		const FVector Right = PerpendicularTo(Direction);
		const FVector Up = FVector::CrossProduct(Direction, Right).GetSafeNormal();
		for (int32 Index = 0; Index < 4; ++Index)
		{
			const double Angle = Index * UE_DOUBLE_HALF_PI;
			const FVector Offset = (Right * FMath::Cos(Angle) + Up * FMath::Sin(Angle)) * 4.0;
			PDI->DrawLine(Origin + Offset, Origin + Offset + Direction * Reach, Color, SDPG_Foreground, 1.0f);
		}
		return;
	}

	PDI->DrawLine(Origin, Origin + Direction * Reach, Color, SDPG_Foreground, 1.5f);

	if (Type == EMjLightType::spot)
	{
		// The cone the cutoff describes, as a rim circle and four edges: built
		// from the direction rather than from a transform so the half-angle
		// cannot silently become a full angle.
		const double HalfAngle = FMath::DegreesToRadians(FMath::Clamp(CutoffDegrees, 1.0, 89.0));
		const double RimRadius = Reach * FMath::Tan(HalfAngle);
		const FVector Right = PerpendicularTo(Direction);
		const FVector Up = FVector::CrossProduct(Direction, Right).GetSafeNormal();
		const FVector RimCentre = Origin + Direction * Reach;

		DrawCircle(PDI, RimCentre, Right, Up, Color, RimRadius, 24, SDPG_Foreground);
		for (int32 Index = 0; Index < 4; ++Index)
		{
			const double Angle = Index * UE_DOUBLE_HALF_PI;
			const FVector Rim = RimCentre + (Right * FMath::Cos(Angle) + Up * FMath::Sin(Angle)) * RimRadius;
			PDI->DrawLine(Origin, Rim, Color, SDPG_Foreground, 1.0f);
		}
	}
}

/**
 * A camera's frustum.
 *
 * MuJoCo cameras look down their own -Z with +Y up, which crossing into
 * Unreal's frame turns into local -Z forward and local -Y up (the same
 * correction `UMjCamera` applies to its scene capture). `fovy` is the VERTICAL
 * field of view in degrees and the horizontal one follows from the authored
 * resolution's aspect, which is why the resolution is read here at all.
 */
void DrawCamera(const UMjCameraBase& Camera, FPrimitiveDrawInterface* PDI)
{
	double Fovy = 45.0;
	double Aspect = 1.0;

	WithElementReader(Camera, [&](auto Read) {
		Fovy = Read(&UMjCameraBase::Fovy).Get(45.0);
		const TOptional<TArray<int32>> Resolution = Read(&UMjCameraBase::Resolution);
		if (Resolution.IsSet() && Resolution.GetValue().Num() >= 2 && Resolution.GetValue()[1] > 0)
		{
			Aspect = static_cast<double>(Resolution.GetValue()[0])
				   / static_cast<double>(Resolution.GetValue()[1]);
		}
	});

	const FTransform Frame = Camera.GetComponentTransform();
	const FVector Origin = Frame.GetLocation();
	const FVector Forward = -Frame.GetUnitAxis(EAxis::Z);
	const FVector Up = -Frame.GetUnitAxis(EAxis::Y);
	const FVector Right = Frame.GetUnitAxis(EAxis::X);

	const double Depth = ArcRadius * 3.0;
	const double HalfHeight = Depth * FMath::Tan(FMath::DegreesToRadians(FMath::Clamp(Fovy, 1.0, 179.0)) * 0.5);
	const double HalfWidth = HalfHeight * FMath::Max(Aspect, UE_KINDA_SMALL_NUMBER);

	const FVector Centre = Origin + Forward * Depth;
	const FVector Corners[4] = {
		Centre + Right * HalfWidth + Up * HalfHeight,
		Centre - Right * HalfWidth + Up * HalfHeight,
		Centre - Right * HalfWidth - Up * HalfHeight,
		Centre + Right * HalfWidth - Up * HalfHeight,
	};

	for (int32 Index = 0; Index < 4; ++Index)
	{
		PDI->DrawLine(Origin, Corners[Index], MarkerColor, SDPG_Foreground, 1.0f);
		PDI->DrawLine(Corners[Index], Corners[(Index + 1) % 4], MarkerColor, SDPG_Foreground, 1.0f);
	}

	// Which way is up, on a frustum that is otherwise symmetric.
	PDI->DrawLine(Corners[0], Centre + Up * HalfHeight * 1.4, MarkerColor, SDPG_Foreground, 1.0f);
	PDI->DrawLine(Corners[1], Centre + Up * HalfHeight * 1.4, MarkerColor, SDPG_Foreground, 1.0f);
}

/** Everything else: a marker, so a selected element is at least locatable. */
void DrawMarker(const USceneComponent& Element, FPrimitiveDrawInterface* PDI)
{
	DrawWireDiamond(PDI, Element.GetComponentTransform().ToMatrixNoScale(), 3.0f, MarkerColor,
		SDPG_Foreground);
}

#endif // URLAB_MJ_GEN

} // namespace

FMjElementVisualizer::FMjElementVisualizer()
{
#if URLAB_MJ_GEN
	// The sweep's index is a snapshot of a tree, and the frame that painted it
	// is the longest it may describe one. End of frame is after every viewport
	// has drawn and before anything can edit again, which is the exact boundary.
	EndFrameHandle = FCoreDelegates::OnEndFrame.AddRaw(this, &FMjElementVisualizer::EndSweep);
#endif
}

FMjElementVisualizer::~FMjElementVisualizer()
{
#if URLAB_MJ_GEN
	FCoreDelegates::OnEndFrame.Remove(EndFrameHandle);
	EndSweep();
#endif
}

#if URLAB_MJ_GEN

void FMjElementVisualizer::BeginSweep(const UMjNodeComponent& Element)
{
	using namespace urlab::spec;

	const FSpecRef Doc = FSpecRef::OverOwner(&Element);
	const UMjModel* const Root = Cast<UMjModel>(Doc.GetRoot());
	if (Root == nullptr)
	{
		EndSweep();
		return;
	}
	if (SweepRoot.Get() == Root && SweepFrame == GFrameNumber)
	{
		return;
	}

	EndSweep();
	SweepRoot = Root;
	SweepFrame = GFrameNumber;
	// Only when nobody else is holding one over this spec. A scope that adopted
	// an outer scope's indexes would outlive the object it borrowed them from,
	// and this one deliberately lives longer than the call that made it.
	if (FMjEffectiveScope::Find(Root) == nullptr)
	{
		SweepScope = MakeUnique<FMjEffectiveScope>(Doc);
	}
	SweepRadiansPerAngle = RadiansPerAuthoredAngle(Doc);
}

void FMjElementVisualizer::EndSweep()
{
	SweepScope.Reset();
	SweepRoot = nullptr;
	SweepRadiansPerAngle.Reset();
}

#endif // URLAB_MJ_GEN

void FMjElementVisualizer::DrawVisualization(const UActorComponent* Component, const FSceneView* View,
	FPrimitiveDrawInterface* PDI)
{
#if URLAB_MJ_GEN
	const UMjNodeComponent* const Element = Cast<UMjNodeComponent>(Component);
	if (Element == nullptr || PDI == nullptr || Element->HasAnyFlags(RF_ClassDefaultObject))
	{
		return;
	}

	// The first element of the frame builds the spec's index; every element
	// after it joins that one. Before the class-partial test below, because
	// that test is itself an ancestor walk over the spec.
	BeginSweep(*Element);

	// A `<default>` partial is an inheritance template, not an element: MuJoCo
	// never places one, so drawing it would put a joint axis or a light cone at
	// the origin belonging to nothing. The geom preview makes the same
	// exclusion for the same reason.
	if (Element->IsClassPartial())
	{
		return;
	}

	if (const UMjJoint* const Joint = Cast<UMjJoint>(Element))
	{
		DrawJoint(*Joint, PDI, SweepRadiansPerAngle.Get(UE_DOUBLE_PI / 180.0));
	}
	else if (const UMjFreeJoint* const FreeJoint = Cast<UMjFreeJoint>(Element))
	{
		DrawFreeJoint(*FreeJoint, PDI);
	}
	else if (const UMjSite* const Site = Cast<UMjSite>(Element))
	{
		DrawSite(*Site, View, PDI);
	}
	else if (const UMjLight* const Light = Cast<UMjLight>(Element))
	{
		DrawLight(*Light, PDI);
	}
	else if (const UMjCameraBase* const Camera = Cast<UMjCameraBase>(Element))
	{
		DrawCamera(*Camera, PDI);
	}
	else if (Element->IsA<UMjGeomBase>())
	{
		// A geom already previews as a real mesh, and a marker on top of it
		// would only be in the way -- except when there is no mesh: a
		// `childclass` template that inherits `type="mesh"` and names no mesh
		// builds no preview at all, and a null `Cast<UMjGeom>` (a base-class
		// template) means the same thing. That geom is otherwise unlocatable
		// and silently breaks the next compile, so it gets the same marker
		// everything else without a mesh gets.
		const UMjGeom* const Geom = Cast<UMjGeom>(Element);
		if (Geom == nullptr || Geom->GetVisualizerMesh() == nullptr)
		{
			DrawMarker(*Element, PDI);
		}
	}
	else
	{
		DrawMarker(*Element, PDI);
	}
#endif // URLAB_MJ_GEN
}

FDelegateHandle FMjElementVisualizer::PostEngineInitHandle;

void FMjElementVisualizer::RegisterAll()
{
	if (GUnrealEd == nullptr)
	{
		// Not an error, and that is the danger: the visualizer registry belongs
		// to the editor engine, and this module can be loaded before the engine
		// exists. Registering then succeeds at nothing, silently, and every
		// element draws nothing for the rest of the session. Wait for the
		// engine and register once it is there.
		if (!PostEngineInitHandle.IsValid())
		{
			PostEngineInitHandle = FCoreDelegates::OnPostEngineInit.AddStatic(&FMjElementVisualizer::RegisterAll);
		}
		return;
	}
	// Removed, not merely forgotten: the binding is what brought us here, and
	// dropping the handle on its own leaves a static delegate pointing into this
	// module for the rest of the process, which a hot reload then calls.
	if (PostEngineInitHandle.IsValid())
	{
		FCoreDelegates::OnPostEngineInit.Remove(PostEngineInitHandle);
		PostEngineInitHandle.Reset();
	}
	// One registration for every element class. UnrealEd resolves a visualizer
	// by walking up the component's class chain, so registering the base covers
	// all 145 generated classes and the hand subclasses over them -- and a new
	// element added by a schema bump is covered the day it is generated.
	GUnrealEd->RegisterComponentVisualizer(UMjNodeComponent::StaticClass()->GetFName(),
		MakeShared<FMjElementVisualizer>());
}

void FMjElementVisualizer::UnregisterAll()
{
	if (PostEngineInitHandle.IsValid())
	{
		FCoreDelegates::OnPostEngineInit.Remove(PostEngineInitHandle);
		PostEngineInitHandle.Reset();
	}
	if (GUnrealEd != nullptr)
	{
		GUnrealEd->UnregisterComponentVisualizer(UMjNodeComponent::StaticClass()->GetFName());
	}
}
