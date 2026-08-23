// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "Components/SceneComponent.h"
#include "MuJoCo/Entity/MjOverlayFlags.h"
#include "MjOverlayRenderer.generated.h"

struct mjModel_;
struct FMjRenderSnapshot;
class UInstancedStaticMeshComponent;
class UStaticMesh;
class UMaterialInterface;

/**
 * Draws MuJoCo's visualization overlays -- collision hulls, site crosses, contact
 * forces, CoM/inertia markers, camera/light glyphs, actuators, tendons,
 * rangefinders, constraints, static-body and auto-connect markers -- for a
 * compiled model, gated by a composable FMjOverlayFlags bitmask that mirrors
 * mjvOption.
 *
 * Rendering model (source-of-truth 8.5): every overlay primitive is one instance
 * in a pooled UInstancedStaticMeshComponent (one component per unit mesh x colour x
 * blend), so the whole overlay is O(number-of-distinct-styles) draw calls and
 * scales to hundreds of contacts -- the per-actor DrawDebug* path this replaces
 * could not. Primitives compose from four engine BasicShapes meshes (sphere, cube,
 * cylinder, cone): arrows are shaft-cylinder + cone head, lines/wire-boxes are thin
 * cylinders. Pools are cleared and refilled each frame (high-water-mark reuse), so
 * the visual is identical to the old one-frame DrawDebug lifetime.
 *
 * Joints are deliberately EXCLUDED -- the plugin renders them natively; this
 * renderer no longer ports DrawJoints.
 *
 * The topology (geom shapes, site sizes, inertia) comes from the model; every pose
 * comes from the engine's published render snapshot rather than live mjData, so this
 * is a safe game-thread call while the physics worker steps. The owner drives
 * DrawOverlays once per frame under the render-state lock.
 *
 * The perturbation drag spring (UMjPerturbation) and the legacy contact debug
 * (UMjDebugVisualizer) route their own draws through this renderer's auxiliary
 * layers so all overlay geometry shares one pooled ISM substrate.
 */
UCLASS(ClassGroup = (URLab), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjOverlayRenderer : public USceneComponent
{
	GENERATED_BODY()

public:
	UMjOverlayRenderer();

	/** Bind the model whose topology the overlays read. Borrowed; the caller owns it. */
	void SetModel(mjModel_* InModel);

	/** Which overlays to emit, and the per-group visibility masks. */
	FMjOverlayFlags Flags;

	/** Draw the site crosses this frame (our extra, mirroring the authoring toggle). */
	bool bDrawSites = false;

	/** Draw the net-new-vs-upstream 6-DOF wrench glyph (force arrow + curl/torus
	 *  torque glyph) for the contact torque (confrc[3:6]) and applied torque
	 *  (xfrc[3:6]) that upstream computes but never renders. OFF by default: opt-in
	 *  so it does not clutter the scene unless explicitly enabled. Scaled by
	 *  vis.map.torque, coloured vis.rgba.contacttorque. See source-of-truth 8.5. */
	bool bDrawWrench = false;

	/** Active perturbation to visualise (mjVIS_PERTURBFORCE / mjVIS_PERTURBOBJ),
	 *  refreshed by the owner each frame from UMjPerturbation. Body id < 0 = none. */
	int32 PerturbBodyId = -1;

	/** Applied perturb wrench on the selected body (force xyz, torque xyz), MuJoCo world. */
	double PerturbForce[6] = {0, 0, 0, 0, 0, 0};

	/** World offset applied to every drawn primitive (UE cm), matching the render
	 *  scene's own origin so the overlays land on the geometry, not the world zero. */
	FVector SceneOrigin = FVector::ZeroVector;

	/** Emit the enabled overlays for this frame from the snapshot's poses. No-op
	 *  without a bound model. */
	void DrawOverlays(const FMjRenderSnapshot& Snap);

	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	// --- Auxiliary layers driven by sibling components (perturbation, debug viz) ---
	// These live in independent pools so the main-overlay per-frame clear never wipes
	// them; each caller clears + refills its own layer.

	/** Perturbation drag spring: grab-point marker + spring/tangent arrow. All UE
	 *  world-space cm. Clears the drag layer, then emits. */
	void DrawDragSpring(const FVector& GrabUE, const FVector& TargetUE, bool bTranslate,
		bool bRotate, const FVector& RotTangentEndUE);
	/** Hide the drag spring (drag released). */
	void ClearDragSpring();

	/** Legacy contact debug (UMjDebugVisualizer): begin a fresh contact batch. */
	void BeginDebugContacts();
	/** One contact point marker (UE world-space cm). */
	void AddDebugContactPoint(const FVector& PointUE, float RadiusCm, const FColor& Color);
	/** One contact force arrow (UE world-space cm). */
	void AddDebugContactArrow(const FVector& FromUE, const FVector& ToUE, const FColor& Color);

private:
	// The four engine BasicShapes unit meshes every overlay primitive composes from.
	enum class EMjMesh : uint8
	{
		Sphere = 0,
		Cube = 1,
		Cylinder = 2,
		Cone = 3,
		Count = 4
	};

	// One pooled ISM component per (mesh, colour, blend). Draw calls scale with the
	// number of distinct styles on screen, not the number of primitives.
	struct FPoolLayer
	{
		TMap<uint64, TObjectPtr<UInstancedStaticMeshComponent>> Pools;
	};

	void EnsureResources();
	UInstancedStaticMeshComponent* GetPool(FPoolLayer& Layer, EMjMesh Mesh, const FColor& Color,
		bool bTranslucent, bool bCustomDepth);
	void ClearLayer(FPoolLayer& Layer);

	// Low-level: add one mesh instance in world space with per-instance RGBA custom data.
	void AddPrim(FPoolLayer& Layer, EMjMesh Mesh, const FTransform& WorldXform, const FColor& Color,
		bool bTranslucent, bool bCustomDepth = false);

	// Composed primitives (all world-space cm), emitted into the given layer.
	void EmitSphere(FPoolLayer& Layer, const FVector& Center, float RadiusCm, const FColor& Color,
		bool bTranslucent, bool bCustomDepth = false);
	void EmitEllipsoid(FPoolLayer& Layer, const FVector& Center, const FVector& RadiiCm,
		const FQuat& Rot, const FColor& Color, bool bTranslucent);
	void EmitBox(FPoolLayer& Layer, const FVector& Center, const FVector& HalfExtentCm,
		const FQuat& Rot, const FColor& Color, bool bTranslucent);
	void EmitCylinder(FPoolLayer& Layer, const FVector& A, const FVector& B, float RadiusCm,
		const FColor& Color, bool bTranslucent);
	void EmitLine(FPoolLayer& Layer, const FVector& A, const FVector& B, float ThicknessCm,
		const FColor& Color);
	void EmitArrow(FPoolLayer& Layer, const FVector& A, const FVector& B, float ShaftRadiusCm,
		float HeadRadiusCm, const FColor& Color, bool bTranslucent);
	void EmitWireBox(FPoolLayer& Layer, const FVector& Center, const FVector& HalfExtentCm,
		const FQuat& Rot, float ThicknessCm, const FColor& Color);
	// Curl/torus-about-axis torque glyph: an arced arrow (a ring of short cylinder
	// segments closed by a tangent cone head) encircling AxisUE by the right-hand
	// rule, composed entirely from the existing cylinder/cone primitives. RadiusCm is
	// the ring radius (already scaled by the caller); the head shows rotation sense.
	void EmitTorqueGlyph(FPoolLayer& Layer, const FVector& Center, const FVector& AxisUE,
		float RadiusCm, float TubeRadiusCm, const FColor& Color, bool bTranslucent);

	// Per-overlay builders (main layer). Snapshot-driven, thread-safe off the game thread.
	void DrawCollision(const FMjRenderSnapshot& Snap);
	void DrawSites(const FMjRenderSnapshot& Snap);
	void DrawCom(const FMjRenderSnapshot& Snap);
	void DrawInertia(const FMjRenderSnapshot& Snap);
	void DrawContacts(const FMjRenderSnapshot& Snap, bool bPoints, bool bForces, bool bSplit);
	void DrawPerturb(const FMjRenderSnapshot& Snap);
	void DrawCameras(const FMjRenderSnapshot& Snap);
	void DrawLights(const FMjRenderSnapshot& Snap);
	void DrawActuators(const FMjRenderSnapshot& Snap);
	void DrawTendons(const FMjRenderSnapshot& Snap);
	void DrawRangefinders(const FMjRenderSnapshot& Snap);
	void DrawConstraints(const FMjRenderSnapshot& Snap);
	void DrawStaticBodies(const FMjRenderSnapshot& Snap);
	void DrawAutoConnect(const FMjRenderSnapshot& Snap);
	// The net-new 6-DOF wrench glyph (opt-in, gated by bDrawWrench). Draws force
	// arrow + torque curl for every contact (force[6] in the contact frame) and for
	// every body carrying an applied wrench (xfrc_applied[6], world frame).
	void DrawWrenchGlyphs(const FMjRenderSnapshot& Snap);

	// Borrowed; the owning scene holds the mjModel lifetime.
	mjModel_* Model = nullptr;

	// Main overlay pools (cleared + refilled each frame by DrawOverlays); auxiliary
	// pools owned by sibling components (cleared by their own callers).
	FPoolLayer MainLayer;
	FPoolLayer DragLayer;
	FPoolLayer DebugContactLayer;

	// Unit meshes + their local half-extents / pivot Z, probed once at runtime so the
	// exact asset dimensions never need hardcoding. Indexed by EMjMesh; sized to
	// EMjMesh::Count in EnsureResources.
	UPROPERTY(Transient)
	TArray<TObjectPtr<UStaticMesh>> Meshes;
	FVector MeshExtent[static_cast<int32>(EMjMesh::Count)] = {};
	float MeshPivotZ[static_cast<int32>(EMjMesh::Count)] = {};

	// Fallback overlay material. Per-instance colour is carried both by the per-pool
	// MID (correct today) and by 4-float PerInstanceCustomData (RGBA, forward-compatible
	// with an authored master material that reads it). See the .cpp header note.
	UPROPERTY(Transient)
	TObjectPtr<UMaterialInterface> OpaqueBaseMaterial;
	UPROPERTY(Transient)
	TObjectPtr<UMaterialInterface> TranslucentBaseMaterial;
	FName ColorParamName = NAME_None;

	bool bResourcesReady = false;
	// Frame number of the last DrawOverlays; the tick clears the main layer once
	// overlays stop being driven (all vis flags off), matching DrawDebug's expiry.
	uint64 LastDrawFrame = 0;
};
