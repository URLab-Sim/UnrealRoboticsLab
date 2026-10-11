// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "MjFromtoFold.h"
#if URLAB_MJ_GEN
#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjScalePolicy.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "MuJoCo/Gen/Elements/Geometry/MjGeom.gen.h"
#include "MuJoCo/Gen/Elements/Geometry/MjSite.gen.h"
THIRD_PARTY_INCLUDES_START
#include "resolve.h"
THIRD_PARTY_INCLUDES_END

namespace urlab::spec {
namespace {
constexpr double MjEps = 1e-14;
bool AdmitsFromto(EMjGeomType T) {
  return T == EMjGeomType::capsule || T == EMjGeomType::cylinder ||
         T == EMjGeomType::ellipsoid || T == EMjGeomType::box;
}
template<class N, class F> void WithSpatial(N& Node, F&& Fn) {
  if (auto* G = Cast<UMjGeomBase>(&Node)) Fn(*G);
  else if (auto* S = Cast<UMjSite>(&Node)) Fn(*S);
}
bool IsDefaultClass(const UMjNodeComponent& N) { return Cast<UMjDefault>(&N) != nullptr; }
double VectorNorm(const double V[3]) {
  double Sum = 0.0;
  for (int I = 0; I < 3; ++I) Sum += V[I] * V[I];
  return Sum < MjEps ? 0.0 : FMath::Sqrt(Sum);
}
void AxisToQuat(const double V[3], double Q[4]) {
  const std::array<double, 4> Native =
      ps::core::ResolveOrientation(ps::core::OrientKind::ZAxis, V, ps::core::OrientContext{});
  for (int I = 0; I < 4; ++I) Q[I] = Native[I];
}
struct FSnapshot {
  UMjNodeComponent* Node = nullptr;
  bool Pose = false, Dimensions = false, ClearFromto = false, ClearSize = false;
  bool ClearPos = false, ClearQuat = false;
  double Pos[3] = {0,0,0}, Quat[4] = {1,0,0,0};
  FVector HalfExtentsCm = FVector::ZeroVector;
};
struct FFamily { TArray<double> Fromto, Size; };
struct FTemplates { FFamily Geom, Site; };
// Endpoint delta, half-length and midpoint adapted from MuJoCo 3.11.1
// src/user/user_objects.cc:3977-4014 (geom) and :4255-4288 (site),
// revision 2544c349977261541ad493ce03b32c711618b91d.
// Copyright 2021 DeepMind Technologies Limited; Apache License 2.0.
// AxisToQuat calls MuJoCo's public mjs_resolveOrientation rather than copying
// the private mjuu_z2quat used in those compiler blocks.
bool MakeSpatial(const TArray<double>& Fromto, const TArray<double>& Size, FSnapshot& Out) {
  if (Fromto.Num() != 6) return false;
  const double V[3] = {Fromto[0]-Fromto[3], Fromto[1]-Fromto[4], Fromto[2]-Fromto[5]};
  const double Half = VectorNorm(V) / 2.0;
  if (Half < MjEps) return false;
  const double Radius = Size.IsEmpty() ? 0.0 : Size[0];
  Out.HalfExtentsCm = FVector(Radius*100.0, Radius*100.0, Half*100.0);
  for (int I = 0; I < 3; ++I) Out.Pos[I] = (Fromto[I]+Fromto[I+3])/2.0;
  AxisToQuat(V, Out.Quat);
  return true;
}
template<class E> TArray<double> SchemaSize(const E& Element) {
  TArray<FMjEffectiveLayer> Layers;
  MjEffectiveLayersOf(Element, Layers);
  TArray<double> Result;
  for (const FMjEffectiveLayer& Layer : Layers) {
    if (!Layer.bSchema) continue;
    WithSpatial(*Layer.Node, [&](const auto& Partial) {
      if (Partial.Size.IsSet()) Result = Partial.Size.GetValue();
    });
    break;
  }
  return Result;
}
template<class E> bool CapturePartial(E& Element, FFamily& State, TArray<FSnapshot>& Pending) {
  const bool Fromto = Element.Fromto.IsSet(), Size = Element.Size.IsSet();
  if (Fromto) State.Fromto = Element.Fromto.GetValue();
  if (Size) State.Size = Element.Size.GetValue();
  if (State.Size.IsEmpty()) State.Size = SchemaSize(Element);
  if (State.Fromto.Num() != 6) return true;
  FSnapshot S;
  S.Node = &Element;
  S.ClearFromto = Fromto; S.ClearSize = Size;
  S.ClearPos = Element.Pos.IsSet() && !Fromto;
  S.ClearQuat = Element.Quat.IsSet() && !Fromto;
  S.Pose = Fromto; S.Dimensions = Fromto || Size;
  if (S.Dimensions && !MakeSpatial(State.Fromto, State.Size, S)) return false;
  Pending.Add(S);
  return true;
}
template<class A> bool CaptureClass(UMjNodeComponent& Class, FTemplates& State,
                                    TArray<FSnapshot>& Pending) {
  bool Valid = true;
  // Native reader applies direct partials before constructing nested classes.
  for (const FMjOrderedChild& C : A::OrderedChildren(Class)) {
    if (!C.Node || IsDefaultClass(*C.Node)) continue;
    if (auto* G = Cast<UMjGeomBase>(C.Node)) {
      if (!CapturePartial(*G, State.Geom, Pending)) Valid = false;
    } else if (auto* S = Cast<UMjSite>(C.Node)) {
      if (!CapturePartial(*S, State.Site, Pending)) Valid = false;
    }
  }
  for (const FMjOrderedChild& C : A::OrderedChildren(Class)) {
    if (!C.Node || !IsDefaultClass(*C.Node)) continue;
    FTemplates Nested = State;
    if (!CaptureClass<A>(*C.Node, Nested, Pending)) Valid = false;
  }
  return Valid;
}
template<class E> bool CaptureConcrete(E& Element, TArray<FSnapshot>& Pending) {
  TArray<FMjEffectiveLayer> Layers;
  MjEffectiveLayersOf(Element, Layers);
  TArray<double> Fromto, Size;
  EMjGeomType Type = EMjGeomType::sphere;
  FMjPosition3 Pos;
  bool GotFromto = false, GotSize = false, GotType = false, GotPos = false;
  for (const FMjEffectiveLayer& L : Layers) {
    WithSpatial(*L.Node, [&](const auto& P) {
      if (!GotFromto && P.Fromto.IsSet()) { Fromto = P.Fromto.GetValue(); GotFromto = true; }
      if (!GotSize && P.Size.IsSet()) { Size = P.Size.GetValue(); GotSize = true; }
      if (!GotType && P.Type.IsSet()) { Type = P.Type.GetValue(); GotType = true; }
      if (!GotPos && P.Pos.IsSet()) { Pos = P.Pos.GetValue(); GotPos = true; }
    });
  }
  if (Fromto.Num() != 6) return true;
  if (!AdmitsFromto(Type) || Pos.X != 0 || Pos.Y != 0 || Pos.Z != 0) return false;
  FSnapshot S;
  S.Node = &Element;
  S.ClearFromto = Element.Fromto.IsSet(); S.ClearSize = Element.Size.IsSet();
  S.ClearPos = Element.Pos.IsSet() && !S.ClearFromto;
  S.ClearQuat = Element.Quat.IsSet() && !S.ClearFromto;
  S.Pose = S.ClearFromto; S.Dimensions = S.ClearFromto || S.ClearSize;
  if (S.Dimensions && !MakeSpatial(Fromto, Size, S)) return false;
  Pending.Add(S);
  return true;
}
template<class A> bool CaptureTree(UMjNodeComponent& Node, bool InDefaults,
                                   TArray<FSnapshot>& Pending) {
  const bool Default = IsDefaultClass(Node);
  bool Valid = true;
  if (!InDefaults && !Default) {
    if (auto* G = Cast<UMjGeomBase>(&Node)) Valid = CaptureConcrete(*G, Pending);
    else if (auto* S = Cast<UMjSite>(&Node)) Valid = CaptureConcrete(*S, Pending);
  }
  for (const FMjOrderedChild& C : A::OrderedChildren(Node)) {
    if (C.Node && !CaptureTree<A>(*C.Node, InDefaults || Default, Pending)) Valid = false;
  }
  return Valid;
}
void Commit(const FSnapshot& S) {
  WithSpatial(*S.Node, [&](auto& E) {
    if (S.ClearFromto) E.Fromto.Reset();
    if (S.ClearSize) E.Size.Reset();
    if (S.ClearPos) E.Pos.Reset();
    if (S.ClearQuat) E.Quat.Reset();
    if (S.Pose) {
      E.Pos = FMjPosition3(S.Pos[0], S.Pos[1], S.Pos[2]);
      E.Quat = FMjQuatRot(S.Quat[0], S.Quat[1], S.Quat[2], S.Quat[3]);
    }
    if (S.Dimensions) E.MjSpatialHalfExtentsCm = S.HalfExtentsCm;
  });
}
} // namespace

template<class A> bool NormalizeImportedFromtoTree(UMjNodeComponent& Root) {
  FMjEffectiveScope Scope(Root);
  TArray<FSnapshot> Pending;
  FTemplates Main;
  bool Valid = true;
  for (const FMjOrderedChild& C : A::OrderedChildren(Root)) {
    if (C.Node && IsDefaultClass(*C.Node) && !CaptureClass<A>(*C.Node, Main, Pending)) Valid = false;
  }
  if (!CaptureTree<A>(Root, false, Pending)) Valid = false;
  if (Valid) for (const FSnapshot& S : Pending) Commit(S);
  return Valid;
}
template bool NormalizeImportedFromtoTree<FMjInstanceAdapter>(UMjNodeComponent&);
#if WITH_EDITOR
template bool NormalizeImportedFromtoTree<FMjScsAdapter>(UMjNodeComponent&);
#endif

bool MjSpatialSizeOverride(const UMjNodeComponent& Node, bool InDefault, TArray<double>& OutSize) {
  bool Has = false;
  OutSize.Reset();
  WithSpatial(Node, [&](const auto& E) {
    if (E.Size.IsSet()) return;
    FVector Half;
    TArray<FMjEffectiveLayer> Layers;
    if (InDefault) {
      if (!E.MjSpatialHalfExtentsCm.IsSet()) return;
      Half = E.MjSpatialHalfExtentsCm.GetValue();
    } else {
      MjEffectiveLayersOf(Node, Layers);
      for (const FMjEffectiveLayer& L : Layers) {
        if (L.Node->MjSpatialHalfExtentsCm.IsSet()) {
          Half = L.Node->MjSpatialHalfExtentsCm.GetValue();
          Has = true; break;
        }
      }
      if (!Has) return;
    }
    Has = true;
    EMjGeomType Type = EMjGeomType::sphere;
    bool GotType = false;
    if (InDefault) {
      if (E.Type.IsSet()) { Type = E.Type.GetValue(); GotType = true; }
    } else {
      for (const FMjEffectiveLayer& L : Layers) {
        WithSpatial(*L.Node, [&](const auto& P) {
          if (!GotType && P.Type.IsSet()) { Type = P.Type.GetValue(); GotType = true; }
        });
        if (GotType) break;
      }
    }
    if (GotType && AdmitsFromto(Type))
      OutSize = MjSizeFromScale(MjSizeShapeFor(Type), Half / 50.0);
  });
  return Has;
}
} // namespace urlab::spec
#endif // URLAB_MJ_GEN
