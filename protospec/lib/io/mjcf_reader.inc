// MJCF reader: the profile-generic half. tinyxml2 document -> a document tree.
//
// Included by exactly one translation unit per emission profile, which then
// defines that profile's public entry points. It is a .inc rather than a header
// because its private signatures mention tinyxml2, which must not leak into a
// consumer's include path.
//
// The reader is table-driven by the generated xml_binding (attrs/children) and
// by the profile's Visit hook for typed field access. Hand code appears only at
// the quirks the tables cannot carry: numeric parsing, variable arity, and the
// strict support boundary -- all of which live in mjcf_common.h and compile once.
//
// Orientation and inertia canonicalization: orientation is stored as a single
// canonical quaternion and inertia as diaginertia + quat. The reader still
// ACCEPTS every authored spelling (quat/euler/axisangle/xyaxes/zaxis via the
// binding's input aliases; diaginertia/fullinertia), but folds them into the
// canonical field. Inertia has no angular context, so fullinertia is
// eigendecomposed inline at read. Orientation resolution IS deferred to parse
// end: MJCF admits <compiler> blocks anywhere (even after <worldbody>) and
// euler/axisangle depend on compiler.angle/eulerseq, so resolving mid-parse
// would make the result depend on document order. The reader records each
// authored orientation as a pending fold -- as a node plus the canonical quat
// field's schema id, never as a raw pointer into storage, so nothing writes past
// the profile's own presence plumbing -- and resolves them all in one pass over
// the effective, document-order-folded compiler context. Element-wins-atomic: an
// element's authored orientation always wins over a class-inherited alt spelling
// (a deliberate, documented divergence from MuJoCo's ReadAlternative precedence
// wart; no corpus witness).
//
// Angles are handled by form preservation, not read-time conversion: angle
// values are stored exactly as authored and the compiler's angle unit round
// trips verbatim, so MuJoCo performs every degree->radian conversion at compile
// time. This is necessary, not merely tidy: MuJoCo converts joint range only for
// LIMITED hinge/ball joints and ref/springref only for hinge -- conditions
// resolved per consuming joint at compile. A default class (data) can be
// shared by joints that resolve those conditions differently, so no single
// pre-converted stored value is correct.
//
// Presence: only authored attributes set fields; a missing attribute
// leaves the slot unauthored. Provenance: every element records
// {file, line}.

namespace ps::mjcf::io {

namespace common_ = ps::mjcf::io::common;

// A collected-but-unresolved orientation: an authored spelling awaiting the
// document-order-folded compiler context (parse-end resolution).
// `node` plus `field_id` names the canonical quat slot; `apply` recovers the
// node's concrete type, which is known where the record is made and nowhere
// else. `xml` stays valid for the whole parse.
struct PendingOrient {
  const XMLElement* xml;
  void* node;
  int field_id;
  void (*apply)(void* node, int field_id, const double* quat);
  ps::core::OrientKind kind;
  double raw[6];
};

// --------------------------------------------------------------------------- //
// Reader                                                                        //
// --------------------------------------------------------------------------- //
template <class P, class Factory>
class Reader : public common_::ReaderBase {
 public:
  using ReaderBase::ReaderBase;
  using Shp = typename P::Shape;

  // Read a concrete element from an XML node: attributes, the per-element quirk
  // hooks, then supported children.
  template <class E>
  void ReadElement(XMLElement* xml, E& out) {
    // Bump the nesting depth for the whole subtree read (RAII: every return path
    // below unwinds it). depth_ is 1 at the root <mujoco>, so a document nested
    // past kMaxElementDepth levels is rejected here, before recursing deeper.
    struct DepthScope {
      int& d;
      explicit DepthScope(int& d) : d(d) { ++d; }
      ~DepthScope() { --d; }
    } depth_scope{depth_};
    if (depth_ > kMaxElementDepth) {
      Err(xml, "element nesting depth exceeds the maximum of " +
                   std::to_string(kMaxElementDepth) + " at <" +
                   std::string(xml->Value()) + "> (depth " +
                   std::to_string(depth_) + ")");
      return;
    }

    const ElementType type = sdk::ElementTypeOf<P, E>;
    const ElementBinding& b = Bind(type);
    P::Ident::SetLoc(out, Loc(xml));
    CheckDuplicateName(xml, b.type);

    CheckUnknownAttributes(xml, b);
    AttrVisitor<E> av{this, xml, &b};
    P::Visit(out, av);

    // Collect the authored orientation (resolved at parse end against
    // the folded compiler context). Elements whose schema orientation is the
    // variant group only. The "orientation" resolver is dispatched here, not
    // through ApplyResolvers, because it defers to the document-order compiler
    // fold.
    if (common_::FoldsOrientation(b)) CollectOrientation(xml, out, b);
    // The mutual-exclusion check is a hardcoded quirk (not a schema
    // resolver); the fullinertia/diaginertia fold itself is the "inertia"
    // resolver, dispatched below off the binding metadata.
    if (type == ElementType::Inertial) InertialOrientExclusion(xml);
    // Schema-declared canonicalizations (resolver="..." in the schema, emitted
    // into AttrBinding::resolver) are dispatched off the generated metadata, not
    // a hardcoded per-element chain: adding a resolver in the schema wires it
    // here automatically, and a name with no registered function fails loudly
    // (DispatchResolver's exhaustive switch + the ResolverRegistryComplete test).
    ApplyResolvers(xml, out, b);
    // Presence constraints (exclusive/together/requires/oneof) come from the
    // schema through the generated reflection tables, so an element's
    // attribute-combination rules need no reader code at all.
    CheckConstraints(xml, b.type);
    // The remaining hooks are the quirks the constraint rows cannot state --
    // conditional exclusivity, ordering, value ranges, child uniqueness.
    if (type == ElementType::Size) MemoryFixup(xml, out, b);
    if (IsActuatorSpelling(type)) ActuatorTransmission(xml);
    if (type == ElementType::SensorContact) ContactSensorData(xml);
    if (type == ElementType::SensorPlugin) PluginSensorPairing(xml);
    if (HasConfigChildren(type)) ConfigKeyUnique(xml);

    ClassifyChildren(xml, b, out);
    ReadChildren(xml, b, out);

    // A material's `texture=` attribute is the RGB entry of the canonical
    // <layer> list. Folded after children are read so the mix-with-layers error
    // sees the authored layers, and the injected RGB layer is prepended (first)
    // exactly as MuJoCo slots mjTEXROLE_RGB.
    if constexpr (std::is_same_v<
                      std::decay_t<E>,
                      sdk::ElementOf<P, ElementType::Material>>) {
      MaterialTextureFixup(xml, out);
    }
  }

  // Parse-end orientation fold. Computes the effective compiler
  // context by folding the document's <compiler> blocks in document order (later
  // authored attributes win, matching MuJoCo's accumulate-into-one-spec), then
  // resolves every collected authored orientation against that single context --
  // so the result is document-order independent even though <compiler> may
  // appear anywhere. Defaults when unauthored: angle="degree", eulerseq="xyz".
  void ResolvePendingOrientations(const sdk::DocOf<P>& m) {
    using CompilerT = sdk::ElementOf<P, ElementType::Compiler>;
    const int angle_id =
        sdk::internal::FieldIdByName(ElementType::Compiler, "angle");
    const int eulerseq_id =
        sdk::internal::FieldIdByName(ElementType::Compiler, "eulerseq");
    ps::core::OrientContext ctx;  // degree=true, eulerseq="xyz"
    P::Tree::template ForEachChildOfType<CompilerT>(m, [&](const CompilerT& c) {
      const int angle = sdk::internal::GetEnumField<P>(c, angle_id, -1);
      if (angle >= 0)
        ctx.degree = (static_cast<AngleUnit>(angle) == AngleUnit::degree);
      std::string seq;
      if (sdk::internal::GetStrField<P>(c, eulerseq_id, seq)) ctx.eulerseq = seq;
    });
    for (const PendingOrient& p : pending_orient_) {
      const std::array<double, 4> q =
          ps::core::ResolveOrientation(p.kind, p.raw, ctx);
      p.apply(p.node, p.field_id, q.data());
    }
  }

 private:
  // --- schema-derived element classification ------------------------------- //
  // These replace hand-written type lists. The eleven actuator spellings share
  // the common transmission attributes and so the same target-exclusivity quirk;
  // the schema already groups them as one name namespace.
  static bool IsActuatorSpelling(ElementType t) {
    return sdk::detail::InUnionNamespace("ActuatorAny", t);
  }

  // Elements that hold a <config> key/value list (a plugin reference or
  // instance): MuJoCo rejects duplicate config keys.
  static bool HasConfigChildren(ElementType t) {
    const reflect::ElementDescriptor& d = reflect::Describe(t);
    for (std::size_t i = 0; i < d.child_count; ++i)
      if (d.children[i].target == "Config") return true;
    return false;
  }

  // ---- attribute field reading -------------------------------------------- //
  template <class E>
  struct AttrVisitor {
    Reader* r;
    XMLElement* xml;
    const ElementBinding* b;

    template <class T>
    void field(int id, const char*, T& value) {
      for (std::size_t i = 0; i < b->attr_count; ++i) {
        if (b->attrs[i].field_id == id) {
          // Resolved (canonicalized) fields -- quat/diaginertia/area/... -- are
          // set by the parse-end orientation fold / the schema-declared resolver
          // registry (ApplyResolvers), not the plain attr path. A non-empty
          // resolver name marks them.
          if (!b->attrs[i].resolver.empty()) return;
          r->ReadAttr(xml, b->type, b->attrs[i], value);
          return;
        }
      }
    }
    template <class C>
    void child(int, const char*, C&) {}
    template <class C>
    void union_child(int, const char*, C&) {}
  };

  template <class T>
  void ReadAttr(XMLElement* xml, ElementType type, const AttrBinding& ab,
                T& slot) {
    if constexpr (std::is_const_v<T>) {
      (void)slot;
    } else {
      std::string attr(ab.attr);
      const char* raw =
          ab.element_text ? xml->GetText() : xml->Attribute(attr.c_str());
      if (!raw) return;
      std::string_view text(raw);
      using I = typename Shp::template inner_t<std::decay_t<T>>;
      I tmp{};
      if (ReadInner<I>(xml, type, ab, text, tmp))
        Shp::Author(slot, std::move(tmp));
    }
  }

  template <class Inner>
  bool ReadInner(XMLElement* xml, ElementType type, const AttrBinding& ab,
                 std::string_view text, Inner& out) {
    constexpr sdk::Shape kind = Shp::template kind_v<Inner>;
    if constexpr (kind == sdk::Shape::Ref) {
      out = Shp::template RefMake<Inner>(text);
      return true;
    } else if constexpr (kind == sdk::Shape::Str) {
      out = Shp::template StrMake<Inner>(text);
      return true;
    } else if constexpr (kind == sdk::Shape::Bool) {
      // The <flag> element spells its booleans enable/disable (MuJoCo
      // enable_map); every other boolean attribute is true/false (bool_map).
      const char* yes = type == ElementType::Flag ? "enable" : "true";
      const char* no = type == ElementType::Flag ? "disable" : "false";
      if (text == yes) {
        out = true;
      } else if (text == no) {
        out = false;
      } else {
        Err(xml, "invalid keyword '" + std::string(text) + "' for attribute '" +
                     std::string(ab.attr) + "' (expected " + yes + " or " + no +
                     ")");
        return false;
      }
      return true;
    } else if constexpr (kind == sdk::Shape::Enum) {
      if (Shp::template EnumFromMjcf<Inner>(text, out)) return true;
      Err(xml, "invalid keyword '" + std::string(text) + "' for attribute '" +
                   std::string(ab.attr) + "'");
      return false;
    } else if constexpr (kind == sdk::Shape::Fixed) {
      return ReadFixed<Inner>(xml, ab, text, out);
    } else if constexpr (kind == sdk::Shape::Range) {
      return ReadRange<Inner>(xml, ab, text, out);
    } else if constexpr (kind == sdk::Shape::EnumList) {
      return ReadEnumList<Inner>(xml, ab, text, out);
    } else if constexpr (kind == sdk::Shape::RefList) {
      return ReadRefList<Inner>(xml, ab, text, out);
    } else if constexpr (kind == sdk::Shape::Unbounded) {
      return ReadVector<Inner>(xml, ab, text, out);
    } else if constexpr (kind == sdk::Shape::Scalar) {
      return ReadNumberN(xml, ab, text, &out, 1, 1) == 1;
    } else {
      return false;
    }
  }

  // The arity the binding declares, checked against a value count the storage
  // type could not constrain.
  bool CheckDeclaredArity(XMLElement* xml, const AttrBinding& ab,
                          std::size_t got) {
    const std::size_t lo = static_cast<std::size_t>(ab.arity_min);
    const std::size_t hi = static_cast<std::size_t>(ab.arity_max);
    if (ab.arity == reflect::ArityKind::Fixed && got != lo) {
      Err(xml, "attribute '" + std::string(ab.attr) + "' expects " +
                   std::to_string(lo) + " values, got " +
                   std::to_string(got));
      return false;
    }
    if (ab.arity == reflect::ArityKind::Range && got > hi) {
      Err(xml, "attribute '" + std::string(ab.attr) + "' expects at most " +
                   std::to_string(hi) + " values, got " + std::to_string(got));
      return false;
    }
    return true;
  }

  // Fixed-arity: exactly N values. The scalars go to the
  // profile's fixed adapter as a whole array in MJCF component order; the
  // adapter, not this code, decides where each component lands.
  template <class Inner>
  bool ReadFixed(XMLElement* xml, const AttrBinding& ab, std::string_view text,
                 Inner& out) {
    using F = typename Shp::template fixed<Inner>;
    using S = typename F::scalar;
    constexpr std::size_t N = F::size;
    S buf[N]{};
    int got = ReadNumberN(xml, ab, text, buf, static_cast<int>(N),
                          static_cast<int>(N));
    if (got != static_cast<int>(N)) return false;
    out = F::Make(buf);
    return true;
  }

  // Range arity: up to capacity values, fewer is fine (record the filled count),
  // more is an error.
  template <class Inner>
  bool ReadRange(XMLElement* xml, const AttrBinding& ab, std::string_view text,
                 Inner& out) {
    using Q = typename Shp::template seq<Inner>;
    using S = typename Q::scalar;
    constexpr std::size_t N = Q::max_size;
    S buf[N]{};
    int got = ReadNumberN(xml, ab, text, buf, 0, static_cast<int>(N));
    if (got < 0) return false;
    out = Q::Make(buf, static_cast<std::size_t>(got));
    return true;
  }

  // Unbounded numeric list.
  //
  // A profile whose storage does not encode arity -- one growable container for
  // every list length -- reaches this for fixed and range attributes too, so the
  // bounds are enforced from the binding rather than from the storage type. For
  // a profile that does encode arity the check is unreachable: ReadFixed and
  // ReadRange have already taken those attributes.
  template <class Inner>
  bool ReadVector(XMLElement* xml, const AttrBinding& ab, std::string_view text,
                  Inner& out) {
    using Q = typename Shp::template seq<Inner>;
    using S = typename Q::scalar;
    auto toks = num::Tokens(text);
    if (!CheckDeclaredArity(xml, ab, toks.size())) return false;
    std::vector<S> vals(toks.size());
    for (std::size_t i = 0; i < toks.size(); ++i) {
      if (!ParseScalar(xml, ab, toks[i], vals[i])) return false;
    }
    out = Q::Make(vals.data(), vals.size());
    return true;
  }

  // A space-separated keyword set (Q for MapValues).
  template <class Inner>
  bool ReadEnumList(XMLElement* xml, const AttrBinding& ab,
                    std::string_view text, Inner& out) {
    using Q = typename Shp::template seq<Inner>;
    using S = typename Q::scalar;
    std::unordered_set<std::string> seen;
    std::vector<S> vals;
    for (std::string_view tok : num::Tokens(text)) {
      std::string key(tok);
      if (!seen.insert(key).second) {
        Err(xml, "duplicate keyword '" + key + "' in attribute '" +
                     std::string(ab.attr) + "'");
        return false;
      }
      S v{};
      if (!Shp::template EnumFromMjcf<S>(tok, v)) {
        Err(xml, "invalid keyword '" + key + "' for attribute '" +
                     std::string(ab.attr) + "'");
        return false;
      }
      vals.push_back(v);
    }
    // Keyword-set canonicalization: a keyword set is an order-insensitive
    // bitmask (MapValues); store it in enum-declaration order so the canonical
    // form is deterministic. The enum's declaration index is also the MJCF map
    // order -- for the contact sensor, whose reader demands that exact order,
    // the strict-order check runs separately on the raw input before this
    // normalization.
    std::sort(vals.begin(), vals.end(),
              [](const S& a, const S& b) {
                return Shp::template EnumIndex<S>(a) <
                       Shp::template EnumIndex<S>(b);
              });
    out = Q::Make(vals.data(), vals.size());
    return true;
  }

  // ref<T>[]: a space-separated list of element names (<flex body="b1 b2">). A
  // name cannot contain whitespace on this wire, so tokenizing is exact.
  template <class Inner>
  bool ReadRefList(XMLElement*, const AttrBinding&, std::string_view text,
                   Inner& out) {
    std::vector<std::string_view> names;
    for (std::string_view tok : num::Tokens(text)) names.push_back(tok);
    out = Shp::template reflist<Inner>::Make(names);
    return true;
  }

  // ---- Orientation collection --------------------------------------------- //
  // Recover the element's concrete type here, where it is known, and hand the
  // parse-end fold a plain function pointer plus the canonical field's schema
  // id -- so the deferred write goes through the profile's own presence plumbing
  // instead of a raw pointer into storage that a component profile could not
  // hand out.
  template <class E>
  static void ApplyQuat(void* node, int field_id, const double* q) {
    const double v[4] = {q[0], q[1], q[2], q[3]};
    sdk::internal::SetFixedField<P, 4>(*static_cast<E*>(node), field_id, v);
  }

  // Collect the authored orientation spelling for later resolution against the
  // folded compiler context (parse end). Multiple-specifier and zero-quaternion
  // errors are reader-level (no compiler context needed) and are reported here,
  // verbatim as MuJoCo does. A no-op for elements with no canonical quat field.
  template <class E>
  void CollectOrientation(XMLElement* xml, E& out, const ElementBinding& b) {
    const int field_id = common_::ResolverFieldId(b, "orientation");
    if (field_id < 0) return;
    int count = Has(xml, "quat") + Has(xml, "axisangle") + Has(xml, "xyaxes") +
                Has(xml, "zaxis") + Has(xml, "euler");
    if (count == 0) return;
    if (count > 1) {
      Err(xml, "multiple orientation specifiers are not allowed");
      return;
    }
    PendingOrient p{};
    p.xml = xml;
    p.node = static_cast<void*>(&out);
    p.field_id = field_id;
    p.apply = &ApplyQuat<E>;
    if (Has(xml, "quat")) {
      if (!ReadDoubleArr(xml, "quat", 4, p.raw)) return;
      if (p.raw[0] == 0 && p.raw[1] == 0 && p.raw[2] == 0 && p.raw[3] == 0) {
        Err(xml, "zero quaternion is not allowed");
        return;
      }
      p.kind = ps::core::OrientKind::Quat;
    } else if (Has(xml, "axisangle")) {
      if (!ReadDoubleArr(xml, "axisangle", 4, p.raw)) return;
      p.kind = ps::core::OrientKind::AxisAngle;
    } else if (Has(xml, "xyaxes")) {
      if (!ReadDoubleArr(xml, "xyaxes", 6, p.raw)) return;
      p.kind = ps::core::OrientKind::XYAxes;
    } else if (Has(xml, "zaxis")) {
      if (!ReadDoubleArr(xml, "zaxis", 3, p.raw)) return;
      p.kind = ps::core::OrientKind::ZAxis;
    } else {  // euler
      if (!ReadDoubleArr(xml, "euler", 3, p.raw)) return;
      p.kind = ps::core::OrientKind::Euler;
    }
    pending_orient_.push_back(p);
  }

  // ---- schema-declared resolvers ------------------------------------------ //
  // Each writes its canonical field by schema id, so one body serves every
  // profile and every element type the schema points at it.

  // Diaginertia stores as authored; fullinertia is eigendecomposed
  // into diaginertia + the inertial-frame quaternion (iquat) inline -- inertia
  // has no angular context, so no parse-end deferral is needed. Mutual exclusion
  // mirrors MuJoCo.
  template <class E>
  void ResolveInertiaAttrs(XMLElement* xml, E& out, const ElementBinding& b) {
    const int diag_id = common_::ResolverFieldId(b, "inertia");
    const int quat_id = common_::ResolverFieldId(b, "orientation");
    if (diag_id < 0) return;
    bool diag = Has(xml, "diaginertia");
    bool full = Has(xml, "fullinertia");
    if (diag && full) {
      Err(xml, "diaginertia and fullinertia cannot both be specified");
      return;
    }
    if (diag) {
      double d[3]{};
      if (!ReadDoubleArr(xml, "diaginertia", 3, d)) return;
      sdk::internal::SetFixedField<P, 3>(out, diag_id, d);
    } else if (full) {
      double f[6]{};
      if (!ReadDoubleArr(xml, "fullinertia", 6, f)) return;
      double diagv[3]{}, q[4]{};
      if (const char* e = ps::core::FullInertiaToDiag(f, diagv, q)) {
        Err(xml, e);
        return;
      }
      sdk::internal::SetFixedField<P, 3>(out, diag_id, diagv);
      // fullinertia carries the inertial frame in its eigenvectors (the inertial
      // orientation must be absent -- InertialOrientExclusion).
      if (quat_id >= 0) sdk::internal::SetFixedField<P, 4>(out, quat_id, q);
    }
  }

  // springlength: read one or two values; a lone value duplicates into the
  // second slot, exactly as MuJoCo. Canonical storage is the two-value pair.
  template <class E>
  void ResolveSpringlength(XMLElement* xml, E& out, const ElementBinding& b) {
    const int id = common_::ResolverFieldId(b, "springlength");
    if (id < 0) return;
    if (!Has(xml, "springlength")) return;
    double v[2]{};
    AttrBinding ab{};
    ab.attr = "springlength";
    int got = ReadNumberN(xml, ab, xml->Attribute("springlength"), v, 1, 2);
    if (got < 1) return;
    if (got == 1) v[1] = v[0];
    sdk::internal::SetSeqField<P>(out, id, v, 2);
  }

  // light type: `directional` (legacy bool) and `type` (enum) both fill the
  // canonical `type`, mutually exclusive. directional="true" -> directional,
  // "false" -> spot.
  template <class E>
  void ResolveLightType(XMLElement* xml, E& out, const ElementBinding& b) {
    const int id = common_::ResolverFieldId(b, "lighttype");
    if (id < 0) return;
    bool has_dir = Has(xml, "directional");
    bool has_type = Has(xml, "type");
    if (has_dir && has_type) {
      Err(xml, "type and directional cannot both be defined");
      return;
    }
    if (has_dir) {
      bool d{};
      AttrBinding ab{};
      ab.attr = "directional";
      if (ReadInner<bool>(xml, ElementType::Light, ab,
                          std::string_view(xml->Attribute("directional")), d)) {
        sdk::internal::SetEnumFieldByKeyword<P>(out, id,
                                                d ? "directional" : "spot");
      }
    } else if (has_type) {
      const char* kw = xml->Attribute("type");
      if (!sdk::internal::SetEnumFieldByKeyword<P>(out, id, kw)) {
        Err(xml, "invalid keyword '" + std::string(kw) +
                     "' for attribute 'type'");
      }
    }
  }

  // cylinder area: `area` and `diameter` both fill gainprm[0]; a non-negative
  // diameter overrides area with pi/4 d^2 (mjs_setToCylinder). Canonical storage
  // is `area`. Element-wins-atomic (a class diameter beating an element area is
  // not emulated).
  template <class E>
  void ResolveCylinderArea(XMLElement* xml, E& out, const ElementBinding& b) {
    const int id = common_::ResolverFieldId(b, "cylinderarea");
    if (id < 0) return;
    // mjPI verbatim (this module is MuJoCo-free) so the fold is bit-identical to
    // mjs_setToCylinder's `mjPI / 4 * diameter*diameter`.
    constexpr double kPi = 3.14159265358979323846;
    double diameter = -1;
    bool has_diam = Has(xml, "diameter");
    if (has_diam && !ReadDoubleArr(xml, "diameter", 1, &diameter)) return;
    if (has_diam && diameter >= 0) {
      sdk::internal::SetScalarField<P>(out, id, kPi / 4 * diameter * diameter);
      return;
    }
    if (Has(xml, "area")) {
      double area{};
      if (ReadDoubleArr(xml, "area", 1, &area))
        sdk::internal::SetScalarField<P>(out, id, area);
    }
  }

  // Route one resolver to its canonicalization. Exhaustive over Resolver, so
  // adding an enumerator without a canonicalization does not compile.
  template <class E>
  void DispatchResolver(common_::Resolver r, XMLElement* xml, E& out,
                        const ElementBinding& b) {
    switch (r) {
      // Owned by their own phases: Orientation by the document-order compiler
      // fold (CollectOrientation), MaterialLayer by MaterialTextureFixup after
      // children (and it never rides an AttrBinding, so ApplyResolvers never
      // passes it).
      case common_::Resolver::Orientation:
      case common_::Resolver::MaterialLayer:
      case common_::Resolver::None:      return;
      case common_::Resolver::Inertia:      ResolveInertiaAttrs(xml, out, b); return;
      case common_::Resolver::Springlength: ResolveSpringlength(xml, out, b); return;
      case common_::Resolver::LightType:    ResolveLightType(xml, out, b); return;
      case common_::Resolver::CylinderArea: ResolveCylinderArea(xml, out, b); return;
    }
  }

  // Consult the generated binding metadata: canonicalize each field whose
  // AttrBinding carries a resolver name.
  template <class E>
  void ApplyResolvers(XMLElement* xml, E& out, const ElementBinding& b) {
    for (std::size_t i = 0; i < b.attr_count; ++i) {
      DispatchResolver(common_::LookupResolver(b.attrs[i].resolver), xml, out, b);
    }
  }

  // A material's `texture=` attribute is a read-only input alias (no field); it
  // prepends an RGB layer, and mixing it with authored <layer> children is an
  // error.
  template <class E>
  void MaterialTextureFixup(XMLElement* xml, E& out) {
    using LayerT = sdk::ElementOf<P, ElementType::MaterialLayer>;
    const char* tex = xml->Attribute("texture");
    if (!tex) return;
    if (P::Tree::template FirstChildOfType<LayerT>(out) != nullptr) {
      Err(xml,
          "a material with a texture attribute cannot have layer sub-elements");
      return;
    }
    LayerT& layer = Factory::template Create<LayerT>(out, /*index=*/0);
    P::Ident::SetLoc(layer, P::Ident::Loc(out));
    sdk::internal::SetStrField<P>(
        layer, sdk::internal::FieldIdByName(ElementType::MaterialLayer, "role"),
        "rgb");
    auto slot = P::Ref::SlotAt(
        layer, sdk::internal::FieldIdByName(ElementType::MaterialLayer, "texture"));
    if (slot) slot.Set(P::Str::View(P::Str::FromUtf8(tex)));
  }

  // The `memory` size is authored as an integer with an optional
  // K/M/G/T/P/E binary suffix; the reader stores the canonical byte count (as a
  // string, the field's storage). "-1" or absent means unset.
  template <class E>
  void MemoryFixup(XMLElement* xml, E& out, const ElementBinding& b) {
    (void)b;
    const int id = sdk::internal::FieldIdByName(ElementType::Size, "memory");
    if (id < 0) return;
    const char* raw = xml->Attribute("memory");
    if (!raw) {
      sdk::internal::ClearField<P>(out, id);
      return;
    }
    std::uint64_t bytes = 0;
    switch (num::ParseMemory(raw, bytes)) {
      case num::MemStatus::Ok:
        sdk::internal::SetStrField<P>(out, id, std::to_string(bytes));
        break;
      case num::MemStatus::Unset:
        sdk::internal::ClearField<P>(out, id);
        break;
      case num::MemStatus::Bad:
        Err(xml,
            "unsigned integer with an optional suffix {K,M,G,T,P,E} is expected "
            "in attribute 'memory'");
        break;
    }
  }

  // ---- children ----------------------------------------------------------- //
  // The contextual XML tag of the element type an admissible child slot holds:
  // an owned child list carries its own tag in the binding (a `<frame>` alias, a
  // `<joint>` inside `<composite>`), while a union list routes by each member's
  // own tag.
  static std::string_view SlotTag(const ElementBinding& b, int slot,
                                  ElementType child) {
    if (slot < 0 || slot >= static_cast<int>(b.child_count))
      return Bind(child).tag;
    return b.children[slot].is_union ? Bind(child).tag : b.children[slot].tag;
  }

  // Diagnose child tags that map to no child slot (malformed, an error) or to an
  // unsupported child type (skip signal). Union members are classified by member
  // tag, so an unsupported member of a supported union still signals skip rather
  // than a false unknown.
  template <class E>
  void ClassifyChildren(XMLElement* xml, const ElementBinding& b, E& out) {
    std::vector<std::pair<std::string_view, bool>> claims;
    P::Tree::ForEachChildSlot(out, [&](int slot, auto tag) {
      using T = typename decltype(tag)::type;
      const ElementType t = sdk::ElementTypeOf<P, T>;
      claims.push_back({SlotTag(b, slot, t), common_::IsSupported(t)});
    });
    for (XMLElement* c = xml->FirstChildElement(); c;
         c = c->NextSiblingElement()) {
      std::string_view tag = c->Value();
      bool known = false, supported = false;
      for (const auto& [t, s] : claims) {
        if (t == tag) {
          known = true;
          supported = s;
          break;
        }
      }
      if (!known) {
        Err(c, "unknown element '" + std::string(tag) + "' in '" +
                   std::string(b.tag) + "'");
      } else if (!supported) {
        Unsupported(c, std::string(tag));
      }
    }
  }

  // Read the supported children, one storage slot at a time. Within a slot the
  // XML children are taken in document order, which is what reproduces MuJoCo's
  // id assignment for an interleaved union list; children in different slots do
  // not interleave, exactly as the schema's child declarations describe.
  template <class E>
  void ReadChildren(XMLElement* xml, const ElementBinding& b, E& out) {
    int slot_count = 0;
    P::Tree::ForEachChildSlot(
        out, [&](int slot, auto) { slot_count = std::max(slot_count, slot + 1); });
    for (int slot = 0; slot < slot_count; ++slot) {
      for (XMLElement* c = xml->FirstChildElement(); c;
           c = c->NextSiblingElement()) {
        std::string_view tag = c->Value();
        P::Tree::ForEachChildSlot(out, [&](int s, auto type_tag) {
          if (s != slot) return;
          using T = typename decltype(type_tag)::type;
          const ElementType t = sdk::ElementTypeOf<P, T>;
          if (!common_::IsSupported(t)) return;
          if (SlotTag(b, slot, t) != tag) return;
          T& child = Factory::template Create<T>(out, sdk::kAppend);
          ReadElement(c, child);
        });
      }
    }
  }

  std::vector<PendingOrient> pending_orient_;
};

// eulerseq must be length 3; MuJoCo enforces this at read.
template <class P, class Factory>
void ValidateEulerseq(Reader<P, Factory>& r, XMLElement* root,
                      const sdk::DocOf<P>& m) {
  using CompilerT = sdk::ElementOf<P, ElementType::Compiler>;
  const int id = sdk::internal::FieldIdByName(ElementType::Compiler, "eulerseq");
  P::Tree::template ForEachChildOfType<CompilerT>(m, [&](const CompilerT& c) {
    std::string seq;
    if (sdk::internal::GetStrField<P>(c, id, seq) && seq.size() != 3) {
      XMLElement* ce = root->FirstChildElement("compiler");
      r.Err(ce ? ce : root, "euler format must have length 3");
    }
  });
}

// Drive a parsed document through the include pre-pass and the table-driven
// reader. `model_dir` is where relative includes resolve first (the top-level
// file's directory); empty for string input with no real path.
template <class P, class Factory>
ParseResultOf<P> ReadDocumentT(XMLDocument& doc, const std::string& filename,
                               const std::string& model_dir,
                               const ParseOptions& opts) {
  ParseResultOf<P> result;

  XMLElement* root = doc.RootElement();
  if (!root || std::strcmp(root->Value(), "mujoco") != 0) {
    ps::Diagnostic d;
    d.source = "parse";
    d.kind = ps::Diagnostic::Kind::MalformedInput;
    d.loc = ps::SourceLoc{filename, root ? root->GetLineNum() : 0};
    d.message = "root element must be <mujoco>";
    result.errors.push_back(std::move(d));
    return result;
  }

  // Expand <include> before anything else, so the reader sees a flat document.
  // Include failures are structural errors and abort the parse (MuJoCo throws).
  IncludeResult inc =
      ExpandIncludes(doc, model_dir, filename, opts.allow_external_includes);
  if (!inc.errors.empty()) {
    result.errors = std::move(inc.errors);
    return result;
  }

  Reader<P, Factory> reader(filename, result.errors, result.warnings,
                            &inc.provenance);

  auto model = Factory::CreateRoot();
  reader.ReadElement(root, *model);
  // Fold the effective compiler context and resolve every collected
  // orientation now that the whole (include-flattened) document is in the tree.
  reader.ResolvePendingOrientations(*model);
  ValidateEulerseq(reader, root, *model);
  ValidateDefaultClassesT<P>(*model, result.errors);

  result.model = std::move(model);
  return result;
}

template <class P, class Factory>
ParseResultOf<P> ParseMjcfStringT(const std::string& xml,
                                  const std::string& filename,
                                  const ParseOptions& opts) {
  ParseResultOf<P> result;
  XMLDocument doc;
  if (doc.Parse(xml.c_str(), xml.size()) != tinyxml2::XML_SUCCESS) {
    ps::Diagnostic d;
    d.source = "parse";
    d.kind = ps::Diagnostic::Kind::MalformedInput;
    d.loc = ps::SourceLoc{filename, doc.ErrorLineNum()};
    const char* what = doc.ErrorStr();
    d.message =
        std::string("XML parse error: ") + (what ? what : "malformed document");
    result.errors.push_back(std::move(d));
    return result;
  }
  // A pseudo-name like "<string>" yields an empty parent path, disabling
  // relative include resolution; a real path resolves includes beside it.
  std::string dir = std::filesystem::path(filename).parent_path().string();
  return ReadDocumentT<P, Factory>(doc, filename, dir, opts);
}

template <class P, class Factory>
ParseResultOf<P> ParseMjcfFileT(const std::string& path,
                                const ParseOptions& opts) {
  ParseResultOf<P> result;
  XMLDocument doc;
  if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
    ps::Diagnostic d;
    d.source = "parse";
    d.kind = ps::Diagnostic::Kind::MalformedInput;
    d.loc = ps::SourceLoc{path, doc.ErrorLineNum()};
    const char* what = doc.ErrorStr();
    d.message = std::string("cannot read file: ") + (what ? what : "load error");
    result.errors.push_back(std::move(d));
    return result;
  }
  std::string dir = std::filesystem::path(path).parent_path().string();
  return ReadDocumentT<P, Factory>(doc, path, dir, opts);
}

}  // namespace ps::mjcf::io
