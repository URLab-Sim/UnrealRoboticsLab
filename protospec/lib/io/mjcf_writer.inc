// MJCF writer: the profile-generic half. A document tree -> canonical MJCF,
// driven by the same generated xml_binding tables and Visit hook as the reader.
//
// Included by exactly one translation unit per emission profile. Deterministic
// output: 2-space indent, attributes in schema field order, children in schema
// child order, exactly the authored fields (no default diffing). Angle
// values are emitted verbatim in their authored unit and the compiler's angle
// attribute round trips as read (form preservation); MuJoCo applies the
// degree->radian conversion at compile, per consuming element.
//
// Numeric formatting is shortest round-trip via std::to_chars (numeric.cc). This
// deviates from MuJoCo's fixed-precision writer; the differential harness
// accounts for the difference when comparing our output to MuJoCo's.

namespace ps::mjcf::io {

inline std::string Escape(std::string_view s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      default: out += c;
    }
  }
  return out;
}

template <class S>
std::string FormatScalar(S v) {
  if constexpr (std::is_same_v<S, double>) {
    return num::FormatDouble(v);
  } else if constexpr (std::is_same_v<S, float>) {
    return num::FormatFloat(v);
  } else {
    return num::FormatInt(static_cast<std::int64_t>(v));
  }
}

template <class S>
std::string JoinScalars(const S* data, std::size_t n) {
  std::string out;
  for (std::size_t i = 0; i < n; ++i) {
    if (i) out += ' ';
    out += FormatScalar(data[i]);
  }
  return out;
}

inline std::string Indent(int depth) { return std::string(2 * depth, ' '); }

inline bool ContainsSpace(std::string_view s) {
  for (char c : s) {
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' ||
        c == '\v') {
      return true;
    }
  }
  return false;
}

// Format an authored field value into an attribute string, or nullopt to skip
// (empty variable-length values are omitted, matching a non-authored field).
template <class P, class Inner>
std::optional<std::string> FormatValue(const Inner& v, bool keyword_set) {
  using Shp = typename P::Shape;
  constexpr sdk::Shape kind = Shp::template kind_v<Inner>;
  (void)keyword_set;
  if constexpr (kind == sdk::Shape::Ref) {
    // `auto`, not std::string_view: a profile whose names are not UTF-8 has no
    // borrowable view over one and returns an owning string. Binding a view
    // here would dangle at the end of the declaration.
    const auto n = Shp::template RefGet<Inner>(v);
    const std::string_view name(n);
    if (name.empty()) return std::nullopt;
    return Escape(name);
  } else if constexpr (kind == sdk::Shape::Str) {
    return Escape(Shp::template StrGet<Inner>(v));
  } else if constexpr (kind == sdk::Shape::Bool) {
    return std::string(v ? "true" : "false");
  } else if constexpr (kind == sdk::Shape::Enum) {
    return std::string(Shp::template EnumToMjcf<Inner>(v));
  } else if constexpr (kind == sdk::Shape::Fixed) {
    using F = typename Shp::template fixed<Inner>;
    typename F::scalar buf[F::size]{};
    F::Load(v, buf);
    return JoinScalars(buf, F::size);
  } else if constexpr (kind == sdk::Shape::Range) {
    using Q = typename Shp::template seq<Inner>;
    const std::size_t n = Q::Size(v);
    if (n == 0) return std::nullopt;
    typename Q::scalar buf[Q::max_size]{};
    Q::Load(v, buf);
    return JoinScalars(buf, n);
  } else if constexpr (kind == sdk::Shape::RefList) {
    using R = typename Shp::template reflist<Inner>;
    const std::size_t n = R::Size(v);
    if (n == 0) return std::nullopt;
    // ref<T>[]: names joined by single spaces (the wire form they came from).
    std::string out;
    for (std::size_t i = 0; i < n; ++i) {
      if (i) out += ' ';
      out += Escape(R::Get(v, i));
    }
    return out;
  } else if constexpr (kind == sdk::Shape::EnumList) {
    using Q = typename Shp::template seq<Inner>;
    const std::size_t n = Q::Size(v);
    if (n == 0) return std::nullopt;
    std::vector<typename Q::scalar> buf(n);
    Q::Load(v, buf.data());
    std::string out;
    for (std::size_t i = 0; i < n; ++i) {
      if (i) out += ' ';
      out += std::string(Shp::template EnumToMjcf<typename Q::scalar>(buf[i]));
    }
    return out;
  } else if constexpr (kind == sdk::Shape::Unbounded) {
    using Q = typename Shp::template seq<Inner>;
    const std::size_t n = Q::Size(v);
    if (n == 0) return std::nullopt;
    std::vector<typename Q::scalar> buf(n);
    Q::Load(v, buf.data());
    return JoinScalars(buf.data(), n);
  } else if constexpr (kind == sdk::Shape::Scalar) {
    return FormatScalar(v);
  } else {
    // No storage shape reaches here; the branch exists only so the dispatcher
    // type-checks for every field type.
    return std::nullopt;
  }
}

template <class P, class E>
void WriteElement(const E& e, std::string_view tag, int depth, std::string& out,
                  const AutoNames* names, std::vector<ps::Diagnostic>* errors);

// Append ` name="value"` unless value is skipped.
inline void AppendAttr(std::string& attrs, std::string_view name,
                       const std::optional<std::string>& value) {
  if (!value) return;
  attrs += ' ';
  attrs += name;
  attrs += "=\"";
  attrs += *value;
  attrs += '"';
}

template <class P, class E>
struct WriterVisitor {
  using Shp = typename P::Shape;
  const ElementBinding* b;
  ElementType type;
  std::string* attrs;
  const ps::SourceLoc* loc;
  std::vector<ps::Diagnostic>* errors;

  template <class T>
  void field(int id, const char*, T& value) {
    for (std::size_t i = 0; i < b->attr_count; ++i) {
      if (b->attrs[i].field_id == id) {
        WriteAttrField(b->attrs[i], value);
        return;
      }
    }
  }

  template <class T>
  void WriteAttrField(const AttrBinding& ab, const T& slot) {
    using I = typename Shp::template inner_t<std::decay_t<T>>;
    if (!Shp::IsSet(slot)) return;
    const I& inner = Shp::Read(slot);
    // The <flag> element spells booleans enable/disable, not true/false.
    if constexpr (Shp::template kind_v<I> == sdk::Shape::Bool) {
      if (type == ElementType::Flag) {
        AppendAttr(*attrs, ab.attr, std::string(inner ? "enable" : "disable"));
        return;
      }
    }
    // Ref lists are space-joined on the wire, so a name containing whitespace is
    // unrepresentable (it would re-read as multiple refs).
    if constexpr (Shp::template kind_v<I> == sdk::Shape::RefList) {
      using R = typename Shp::template reflist<I>;
      for (std::size_t i = 0; i < R::Size(inner); ++i) {
        const std::string_view n = R::Get(inner, i);
        if (ContainsSpace(n)) {
          ps::Diagnostic d;
          d.source = "write";
          d.kind = ps::Diagnostic::Kind::MalformedInput;
          d.message = "ref list attribute '" + std::string(ab.attr) + "' on <" +
                      std::string(b->tag) + ">: name '" + std::string(n) +
                      "' contains whitespace and cannot be written "
                      "(space-joined list)";
          if (loc) d.loc = *loc;
          errors->push_back(std::move(d));
        }
      }
    }
    AppendAttr(*attrs, ab.attr, FormatValue<P, I>(inner, ab.keyword_set));
  }

  template <class C>
  void child(int, const char*, C&) {}
  template <class C>
  void union_child(int, const char*, C&) {}
};

template <class P, class E>
void WriteElement(const E& e, std::string_view tag, int depth, std::string& out,
                  const AutoNames* names, std::vector<ps::Diagnostic>* errors) {
  const ElementType type = sdk::ElementTypeOf<P, E>;
  const ElementBinding& b = Bind(type);
  std::string attrs;
  std::string children;
  const ps::SourceLoc loc = P::Ident::Loc(e);
  WriterVisitor<P, E> v{&b, type, &attrs, &loc, errors};
  P::Visit(e, v);

  // Children, one storage slot at a time in schema child order; within a slot,
  // document order. An owned child list carries its contextual tag in the
  // binding (a `<frame>` alias, a `<joint>` inside `<composite>`), while a union
  // list routes each member by its own tag.
  P::Tree::ForEachChildAt(e, [&](int slot, const auto& c) {
    using T = std::decay_t<decltype(c)>;
    const ElementType ct = sdk::ElementTypeOf<P, T>;
    std::string_view ctag = (slot >= 0 && slot < static_cast<int>(b.child_count) &&
                             !b.children[slot].is_union)
                                ? b.children[slot].tag
                                : Bind(ct).tag;
    WriteElement<P>(c, ctag, depth + 1, children, names, errors);
  });

  // Auto-name injection: an unnamed bindable element whose serial the
  // bridge recorded is emitted with its reserved name. The tree is untouched;
  // the name exists only here in the emitted text. Prepended so it reads first.
  if (names != nullptr) {
    auto it = names->find(P::Ident::Serial(e));
    if (it != names->end()) {
      std::string name_attr = " name=\"";
      name_attr += Escape(it->second);
      name_attr += '"';
      attrs.insert(0, name_attr);
    }
  }

  out += Indent(depth);
  out += '<';
  out += tag;
  out += attrs;
  if (children.empty()) {
    out += "/>\n";
  } else {
    out += ">\n";
    out += children;
    out += Indent(depth);
    out += "</";
    out += tag;
    out += ">\n";
  }
}

template <class P>
std::string WriteMjcfT(const sdk::DocOf<P>& model, const AutoNames* names,
                       std::vector<ps::Diagnostic>* errors) {
  std::string out;
  std::vector<ps::Diagnostic> local;
  WriteElement<P>(model, Bind(sdk::ElementTypeOf<P, sdk::DocOf<P>>).tag, 0, out,
                  names, &local);
  if (!local.empty()) {
    if (errors) {
      errors->insert(errors->end(), std::make_move_iterator(local.begin()),
                     std::make_move_iterator(local.end()));
    }
    return std::string();
  }
  return out;
}

}  // namespace ps::mjcf::io
