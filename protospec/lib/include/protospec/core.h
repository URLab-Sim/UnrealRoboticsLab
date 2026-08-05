// ProtoSpec runtime support (handwritten, schema-independent).
//
// This header holds the small non-generated core the generated object model
// builds on: the presence-tracking optional alias, a fixed-capacity inline
// vector for variable-arity attributes, structured source provenance, the typed
// reference wrapper, the creation-serial source, and the deep container
// equality/clone helpers the generated code calls. Everything schema-shaped is
// generated (see protospec/lib/generated); nothing here knows about MuJoCo.
#ifndef PROTOSPEC_CORE_H
#define PROTOSPEC_CORE_H

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ps {

// --- Presence (DR-1) ------------------------------------------------------ //
// Every defaultable field is presence-tracked: `has_value()` == "authored".
template <class T>
using opt = std::optional<T>;

// --- Variable-arity storage (Q-ARITY) ------------------------------------- //
// A fixed-capacity vector carrying its filled count, for range-arity MJCF
// attributes (`size` 0..3, `friction` 1..3, `gear` 0..6, ...). Capacity is the
// arity upper bound; the filled count is the number of authored values. Range
// bounds are a validation concern, not enforced by the type -- but the capacity
// N is a hard storage invariant: exceeding it is a programmer error upstream of
// the type, so push_back/resize abort (with a diagnostic) rather than write out
// of bounds. operator[] stays unchecked, as std::array.
template <class T, std::size_t N>
class InlineVec {
 public:
  using value_type = T;
  static constexpr std::size_t capacity = N;

  InlineVec() = default;
  InlineVec(std::initializer_list<T> init) {
    for (const T& v : init) push_back(v);
  }

  std::size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }
  void clear() { size_ = 0; }

  void push_back(const T& v) {
    if (size_ >= N) Overflow(size_ + 1);
    data_[size_++] = v;
  }
  void resize(std::size_t n) {
    if (n > N) Overflow(n);
    size_ = n;
  }

  T& operator[](std::size_t i) { return data_[i]; }
  const T& operator[](std::size_t i) const { return data_[i]; }

  T* begin() { return data_.data(); }
  T* end() { return data_.data() + size_; }
  const T* begin() const { return data_.data(); }
  const T* end() const { return data_.data() + size_; }

  friend bool operator==(const InlineVec& a, const InlineVec& b) {
    if (a.size_ != b.size_) return false;
    for (std::size_t i = 0; i < a.size_; ++i) {
      if (!(a.data_[i] == b.data_[i])) return false;
    }
    return true;
  }
  friend bool operator!=(const InlineVec& a, const InlineVec& b) {
    return !(a == b);
  }

 private:
  [[noreturn]] static void Overflow(std::size_t requested) {
    std::fprintf(stderr,
                 "protospec: InlineVec overflow: capacity %zu, requested %zu\n",
                 N, requested);
    std::abort();
  }

  std::array<T, N> data_{};
  std::size_t size_ = 0;
};

// --- Reserved name prefix (DR-10) ----------------------------------------- //
// Auto-generated binding names (`_ps:<family>:<serial>`, injected only into
// compile-XML, never into saved documents) own this prefix. Authored names must
// never start with it: SDK SetName/Rename reject such names and validation
// flags them, so an auto-name can never collide with an authored one.
inline constexpr std::string_view kReservedNamePrefix = "_ps:";

// --- Provenance (DR-9) ---------------------------------------------------- //
// mjSpec's `info` string, structured. Populated by the reader through include
// expansion; empty for programmatically built elements. Not part of semantic
// equality (it is provenance, not content).
struct SourceLoc {
  std::string file;
  int line = 0;

  friend bool operator==(const SourceLoc&, const SourceLoc&) = default;
};

// --- Creation serials (DR-10 / DR-11) ------------------------------------- //
// Each element is stamped with a monotonic serial at construction. Serials are
// process-unique (hence Model-unique: a superset guarantee), never reassigned
// or reused, so they are stable across tree edits. Auto-generated binding names
// (`_ps:geom:<serial>`) derive from them, so an unnamed element's compiled
// state migrates correctly after sibling insertions/deletions. Clone mints
// fresh serials (below), so a clone attached into the same Model never
// duplicates a serial.
namespace detail {
inline std::uint64_t next_serial() {
  static std::atomic<std::uint64_t> counter{0};
  return counter.fetch_add(1, std::memory_order_relaxed) + 1;
}
}  // namespace detail

// --- Typed references (DR-8) ---------------------------------------------- //
// Storage is the name string (that is what MJCF is); the template parameter is
// a phantom recording the target element (or union) type so validation, rename,
// and resolve stay type-aware without storing tree pointers.
template <class Target>
struct Ref {
  std::string name;

  Ref() = default;
  explicit Ref(std::string n) : name(std::move(n)) {}

  bool empty() const { return name.empty(); }

  friend bool operator==(const Ref& a, const Ref& b) {
    return a.name == b.name;
  }
  friend bool operator!=(const Ref& a, const Ref& b) { return !(a == b); }
};

// --- Deep container equality/clone helpers -------------------------------- //
// Child lists are `std::vector<std::unique_ptr<Child>>` (DR-2, stable identity).
// These compare/copy them by pointed-to value. The element `operator==` and
// `Clone` these dispatch to are generated; instantiation happens in the
// translation unit that has the full element definitions.
template <class T>
bool PtrVecEq(const std::vector<std::unique_ptr<T>>& a,
              const std::vector<std::unique_ptr<T>>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const bool ha = static_cast<bool>(a[i]);
    const bool hb = static_cast<bool>(b[i]);
    if (ha != hb) return false;
    if (ha && !(*a[i] == *b[i])) return false;
  }
  return true;
}

template <class T>
std::vector<std::unique_ptr<T>> PtrVecClone(
    const std::vector<std::unique_ptr<T>>& src) {
  std::vector<std::unique_ptr<T>> out;
  out.reserve(src.size());
  for (const auto& p : src) out.push_back(p ? Clone(*p) : nullptr);
  return out;
}

}  // namespace ps

#endif  // PROTOSPEC_CORE_H
