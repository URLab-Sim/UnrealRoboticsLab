#include "numeric.h"

#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>
#include <system_error>
#include <type_traits>

namespace ps::mjcf::io::num {
namespace {

bool IsSpace(char c) {
  return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' ||
         c == '\v';
}

// MuJoCo's ParseInfOrNan (xml_util.cc:53): a token of exactly "inf"/"nan"
// (optionally a single leading '-') is accepted case-insensitively, since
// std::from_chars does not parse them.
template <class T>
bool ParseInfOrNan(std::string_view s, T& out, bool& is_nan) {
  const char* str = s.data();
  std::size_t n = s.size();
  T sign = 1;
  if (n == 4 && str[0] == '-') {
    sign = -1;
    ++str;
    --n;
  }
  if (n != 3) {
    return false;
  }
  auto lc = [](char c) -> char {
    return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
  };
  if (lc(str[0]) == 'i' && lc(str[1]) == 'n' && lc(str[2]) == 'f') {
    out = sign * std::numeric_limits<T>::infinity();
    return true;
  }
  if (lc(str[0]) == 'n' && lc(str[1]) == 'a' && lc(str[2]) == 'n') {
    out = std::numeric_limits<T>::quiet_NaN();
    is_nan = true;
    return true;
  }
  return false;
}

}  // namespace

std::vector<std::string_view> Tokens(std::string_view s) {
  std::vector<std::string_view> out;
  std::size_t i = 0;
  const std::size_t n = s.size();
  while (i < n) {
    while (i < n && IsSpace(s[i])) ++i;
    std::size_t start = i;
    while (i < n && !IsSpace(s[i])) ++i;
    if (i > start) out.push_back(s.substr(start, i - start));
  }
  return out;
}

template <class T>
Status ParseInt(std::string_view tok, T& out) {
  if (tok.empty()) return Status::BadFormat;
  // std::from_chars does not accept a leading '+'; MuJoCo's strtod-based reader
  // does, and models in the wild write it (menagerie's go1 has pos="0 +0.04 0").
  if (tok.front() == '+') tok.remove_prefix(1);
  if (tok.empty()) return Status::BadFormat;
  const char* begin = tok.data();
  const char* end = begin + tok.size();
  T value{};
  auto [ptr, ec] = std::from_chars(begin, end, value);
  if (ec == std::errc::result_out_of_range) return Status::Overflow;
  if (ec != std::errc() || ptr != end) return Status::BadFormat;
  out = value;
  return Status::Ok;
}

template <class T>
Status ParseFloat(std::string_view tok, T& out, bool& is_nan) {
  if (tok.empty()) return Status::BadFormat;
  // See ParseInt: a leading '+' is accepted by MuJoCo and rejected by
  // std::from_chars. ParseInfOrNan below sees the stripped token too, so
  // "+inf" reads the same way "-inf" already did.
  if (tok.front() == '+') tok.remove_prefix(1);
  if (tok.empty()) return Status::BadFormat;
  const char* begin = tok.data();
  const char* end = begin + tok.size();
  T value{};
  auto [ptr, ec] = std::from_chars(begin, end, value);
  if (ec == std::errc() && ptr == end) {
    // Some std::from_chars implementations (e.g. MSVC) accept inf/nan spellings
    // directly, so flag NaN here as well as in the manual fallback below.
    if (std::isnan(value)) is_nan = true;
    out = value;
    return Status::Ok;
  }
  // std::from_chars rejects inf/nan spellings; MuJoCo recognizes them by hand.
  if (ParseInfOrNan<T>(tok, out, is_nan)) {
    return Status::Ok;
  }
  if (ec == std::errc::result_out_of_range) {
    // from_chars reports one range error for both directions. MuJoCo's
    // sscanf-based reader accepts underflow silently (the value becomes 0 or a
    // denormal), so only true overflow may fail. strtof/strtod distinguishes
    // the two: an underflowing token yields a magnitude <= the smallest normal.
    if (ptr == end) {
      std::string z(tok);
      char* endp = nullptr;
      T v;
      if constexpr (std::is_same_v<T, float>) {
        v = std::strtof(z.c_str(), &endp);
      } else {
        v = std::strtod(z.c_str(), &endp);
      }
      if (endp == z.c_str() + z.size() &&
          std::fabs(v) <= std::numeric_limits<T>::min()) {
        out = v;
        return Status::Ok;
      }
    }
    return Status::Overflow;
  }
  return Status::BadFormat;
}

MemStatus ParseMemory(std::string_view s, std::uint64_t& bytes) {
  // Trim surrounding whitespace; MuJoCo rejects any interior token (Size()).
  auto toks = Tokens(s);
  if (toks.empty()) return MemStatus::Unset;
  if (toks.size() != 1) return MemStatus::Bad;
  std::string_view t = toks[0];
  if (t == "-1") return MemStatus::Unset;
  if (!t.empty() && t[0] == '-') return MemStatus::Bad;

  const char* begin = t.data();
  const char* end = begin + t.size();
  std::uint64_t base = 0;
  auto [ptr, ec] = std::from_chars(begin, end, base);
  if (ec != std::errc()) return MemStatus::Bad;

  int shift = 0;
  if (ptr != end) {
    // Exactly one suffix character may follow the digits.
    if (end - ptr != 1) return MemStatus::Bad;
    switch (*ptr) {
      case 'K': case 'k': shift = 10; break;
      case 'M': case 'm': shift = 20; break;
      case 'G': case 'g': shift = 30; break;
      case 'T': case 't': shift = 40; break;
      case 'P': case 'p': shift = 50; break;
      case 'E': case 'e': shift = 60; break;
      default: return MemStatus::Bad;
    }
  }
  // Reject a shift that would overflow uint64.
  if (shift >= std::numeric_limits<std::uint64_t>::digits) return MemStatus::Bad;
  const std::uint64_t max_base =
      (std::numeric_limits<std::uint64_t>::max() >> shift);
  if (base > max_base) return MemStatus::Bad;
  bytes = base << shift;
  return MemStatus::Ok;
}

std::string FormatDouble(double v) {
  std::array<char, 32> buf{};
  auto [ptr, ec] = std::to_chars(buf.data(), buf.data() + buf.size(), v);
  if (ec != std::errc()) return "0";
  return std::string(buf.data(), ptr);
}

std::string FormatFloat(float v) {
  std::array<char, 32> buf{};
  auto [ptr, ec] = std::to_chars(buf.data(), buf.data() + buf.size(), v);
  if (ec != std::errc()) return "0";
  return std::string(buf.data(), ptr);
}

std::string FormatInt(std::int64_t v) {
  std::array<char, 24> buf{};
  auto [ptr, ec] = std::to_chars(buf.data(), buf.data() + buf.size(), v);
  if (ec != std::errc()) return "0";
  return std::string(buf.data(), ptr);
}

template Status ParseInt<std::int32_t>(std::string_view, std::int32_t&);
template Status ParseInt<std::uint64_t>(std::string_view, std::uint64_t&);
template Status ParseFloat<float>(std::string_view, float&, bool&);
template Status ParseFloat<double>(std::string_view, double&, bool&);

}  // namespace ps::mjcf::io::num
