// Q-NUM: C-locale numeric parsing and formatting for MJCF attribute values.
//
// Mirrors MuJoCo's xml_util.cc number handling: whitespace-separated tokens,
// C-locale (std::from_chars is locale-independent by construction), explicit
// inf/-inf/nan recognition (std::from_chars does not accept them), integer
// overflow reported as an error, and the `memory` size attribute's K/M/G/T/P/E
// binary suffixes reduced to a canonical byte count. Formatting is shortest
// round-trip via std::to_chars (documented deviation from MuJoCo's fixed-
// precision writer; the differential harness accounts for it).
#ifndef PROTOSPEC_IO_NUMERIC_H
#define PROTOSPEC_IO_NUMERIC_H

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace ps::mjcf::io::num {

// Split on ASCII whitespace (space, tab, newline, cr, ff, vt), dropping empties.
std::vector<std::string_view> Tokens(std::string_view s);

// Per-token parse outcome.
enum class Status {
  Ok,
  BadFormat,  // not a number
  Overflow,   // out of the target type's range (integers) or magnitude too
              // large (floats; underflow is accepted as 0/denormal, sscanf
              // parity with MuJoCo)
};

// Parse one C-locale integer token into T, range-checked (T: int32_t/uint64_t).
template <class T>
Status ParseInt(std::string_view tok, T& out);

// Parse one C-locale floating token into T (float/double), recognizing
// inf/-inf/nan/-nan (case-insensitive). Sets is_nan when the value is NaN.
template <class T>
Status ParseFloat(std::string_view tok, T& out, bool& is_nan);

// Parse the `memory` attribute: an unsigned integer with an optional binary
// suffix {K,M,G,T,P,E} (1<<10 .. 1<<60), or the literal "-1" meaning unset.
enum class MemStatus { Ok, Unset, Bad };
MemStatus ParseMemory(std::string_view s, std::uint64_t& bytes);

// Shortest round-trip formatting. Integer-valued floats render without a
// fractional part (e.g. 1000.0 -> "1000"), matching MuJoCo's integer collapse.
std::string FormatDouble(double v);
std::string FormatFloat(float v);
std::string FormatInt(std::int64_t v);

}  // namespace ps::mjcf::io::num

#endif  // PROTOSPEC_IO_NUMERIC_H
