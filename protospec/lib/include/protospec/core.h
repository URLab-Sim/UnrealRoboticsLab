// Source location retained for MJCF include-expansion diagnostics.
#ifndef PROTOSPEC_CORE_H
#define PROTOSPEC_CORE_H

#include <string>

namespace ps {
struct SourceLoc {
  std::string file;
  int line = 0;
  friend bool operator==(const SourceLoc&, const SourceLoc&) = default;
};
}  // namespace ps

#endif  // PROTOSPEC_CORE_H
