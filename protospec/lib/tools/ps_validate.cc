// ps_validate: parse an MJCF file and run ProtoSpec three-tier validation.
//
// Contract (a corpus harness invokes this exactly so):
//   ps_validate <in.xml>
//   stdout: a machine-readable summary line
//     SUMMARY tier1_errors=A tier2_errors=B tier3_warnings=C
//   plus one "<tierN> <SEV> ..." line per diagnostic.
//   exit 0  -> parsed; zero tier-1/tier-2 errors (tier-3 warnings allowed)
//   exit 2  -> parsed; at least one tier-1/tier-2 error
//   exit 3  -> the file uses elements outside the supported families (skip)
//   exit 1  -> malformed input, missing file, or bad usage (diagnostics on stderr)
#include <cstdio>
#include <string>

#include "mjcf.h"
#include "validate.h"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: ps_validate <in.xml>\n");
    return 1;
  }

  ps::mjcf::io::ParseResult parse = ps::mjcf::io::ParseMjcfFile(argv[1]);
  if (!parse.ok()) {
    const bool skip = parse.unsupported_only();
    for (const auto& d : parse.errors) {
      std::fprintf(stderr, "%s\n", d.Render().c_str());
    }
    return skip ? 3 : 1;
  }

  std::vector<ps::Diagnostic> diags =
      ps::mjcf::validate::Validate(*parse.model);

  int t1 = 0, t2 = 0, t3 = 0;
  for (const auto& d : diags) {
    switch (d.tier) {
      case ps::mjcf::validate::Tier::Structural: ++t1; break;
      case ps::mjcf::validate::Tier::Referential: ++t2; break;
      case ps::mjcf::validate::Tier::Semantic: ++t3; break;
    }
  }

  std::printf("SUMMARY tier1_errors=%d tier2_errors=%d tier3_warnings=%d\n", t1,
              t2, t3);
  for (const auto& d : diags) {
    std::printf("%s\n", d.Render().c_str());
  }

  return (t1 + t2) > 0 ? 2 : 0;
}
