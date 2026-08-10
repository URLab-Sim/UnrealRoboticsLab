// ps_roundtrip: read an MJCF file and print WriteMjcf(ParseMjcf(in)) to stdout.
//
// Contract (stable; a differential harness invokes this exactly so):
//   ps_roundtrip <in.xml>
//   exit 0  -> stdout carries the round-tripped MJCF
//   exit 3  -> the file uses elements outside the supported families (skip
//              signal); the unsupported tags are listed on stderr
//   exit 1  -> any other error (malformed input, missing file, bad usage);
//              the diagnostics are on stderr
#include <cstdio>
#include <string>

#include "mjcf.h"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: ps_roundtrip <in.xml>\n");
    return 1;
  }

  ps::mjcf::io::ParseResult result = ps::mjcf::io::ParseMjcfFile(argv[1]);

  if (result.ok()) {
    std::string xml = ps::mjcf::io::WriteMjcf(*result.model);
    std::fwrite(xml.data(), 1, xml.size(), stdout);
    return 0;
  }

  // Unsupported-only failures are the skip signal (exit 3), distinct from
  // malformed input (exit 1).
  const bool skip = result.unsupported_only();
  for (const auto& d : result.errors) {
    std::fprintf(stderr, "%s\n", d.Render().c_str());
  }
  return skip ? 3 : 1;
}
