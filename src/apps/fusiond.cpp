// sifted_fusiond: the fusion service.
//
// PLACEHOLDER: parses --version/--help only. The real daemon (UDP ingest ->
// engine -> WebSocket publisher, plus --record / --replay) arrives in
// milestone 2; see docs/design.md.

#include <cstdio>
#include <string_view>

#include "sifted/version.hpp"

int main(int argc, char** argv) {
  const std::string_view arg = argc > 1 ? argv[1] : "--help";
  if (arg == "--version") {
    std::printf("sifted_fusiond %s\n", sifted::kVersion);
    return 0;
  }
  const bool help = arg == "--help" || arg == "-h";
  std::fprintf(help ? stdout : stderr,
               "usage: sifted_fusiond [--version | --help]\n"
               "(the fusion service is not implemented yet)\n");
  return help ? 0 : 2;
}
