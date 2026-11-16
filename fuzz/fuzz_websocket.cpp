// libFuzzer target for the WebSocket publisher's parsers of client input
// (threat model B3: clients are untrusted): the opening handshake and the
// client frame stream, exactly as the server loop consumes them.
//
// The input is split at the first "\r\n\r\n": everything up to it is offered
// to parse_handshake (as the server does once it sees the blank line), and
// the rest is consumed as a client frame stream with parse_client_frame.
//
// Properties checked on every input:
//   1. never crashes, hangs or reads out of bounds (ASan/UBSan);
//   2. an accepted handshake yields a 24-character base64 key, and its
//      accept key is the 28-character base64 SHA-1;
//   3. frame parsing always makes progress or stops: Ping, Pong and Close
//      consume 6..131 bytes, never more than were offered; NeedMore and
//      Refused consume nothing; payloads are at most 125 bytes; parsing is
//      deterministic (a second parse of the same bytes agrees).
//
// Run:  mkdir -p build/fuzz-work-ws && build/fuzz/fuzz/fuzz_websocket
//       build/fuzz-work-ws fuzz/corpus_ws -max_total_time=60

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <string_view>

#include "sifted/net/websocket.hpp"

namespace {
void check(bool ok) {
  if (!ok) {
    std::abort();
  }
}
}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  const std::string_view input(reinterpret_cast<const char*>(data), size);  // NOLINT
  const std::size_t end = input.find("\r\n\r\n");
  const std::string_view request = end == std::string_view::npos ? input : input.substr(0, end + 4);
  if (const auto key = sifted::net::parse_handshake(request)) {
    check(key->size() == 24);
    check(sifted::net::websocket_accept_key(*key).size() == 28);
  }
  std::string_view frames =
      end == std::string_view::npos ? std::string_view{} : input.substr(end + 4);
  using Kind = sifted::net::ClientFrame::Kind;
  while (true) {
    const auto f = sifted::net::parse_client_frame(frames);
    const auto g = sifted::net::parse_client_frame(frames);
    check(f.kind == g.kind && f.consumed == g.consumed && f.payload == g.payload);
    if (f.kind == Kind::NeedMore || f.kind == Kind::Refused) {
      check(f.consumed == 0);
      break;
    }
    check(f.consumed >= 6 && f.consumed <= 131 && f.consumed <= frames.size());
    check(f.payload.size() == f.consumed - 6 && f.payload.size() <= 125);
    frames.remove_prefix(f.consumed);
    if (f.kind == Kind::Close) {
      break;  // the server stops reading after Close
    }
  }
  return 0;
}
