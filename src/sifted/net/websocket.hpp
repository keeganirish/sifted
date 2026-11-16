#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

#include "sifted/core/result.hpp"

// Minimal, read-only WebSocket publisher (RFC 6455) for track snapshots.
// Decision record: docs/adr/0006-networking.md.
//
// Clients are untrusted (threat model B3) and read-only: after the opening
// handshake the server only ever sends text frames holding the latest
// snapshot. From clients it accepts nothing but Ping (answered with Pong),
// Pong and Close; any data frame is refused with close code 1008 and the
// connection is closed. Limits: the handshake request is capped at
// max_request_bytes, client frames at 125 bytes (control frames only), the
// number of clients at max_clients (extra connections are closed at once).
//
// Slow clients never stall the engine: publish() only swaps a shared
// "latest snapshot" pointer. Each client holds at most one outgoing frame;
// when it finishes sending, it gets whatever is latest then, so a slow client
// skips snapshots rather than queueing them. A client whose socket makes no
// progress for send_stall_timeout_ms is disconnected.
//
// One server thread owns every socket (poll(2)); the only shared state is the
// latest snapshot, under a mutex, and atomic counters.
namespace sifted::net {

struct WebSocketConfig {
  std::string bind_address = "127.0.0.1";  // local only by default
  std::uint16_t port = 8765;               // 0: ephemeral (tests)
  std::size_t max_clients = 16;
  std::size_t max_request_bytes = 8 * 1024;
  std::size_t max_message_bytes = 8 * 1024 * 1024;  // larger snapshots are skipped
  int send_stall_timeout_ms = 10'000;
};

struct WebSocketStats {
  std::atomic<std::uint64_t> accepted{0};
  std::atomic<std::uint64_t> rejected_full{0};
  std::atomic<std::uint64_t> bad_handshakes{0};
  std::atomic<std::uint64_t> refused_client_data{0};
  std::atomic<std::uint64_t> stalled_dropped{0};
  std::atomic<std::uint64_t> messages_sent{0};
  std::atomic<std::uint64_t> oversize_skipped{0};
};

// ---- protocol pieces (pure functions, unit-tested) -------------------------

// Sec-WebSocket-Accept for a client's Sec-WebSocket-Key.
std::string websocket_accept_key(std::string_view client_key);

// Validates an opening handshake (everything up to and including the blank
// line) and returns the client's Sec-WebSocket-Key, or nullopt if it is not a
// well-formed RFC 6455 version-13 upgrade GET.
std::optional<std::string> parse_handshake(std::string_view request);

// One unmasked, final text frame (server -> client).
std::string encode_text_frame(std::string_view payload);

// The first client frame at the start of `buffer`, under the read-only
// policy: only final, masked control frames (Close, Ping, Pong) with at most
// 125 bytes of payload are accepted; anything else is Refused (the server
// answers with close code 1008). NeedMore: the frame is not complete yet.
// `consumed` bytes (0 unless Ping, Pong or Close) belong to the frame;
// `payload` is unmasked.
struct ClientFrame {
  enum class Kind : std::uint8_t { NeedMore, Refused, Close, Ping, Pong };
  Kind kind = Kind::NeedMore;
  std::size_t consumed = 0;
  std::string payload;
};
ClientFrame parse_client_frame(std::string_view buffer);

class WebSocketServer {
 public:
  static Result<std::unique_ptr<WebSocketServer>, std::string> start(const WebSocketConfig& cfg);
  ~WebSocketServer();
  WebSocketServer(const WebSocketServer&) = delete;
  WebSocketServer& operator=(const WebSocketServer&) = delete;

  std::uint16_t port() const noexcept { return port_; }
  // Thread-safe. The newest snapshot replaces any older one not yet sent.
  void publish(std::string text);
  void stop();
  std::size_t clients() const noexcept { return clients_.load(std::memory_order_relaxed); }
  const WebSocketStats& stats() const noexcept { return stats_; }

 private:
  WebSocketServer(const WebSocketConfig& cfg, int listen_fd, int wake_r, int wake_w,
                  std::uint16_t port);
  void loop();
  void wake() noexcept;

  WebSocketConfig cfg_;
  int listen_fd_;
  int wake_r_;
  int wake_w_;
  std::uint16_t port_;
  std::atomic<bool> stop_{false};
  std::atomic<std::size_t> clients_{0};
  WebSocketStats stats_;

  std::mutex mu_;  // guards latest_ and version_
  std::shared_ptr<const std::string> latest_;
  std::uint64_t version_ = 0;

  std::thread thread_;
};

}  // namespace sifted::net
