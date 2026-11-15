// Minimal WebSocket publisher: handshake, framing, and a loopback client.

#include "sifted/net/websocket.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <string>
#include <thread>

#include "sifted/net/sha1.hpp"

namespace sifted::net {
namespace {

const std::string kRequest =
    "GET /tracks HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: keep-alive, "
    "Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n";

int connect_to(std::uint16_t port) {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(port);
  inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
  EXPECT_EQ(::connect(fd, reinterpret_cast<const sockaddr*>(&a), sizeof a), 0);  // NOLINT
  return fd;
}

// Reads until `pred(buffer)` or 2 s pass.
template <class Pred>
std::string read_until(int fd, Pred pred) {
  std::string buf;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!pred(buf) && std::chrono::steady_clock::now() < deadline) {
    pollfd p{fd, POLLIN, 0};
    if (::poll(&p, 1, 50) > 0) {
      char tmp[4096];
      const ssize_t n = ::recv(fd, tmp, sizeof tmp, 0);
      if (n <= 0) {
        break;
      }
      buf.append(tmp, static_cast<std::size_t>(n));
    }
  }
  return buf;
}

}  // namespace

TEST(Sha1, KnownVectors) {
  auto hex = [](const std::array<std::uint8_t, 20>& d) {
    std::string s;
    for (auto b : d) {
      static const char* k = "0123456789abcdef";
      s += k[b >> 4];
      s += k[b & 15];
    }
    return s;
  };
  const std::string abc = "abc";
  EXPECT_EQ(hex(sha1({reinterpret_cast<const std::uint8_t*>(abc.data()), abc.size()})),
            "a9993e364706816aba3e25717850c26c9cd0d89d");
  EXPECT_EQ(hex(sha1({})), "da39a3ee5e6b4b0d3255bfef95601890afd80709");
  const std::string long_msg(1000, 'a');
  EXPECT_EQ(hex(sha1({reinterpret_cast<const std::uint8_t*>(long_msg.data()), long_msg.size()})),
            "291e9a6c66994949b57ba5e650361e98fc36b1ba");
}

TEST(WebSocket, AcceptKeyMatchesRfc6455Example) {
  EXPECT_EQ(websocket_accept_key("dGhlIHNhbXBsZSBub25jZQ=="), "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");
}

TEST(WebSocket, HandshakeParserIsStrict) {
  EXPECT_EQ(parse_handshake(kRequest), "dGhlIHNhbXBsZSBub25jZQ==");
  auto without = [](const std::string& what) {
    std::string r = kRequest;
    r.erase(r.find(what), r.find("\r\n", r.find(what)) - r.find(what) + 2);
    return r;
  };
  EXPECT_FALSE(parse_handshake(without("Upgrade: websocket")).has_value());
  EXPECT_FALSE(parse_handshake(without("Sec-WebSocket-Version")).has_value());
  EXPECT_FALSE(parse_handshake(without("Sec-WebSocket-Key")).has_value());
  std::string post = kRequest;
  post.replace(0, 3, "PUT");
  EXPECT_FALSE(parse_handshake(post).has_value());
  EXPECT_FALSE(parse_handshake(kRequest.substr(0, kRequest.size() - 2)).has_value());
  std::string v8 = kRequest;
  v8.replace(v8.find("Version: 13"), 11, "Version: 8");
  EXPECT_FALSE(parse_handshake(v8).has_value());
}

TEST(WebSocket, TextFrameLengths) {
  EXPECT_EQ(encode_text_frame("hi"), std::string("\x81\x02hi", 4));
  const std::string mid(300, 'x');
  const std::string f = encode_text_frame(mid);
  EXPECT_EQ(static_cast<unsigned char>(f[1]), 126);
  EXPECT_EQ(f.size(), 4 + 300U);
  const std::string big(70000, 'y');
  EXPECT_EQ(static_cast<unsigned char>(encode_text_frame(big)[1]), 127);
}

TEST(WebSocket, ServesLatestSnapshotAndRefusesClientData) {
  WebSocketConfig cfg;
  cfg.port = 0;
  cfg.max_clients = 1;
  auto started = WebSocketServer::start(cfg);
  ASSERT_TRUE(started.ok()) << started.error();
  std::unique_ptr<WebSocketServer> server = std::move(started.value());

  const int fd = connect_to(server->port());
  ASSERT_EQ(::send(fd, kRequest.data(), kRequest.size(), 0), static_cast<ssize_t>(kRequest.size()));
  const std::string hs =
      read_until(fd, [](const std::string& b) { return b.find("\r\n\r\n") != b.npos; });
  EXPECT_EQ(hs.rfind("HTTP/1.1 101", 0), 0U) << hs;
  EXPECT_NE(hs.find("s3pPLMBiTxaQ9kYGzzhZRbK+xOo="), std::string::npos);

  server->publish("{\"seq\":1}");
  const std::string msg = read_until(fd, [](const std::string& b) { return b.size() >= 11; });
  EXPECT_EQ(msg, std::string("\x81\x09{\"seq\":1}", 11));

  // A second client is over the limit and is closed straight away.
  const int extra = connect_to(server->port());
  const std::string none = read_until(extra, [](const std::string&) { return false; });
  EXPECT_TRUE(none.empty());
  ::close(extra);

  // Clients are read-only: a (masked) text frame gets close code 1008.
  const unsigned char text[] = {0x81, 0x82, 1, 2, 3, 4, 'h' ^ 1, 'i' ^ 2};
  ASSERT_EQ(::send(fd, text, sizeof text, 0), static_cast<ssize_t>(sizeof text));
  const std::string closing = read_until(fd, [](const std::string& b) { return b.size() >= 4; });
  ASSERT_GE(closing.size(), 4U);
  EXPECT_EQ(static_cast<unsigned char>(closing[0]), 0x88);
  EXPECT_EQ((static_cast<unsigned char>(closing[2]) << 8) | static_cast<unsigned char>(closing[3]),
            1008);
  ::close(fd);
  EXPECT_EQ(server->stats().refused_client_data.load(), 1U);
  EXPECT_EQ(server->stats().rejected_full.load(), 1U);
  server->stop();
}

}  // namespace sifted::net
