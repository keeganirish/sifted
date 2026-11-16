#include "sifted/net/websocket.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <vector>

#include "sifted/net/sha1.hpp"
#include "sifted/net/socket_compat.hpp"

namespace sifted::net {
namespace {

constexpr std::string_view kGuid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
constexpr std::size_t kMaxClientFrame = 2 + 4 + 125;  // header + mask + control payload

std::string lower(std::string_view s) {
  std::string out(s);
  for (char& c : out) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return out;
}

std::string_view trim(std::string_view s) {
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
    s.remove_prefix(1);
  }
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) {
    s.remove_suffix(1);
  }
  return s;
}

bool set_nonblocking(int fd) {
  const int flags = ::fcntl(fd, F_GETFL, 0);
  return flags >= 0 && ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

std::int64_t now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::string close_frame(std::uint16_t code) {
  std::string f;
  f += static_cast<char>(0x88);
  f += static_cast<char>(2);
  f += static_cast<char>(code >> 8);
  f += static_cast<char>(code & 0xFF);
  return f;
}

struct Client {
  int fd = -1;
  enum class State { Handshake, Open, Closing } state = State::Handshake;
  std::string in;
  std::string out;
  std::size_t out_off = 0;
  std::uint64_t sent_version = 0;
  std::int64_t last_progress_ms = 0;
};

}  // namespace

std::string websocket_accept_key(std::string_view client_key) {
  std::string s(client_key);
  s += kGuid;
  const auto digest =
      sha1(std::span<const std::uint8_t>(reinterpret_cast<const std::uint8_t*>(s.data()),  // NOLINT
                                         s.size()));
  return base64_encode(digest);
}

std::optional<std::string> parse_handshake(std::string_view request) {
  const std::size_t end = request.find("\r\n\r\n");
  if (end == std::string_view::npos) {
    return std::nullopt;
  }
  std::string_view head = request.substr(0, end + 2);
  const std::size_t first_eol = head.find("\r\n");
  const std::string_view request_line = head.substr(0, first_eol);
  if (request_line.substr(0, 4) != "GET " || request_line.size() < 14 ||
      request_line.substr(request_line.size() - 9) != " HTTP/1.1") {
    return std::nullopt;
  }
  head.remove_prefix(first_eol + 2);
  bool upgrade = false;
  bool connection = false;
  bool version = false;
  std::optional<std::string> key;
  while (!head.empty()) {
    const std::size_t eol = head.find("\r\n");
    const std::string_view line = head.substr(0, eol);
    head.remove_prefix(eol + 2);
    const std::size_t colon = line.find(':');
    if (colon == std::string_view::npos) {
      return std::nullopt;
    }
    const std::string name = lower(trim(line.substr(0, colon)));
    const std::string_view value = trim(line.substr(colon + 1));
    if (name == "upgrade") {
      upgrade = lower(value) == "websocket";
    } else if (name == "connection") {
      connection = lower(value).find("upgrade") != std::string::npos;
    } else if (name == "sec-websocket-version") {
      version = value == "13";
    } else if (name == "sec-websocket-key") {
      // 16 random bytes, base64: 24 characters ending in "==".
      const bool ok = value.size() == 24 && value.substr(22) == "==" &&
                      std::all_of(value.begin(), value.end() - 2, [](char c) {
                        return std::isalnum(static_cast<unsigned char>(c)) || c == '+' || c == '/';
                      });
      if (!ok || key) {
        return std::nullopt;
      }
      key = std::string(value);
    }
  }
  if (!upgrade || !connection || !version || !key) {
    return std::nullopt;
  }
  return key;
}

ClientFrame parse_client_frame(std::string_view buffer) {
  ClientFrame f;
  if (buffer.size() < 2) {
    return f;
  }
  const auto b0 = static_cast<std::uint8_t>(buffer[0]);
  const auto b1 = static_cast<std::uint8_t>(buffer[1]);
  const std::uint8_t opcode = b0 & 0x0FU;
  const bool masked = (b1 & 0x80U) != 0;
  const std::size_t len = b1 & 0x7FU;
  const bool control = opcode == 0x8 || opcode == 0x9 || opcode == 0xA;
  if (!control || !masked || len > 125 || (b0 & 0x80U) == 0) {
    f.kind = ClientFrame::Kind::Refused;
    return f;
  }
  if (buffer.size() < 6 + len) {
    return f;
  }
  f.payload.assign(buffer.substr(6, len));
  for (std::size_t k = 0; k < len; ++k) {
    f.payload[k] = static_cast<char>(f.payload[k] ^ buffer[2 + (k % 4)]);
  }
  f.consumed = 6 + len;
  f.kind = opcode == 0x8   ? ClientFrame::Kind::Close
           : opcode == 0x9 ? ClientFrame::Kind::Ping
                           : ClientFrame::Kind::Pong;
  return f;
}

std::string encode_text_frame(std::string_view payload) {
  std::string f;
  f.reserve(payload.size() + 10);
  f += static_cast<char>(0x81);  // FIN + text
  const std::uint64_t n = payload.size();
  if (n < 126) {
    f += static_cast<char>(n);
  } else if (n <= 0xFFFF) {
    f += static_cast<char>(126);
    f += static_cast<char>((n >> 8) & 0xFF);
    f += static_cast<char>(n & 0xFF);
  } else {
    f += static_cast<char>(127);
    for (int i = 7; i >= 0; --i) {
      f += static_cast<char>((n >> (8 * i)) & 0xFF);
    }
  }
  f.append(payload);
  return f;
}

Result<std::unique_ptr<WebSocketServer>, std::string> WebSocketServer::start(
    const WebSocketConfig& cfg) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(cfg.port);
  if (inet_pton(AF_INET, cfg.bind_address.c_str(), &addr.sin_addr) != 1) {
    return std::string("invalid IPv4 address: ") + cfg.bind_address;
  }
  const int fd = compat::open_socket(AF_INET, SOCK_STREAM);
  if (fd < 0) {
    return std::string("socket: ") + std::strerror(errno);
  }
  const int one = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof addr) != 0 ||  // NOLINT
      ::listen(fd, 16) != 0 || !set_nonblocking(fd)) {
    const std::string err = std::string("websocket listen on ") + cfg.bind_address + ":" +
                            std::to_string(cfg.port) + ": " + std::strerror(errno);
    ::close(fd);
    return err;
  }
  socklen_t len = sizeof addr;
  ::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len);  // NOLINT
  int pipefd[2];
  if (::pipe(pipefd) != 0 || !set_nonblocking(pipefd[0]) || !set_nonblocking(pipefd[1])) {
    ::close(fd);
    return std::string("pipe: ") + std::strerror(errno);
  }
  std::unique_ptr<WebSocketServer> s(
      new WebSocketServer(cfg, fd, pipefd[0], pipefd[1], ntohs(addr.sin_port)));
  s->thread_ = std::thread([p = s.get()] { p->loop(); });
  return s;
}

WebSocketServer::WebSocketServer(const WebSocketConfig& cfg, int listen_fd, int wake_r, int wake_w,
                                 std::uint16_t port)
    : cfg_(cfg), listen_fd_(listen_fd), wake_r_(wake_r), wake_w_(wake_w), port_(port) {}

WebSocketServer::~WebSocketServer() {
  stop();
  ::close(listen_fd_);
  ::close(wake_r_);
  ::close(wake_w_);
}

void WebSocketServer::wake() noexcept {
  const char b = 1;
  [[maybe_unused]] const ssize_t n = ::write(wake_w_, &b, 1);  // full pipe: already awake
}

void WebSocketServer::publish(std::string text) {
  auto p = std::make_shared<const std::string>(std::move(text));
  {
    const std::lock_guard<std::mutex> lock(mu_);
    latest_ = std::move(p);
    ++version_;
  }
  wake();
}

void WebSocketServer::stop() {
  if (!stop_.exchange(true)) {
    wake();
  }
  if (thread_.joinable()) {
    thread_.join();
  }
}

void WebSocketServer::loop() {
  std::vector<Client> clients;
  std::vector<pollfd> fds;
  auto drop = [&](Client& c) {
    if (c.fd >= 0) {
      ::close(c.fd);
      c.fd = -1;
    }
  };
  while (!stop_.load()) {
    fds.clear();
    fds.push_back({wake_r_, POLLIN, 0});
    fds.push_back({listen_fd_, POLLIN, 0});
    for (const Client& c : clients) {
      const short ev = static_cast<short>(POLLIN | (c.out_off < c.out.size() ? POLLOUT : 0));
      fds.push_back({c.fd, ev, 0});
    }
    ::poll(fds.data(), fds.size(), 200);
    if ((fds[0].revents & POLLIN) != 0) {
      char buf[64];
      while (::read(wake_r_, buf, sizeof buf) > 0) {
      }
    }
    if ((fds[1].revents & POLLIN) != 0) {
      while (true) {
        const int cfd = compat::accept_nonblocking(listen_fd_);
        if (cfd < 0) {
          break;
        }
        if (clients.size() >= cfg_.max_clients) {
          ::close(cfd);
          stats_.rejected_full.fetch_add(1);
          continue;
        }
        Client c;
        c.fd = cfd;
        c.last_progress_ms = now_ms();
        clients.push_back(std::move(c));
        stats_.accepted.fetch_add(1);
      }
    }

    std::shared_ptr<const std::string> latest;
    std::uint64_t version = 0;
    {
      const std::lock_guard<std::mutex> lock(mu_);
      latest = latest_;
      version = version_;
    }

    const std::int64_t now = now_ms();
    for (std::size_t i = 0; i < clients.size(); ++i) {
      Client& c = clients[i];
      const short rev = i + 2 < fds.size() && fds[i + 2].fd == c.fd ? fds[i + 2].revents : 0;
      if ((rev & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        drop(c);
        continue;
      }
      if ((rev & POLLIN) != 0) {
        char buf[1024];
        const ssize_t n = ::recv(c.fd, buf, sizeof buf, 0);
        if (n <= 0) {
          drop(c);
          continue;
        }
        c.in.append(buf, static_cast<std::size_t>(n));
      }
      if (c.state == Client::State::Handshake && !c.in.empty()) {
        if (c.in.find("\r\n\r\n") != std::string::npos) {
          const auto key = parse_handshake(c.in);
          c.in.clear();
          if (key) {
            c.out =
                "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: "
                "Upgrade\r\nSec-WebSocket-Accept: " +
                websocket_accept_key(*key) + "\r\n\r\n";
            c.state = Client::State::Open;
          } else {
            stats_.bad_handshakes.fetch_add(1);
            c.out = "HTTP/1.1 400 Bad Request\r\nConnection: close\r\nContent-Length: 0\r\n\r\n";
            c.state = Client::State::Closing;
          }
          c.out_off = 0;
        } else if (c.in.size() > cfg_.max_request_bytes) {
          stats_.bad_handshakes.fetch_add(1);
          drop(c);
          continue;
        }
      } else if (c.state == Client::State::Open) {
        // Client frames: control frames only, masked, <= 125 bytes payload.
        while (true) {
          const ClientFrame f = parse_client_frame(c.in);
          if (f.kind == ClientFrame::Kind::NeedMore) {
            break;
          }
          if (f.kind == ClientFrame::Kind::Refused) {
            stats_.refused_client_data.fetch_add(1);
            c.out = close_frame(1008);  // policy violation: read-only stream
            c.out_off = 0;
            c.state = Client::State::Closing;
            c.in.clear();
            break;
          }
          c.in.erase(0, f.consumed);
          if (f.kind == ClientFrame::Kind::Close) {
            c.out = close_frame(1000);
            c.out_off = 0;
            c.state = Client::State::Closing;
            break;
          }
          if (f.kind == ClientFrame::Kind::Ping && c.out_off >= c.out.size()) {
            c.out = std::string(1, static_cast<char>(0x8A)) + static_cast<char>(f.payload.size()) +
                    f.payload;
            c.out_off = 0;
          }
        }
        if (c.in.size() > kMaxClientFrame) {
          drop(c);
          continue;
        }
      }

      // Refill: the latest snapshot, if this client has not had it yet.
      if (c.state == Client::State::Open && c.out_off >= c.out.size() && latest &&
          c.sent_version != version) {
        c.sent_version = version;
        if (latest->size() > cfg_.max_message_bytes) {
          stats_.oversize_skipped.fetch_add(1);
        } else {
          c.out = encode_text_frame(*latest);
          c.out_off = 0;
          stats_.messages_sent.fetch_add(1);
        }
      }
      if (c.out_off < c.out.size()) {
        const ssize_t n =
            compat::send_nosignal(c.fd, c.out.data() + c.out_off, c.out.size() - c.out_off);
        if (n > 0) {
          c.out_off += static_cast<std::size_t>(n);
          c.last_progress_ms = now;
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
          drop(c);
          continue;
        } else if (now - c.last_progress_ms > cfg_.send_stall_timeout_ms) {
          stats_.stalled_dropped.fetch_add(1);
          drop(c);
          continue;
        }
      } else {
        c.last_progress_ms = now;
        if (c.state == Client::State::Closing) {
          drop(c);
          continue;
        }
      }
    }
    std::erase_if(clients, [](const Client& c) { return c.fd < 0; });
    clients_.store(clients.size(), std::memory_order_relaxed);
  }
  for (Client& c : clients) {
    drop(c);
  }
  clients_.store(0);
}

}  // namespace sifted::net
