#pragma once
// Small portability layer over the Linux-only socket extensions the daemon
// uses (SOCK_CLOEXEC, accept4, MSG_NOSIGNAL), so the same POSIX code builds
// on macOS and the BSDs. On Linux these compile to exactly the calls used
// before; elsewhere they fall back to fcntl() and SO_NOSIGPIPE.
//
// Define SIFTED_PORTABLE_SOCKETS to force the fallback path on Linux (used to
// compile-check it on CI machines without a Mac).

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstddef>

#if defined(__linux__) && !defined(SIFTED_PORTABLE_SOCKETS)
#define SIFTED_LINUX_SOCKETS 1
#else
#define SIFTED_LINUX_SOCKETS 0
#endif

namespace sifted::net::compat {

inline bool set_cloexec(int fd) {
  const int flags = ::fcntl(fd, F_GETFD, 0);
  return flags >= 0 && ::fcntl(fd, F_SETFD, flags | FD_CLOEXEC) == 0;
}

inline bool set_nonblock(int fd) {
  const int flags = ::fcntl(fd, F_GETFL, 0);
  return flags >= 0 && ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

// socket(2) with close-on-exec set.
inline int open_socket(int domain, int type) {
#if SIFTED_LINUX_SOCKETS
  return ::socket(domain, type | SOCK_CLOEXEC, 0);
#else
  const int fd = ::socket(domain, type, 0);
  if (fd >= 0 && !set_cloexec(fd)) {
    ::close(fd);
    return -1;
  }
  return fd;
#endif
}

// accept(2) returning a non-blocking, close-on-exec socket that never raises
// SIGPIPE on send.
inline int accept_nonblocking(int listen_fd) {
#if SIFTED_LINUX_SOCKETS
  return ::accept4(listen_fd, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
#else
  const int fd = ::accept(listen_fd, nullptr, nullptr);
  if (fd < 0) {
    return fd;
  }
  bool ok = set_cloexec(fd) && set_nonblock(fd);
#if defined(SO_NOSIGPIPE)
  const int one = 1;
  ok = ok && ::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one) == 0;
#endif
  if (!ok) {
    ::close(fd);
    return -1;
  }
  return fd;
#endif
}

// send(2) that reports a closed peer as EPIPE instead of raising SIGPIPE.
inline ssize_t send_nosignal(int fd, const char* data, std::size_t len) {
#if defined(MSG_NOSIGNAL)
  return ::send(fd, data, len, MSG_NOSIGNAL);
#else
  return ::send(fd, data, len, 0);  // SO_NOSIGPIPE set in accept_nonblocking()
#endif
}

}  // namespace sifted::net::compat
