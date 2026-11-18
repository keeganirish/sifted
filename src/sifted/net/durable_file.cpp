#include "sifted/net/durable_file.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <utility>

namespace sifted::net {
namespace {

constexpr std::size_t kBufferSize = 64 * 1024;

// Makes the new file's directory entry durable too: without it, power loss
// could lose the whole recording however often the data itself was synced.
void sync_parent_directory(const std::string& path) noexcept {
  const auto slash = path.rfind('/');
  const std::string dir = slash == std::string::npos ? "." : path.substr(0, slash + 1);
  const int fd = ::open(dir.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd >= 0) {
    (void)::fsync(fd);  // best effort: not every file system supports it
    (void)::close(fd);
  }
}

}  // namespace

Result<std::unique_ptr<DurableFile>, std::string> DurableFile::create(const std::string& path,
                                                                      Policy policy) {
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0666);
  if (fd < 0) {
    return "cannot write " + path + ": " + std::strerror(errno);
  }
  sync_parent_directory(path);
  return std::unique_ptr<DurableFile>(new DurableFile(fd, policy));
}

DurableFile::DurableFile(int fd, Policy policy) : fd_(fd), policy_(policy), buffer_(kBufferSize) {
  if (policy_.sync_interval.count() > 0) {
    syncer_ = std::thread([this] { sync_loop(); });
  }
}

DurableFile::~DurableFile() {
  {
    const std::lock_guard<std::mutex> lock(mu_);
    stop_ = true;
  }
  cv_.notify_all();
  if (syncer_.joinable()) {
    syncer_.join();
  }
  (void)sync_to_device();
  (void)::close(fd_);
}

void DurableFile::fail(int err) noexcept {
  int none = 0;
  (void)error_.compare_exchange_strong(none, err, std::memory_order_relaxed);
}

std::string DurableFile::error() const {
  const int err = error_.load(std::memory_order_relaxed);
  return err == 0 ? std::string{} : std::string(std::strerror(err));
}

void DurableFile::drain() {
  const char* p = buffer_.data();
  std::size_t left = used_;
  used_ = 0;
  while (left > 0 && !failed()) {
    const ssize_t n = ::write(fd_, p, left);
    if (n < 0) {
      if (errno != EINTR) {
        fail(errno);
      }
      continue;
    }
    p += n;
    left -= static_cast<std::size_t>(n);
    unsynced_.store(true, std::memory_order_relaxed);
  }
}

std::streamsize DurableFile::xsputn(const char* s, std::streamsize n) {
  if (failed() || n <= 0) {
    return 0;
  }
  auto left = static_cast<std::size_t>(n);
  while (left > 0) {
    if (used_ == buffer_.size()) {
      drain();
      if (failed()) {
        return n - static_cast<std::streamsize>(left);
      }
    }
    const std::size_t chunk = std::min(left, buffer_.size() - used_);
    std::memcpy(buffer_.data() + used_, s, chunk);
    used_ += chunk;
    s += chunk;
    left -= chunk;
  }
  return n;
}

DurableFile::int_type DurableFile::overflow(int_type ch) {
  if (traits_type::eq_int_type(ch, traits_type::eof())) {
    return traits_type::not_eof(ch);
  }
  const char c = traits_type::to_char_type(ch);
  return xsputn(&c, 1) == 1 ? ch : traits_type::eof();
}

int DurableFile::sync() { return flush_to_os() ? 0 : -1; }

bool DurableFile::flush_to_os() {
  drain();
  records_ = 0;
  return !failed();
}

void DurableFile::data_sync() noexcept {
#ifdef __APPLE__
  const int rc = ::fsync(fd_);
#else
  const int rc = ::fdatasync(fd_);
#endif
  // Not every target can sync (a pipe, a special file): that is not a failure.
  if (rc != 0 && errno != EINVAL && errno != ENOTSUP && errno != EROFS && errno != EINTR) {
    fail(errno);
  }
}

bool DurableFile::sync_to_device() {
  const bool ok = flush_to_os();
  unsynced_.store(false, std::memory_order_relaxed);
  data_sync();
  return ok && !failed();
}

void DurableFile::record_written(Clock::time_point now) {
  if (records_ == 0) {
    oldest_ = now;
  }
  ++records_;
  if (records_ >= policy_.max_records || now - oldest_ >= policy_.max_age) {
    (void)flush_to_os();
  }
}

void DurableFile::poll(Clock::time_point now) {
  if (records_ > 0 && now - oldest_ >= policy_.max_age) {
    (void)flush_to_os();
  }
}

void DurableFile::sync_loop() {
  std::unique_lock<std::mutex> lock(mu_);
  while (!stop_) {
    cv_.wait_for(lock, policy_.sync_interval);
    if (!stop_ && unsynced_.exchange(false, std::memory_order_relaxed)) {
      lock.unlock();
      data_sync();
      lock.lock();
    }
  }
}

}  // namespace sifted::net
