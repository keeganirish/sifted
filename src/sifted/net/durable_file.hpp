#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <streambuf>
#include <string>
#include <thread>
#include <vector>

#include "sifted/core/result.hpp"

namespace sifted::net {

// The stream buffer behind a recording: the durability policy of the event log
// lives here, at the daemon edge, so the log format code stays free of OS calls
// (ADR 0012).
//
// Two different losses are bounded:
//   * Process death (crash, SIGKILL). Bytes survive once they are in the
//     operating system. They are handed over after at most `max_records`
//     records and, as long as the owner calls poll(), `max_age`.
//   * Power loss or a kernel crash. Bytes survive once they are on the device.
//     A background thread calls fdatasync every `sync_interval` so a slow disk
//     never stalls the engine thread. Hardware that lies about its write cache
//     is outside this guarantee.
//
// A failed write (a full disk, say) never throws: the first error is kept,
// nothing more is written, and the stream goes bad, so the file stays a valid
// prefix of the run, at worst with a torn final record.
class DurableFile final : public std::streambuf {
 public:
  using Clock = std::chrono::steady_clock;

  struct Policy {
    std::size_t max_records = 64;
    std::chrono::milliseconds max_age{100};
    std::chrono::milliseconds sync_interval{1000};  // 0: never fdatasync in the background
  };

  // Creates or truncates `path`. The error is a readable message.
  static Result<std::unique_ptr<DurableFile>, std::string> create(const std::string& path,
                                                                  Policy policy);
  ~DurableFile() override;
  DurableFile(const DurableFile&) = delete;
  DurableFile& operator=(const DurableFile&) = delete;

  // Call after each complete record, and regularly while idle.
  void record_written(Clock::time_point now);
  void poll(Clock::time_point now);

  // Hands everything buffered to the operating system. False once failed.
  bool flush_to_os();
  // flush_to_os(), then waits for the device.
  bool sync_to_device();

  bool failed() const noexcept { return error_.load(std::memory_order_relaxed) != 0; }
  // The first error, or empty.
  std::string error() const;

 protected:
  std::streamsize xsputn(const char* s, std::streamsize n) override;
  int_type overflow(int_type ch) override;
  int sync() override;

 private:
  DurableFile(int fd, Policy policy);
  void drain();
  void fail(int err) noexcept;
  void data_sync() noexcept;
  void sync_loop();

  int fd_;
  Policy policy_;
  std::vector<char> buffer_;
  std::size_t used_ = 0;
  std::size_t records_ = 0;            // complete records not yet handed over
  Clock::time_point oldest_{};         // when the oldest of them was written
  std::atomic<int> error_{0};          // first errno, 0 while healthy
  std::atomic<bool> unsynced_{false};  // bytes reached the OS since the last fdatasync

  std::mutex mu_;
  std::condition_variable cv_;
  bool stop_ = false;
  std::thread syncer_;
};

}  // namespace sifted::net
