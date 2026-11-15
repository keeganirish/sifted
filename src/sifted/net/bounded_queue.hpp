#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace sifted::net {

// Bounded multi-producer, single-consumer FIFO between the network threads
// and the engine thread (docs/design.md section 2). Producers never block:
// when the queue is full the item is refused and counted, so a flood costs
// dropped input, never unbounded memory or a stalled receiver (threat model:
// resource exhaustion). The consumer sees items in the order they were
// accepted. All state is guarded by one mutex.
template <class T>
class BoundedQueue {
 public:
  explicit BoundedQueue(std::size_t capacity) : capacity_(capacity) {}

  // False (and counted) if the queue is full or closed.
  bool try_push(T item) {
    {
      const std::lock_guard<std::mutex> lock(mu_);
      if (closed_ || items_.size() >= capacity_) {
        ++dropped_;
        return false;
      }
      items_.push_back(std::move(item));
      ++accepted_;
    }
    cv_.notify_one();
    return true;
  }

  // Waits up to `timeout` for an item. nullopt on timeout, or once the queue
  // is closed and drained.
  std::optional<T> pop_for(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mu_);
    cv_.wait_for(lock, timeout, [this] { return closed_ || !items_.empty(); });
    if (items_.empty()) {
      return std::nullopt;
    }
    T item = std::move(items_.front());
    items_.pop_front();
    return item;
  }

  // Refuses further pushes and wakes the consumer; queued items can still be
  // popped.
  void close() {
    {
      const std::lock_guard<std::mutex> lock(mu_);
      closed_ = true;
    }
    cv_.notify_all();
  }

  bool closed() const {
    const std::lock_guard<std::mutex> lock(mu_);
    return closed_;
  }
  std::size_t size() const {
    const std::lock_guard<std::mutex> lock(mu_);
    return items_.size();
  }
  std::uint64_t dropped() const {
    const std::lock_guard<std::mutex> lock(mu_);
    return dropped_;
  }
  std::uint64_t accepted() const {
    const std::lock_guard<std::mutex> lock(mu_);
    return accepted_;
  }
  std::size_t capacity() const noexcept { return capacity_; }

 private:
  const std::size_t capacity_;
  mutable std::mutex mu_;
  std::condition_variable cv_;
  std::deque<T> items_;
  bool closed_ = false;
  std::uint64_t dropped_ = 0;
  std::uint64_t accepted_ = 0;
};

}  // namespace sifted::net
