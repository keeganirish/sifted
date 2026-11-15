#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace sifted::store {

// Fixed-capacity ring buffer: keeps the newest `capacity` items, overwriting
// the oldest. Iteration (at(0) .. at(size()-1)) is oldest to newest, so its
// order depends only on what was pushed. Memory is bounded by construction.
template <class T>
class Ring {
 public:
  explicit Ring(std::size_t capacity = 0) : capacity_(capacity) { items_.reserve(capacity); }

  void push(const T& v) {
    ++pushed_;
    if (capacity_ == 0) {
      return;
    }
    if (items_.size() < capacity_) {
      items_.push_back(v);
      return;
    }
    items_[head_] = v;
    head_ = (head_ + 1) % capacity_;
  }

  std::size_t size() const noexcept { return items_.size(); }
  std::size_t capacity() const noexcept { return capacity_; }
  bool empty() const noexcept { return items_.empty(); }
  // Everything ever pushed, including what has since been overwritten.
  std::uint64_t pushed() const noexcept { return pushed_; }
  // i-th oldest retained item; precondition i < size().
  const T& at(std::size_t i) const { return items_[(head_ + i) % items_.size()]; }
  const T& back() const { return at(size() - 1); }

 private:
  std::size_t capacity_;
  std::vector<T> items_;
  std::size_t head_ = 0;  // index of the oldest item once full
  std::uint64_t pushed_ = 0;
};

}  // namespace sifted::store
