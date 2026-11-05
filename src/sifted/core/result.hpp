#pragma once

#include <utility>
#include <variant>

namespace sifted {

// Minimal stand-in for std::expected (C++23). T and E must be distinct types.
// Used so that parsing untrusted input reports *why* it was rejected without
// exceptions on the hot path.
template <class T, class E>
class [[nodiscard]] Result {
 public:
  Result(T value) : v_(std::in_place_index<0>, std::move(value)) {}  // NOLINT(*-explicit-*)
  Result(E error) : v_(std::in_place_index<1>, std::move(error)) {}  // NOLINT(*-explicit-*)

  bool ok() const noexcept { return v_.index() == 0; }
  explicit operator bool() const noexcept { return ok(); }

  // Precondition: ok(). Throws std::bad_variant_access otherwise.
  const T& value() const { return std::get<0>(v_); }
  // Precondition: !ok(). Throws std::bad_variant_access otherwise.
  const E& error() const { return std::get<1>(v_); }

 private:
  std::variant<T, E> v_;
};

}  // namespace sifted
