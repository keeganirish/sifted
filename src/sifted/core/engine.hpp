#pragma once

#include <cstdint>

#include "sifted/core/measurement.hpp"

namespace sifted {

// The fusion core: association -> estimation -> track store.
//
// Contract (what makes deterministic replay possible):
//   * Single-threaded. Concurrency lives around the engine (network threads
//     feed an ordered queue); never inside it.
//   * Its output is a pure function of the ordered sequence of inputs it is
//     given. It reads no wall clock, no RNG seeded from the machine, and
//     iterates no container whose order depends on addresses or hashes.
//
// PLACEHOLDER: today it only counts input. Association, Kalman filters and the
// track store arrive in milestone 1-2 (see docs/design.md).
class Engine {
 public:
  void on_measurement(const Measurement& m) noexcept;

  std::uint64_t measurements_processed() const noexcept { return processed_; }

 private:
  std::uint64_t processed_ = 0;
};

}  // namespace sifted
