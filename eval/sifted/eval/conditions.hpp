#pragma once

// The evaluation conditions shared by the degraded-network and fault tests and
// sifted_eval: named impairment levels (docs/design.md section 9), the outage
// cases used to measure recovery, and the sensor-fault cases. Defined once so
// the scorecard and the tests cannot drift apart.

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "sifted/core/measurement.hpp"
#include "sifted/core/sensor.hpp"
#include "sifted/net/impairment.hpp"

namespace sifted::eval {

inline constexpr std::int64_t kNsPerS = 1'000'000'000;

// Named impairment levels, in increasing severity, "clean" first. Delays are
// one-way; the scenarios' sensors report at 1 Hz (radar scans) and 2 Hz
// (camera), so jitter of a few hundred ms reorders scans and splits them.
struct ImpairmentLevel {
  std::string name;
  net::LinkProfile profile;
};
const std::vector<ImpairmentLevel>& impairment_levels();
// The level called `name`; throws std::out_of_range if there is none.
const net::LinkProfile& impairment_level(std::string_view name);

// The reorder window the daemon uses by default (ADR 0009; sifted_fusiond).
inline constexpr double kDaemonReorderWindowS = 0.5;

// Recovery cases: an outage of every link starting kOutageStartS after the
// first truth instant, for each of kOutageLengthsS, on top of kOutageLevel.
inline constexpr std::string_view kOutageLevel = "mild";
inline constexpr std::int64_t kOutageStartS = 30;
inline constexpr std::int64_t kOutageLengthsS[] = {5, 8, 15};

// Sensor-fault cases run on this level's link, so only the fault is measured.
inline constexpr std::string_view kFaultLevel = "clean";

// A sensor fault applied to the first sensor of `kind` in table order.
// `start_s` / `length_s` place the fault window relative to the first truth
// instant; recovery is measured after a window with `recovery` set.
struct FaultCase {
  std::string name;
  std::string description;  // what the faulted sensor does, for reports
  SensorKind kind;
  std::int64_t start_s = 0;
  std::int64_t length_s = 0;
  bool recovery = false;
  // (measurements, faulted sensor, window start [ns], window end [ns]).
  std::function<std::vector<Measurement>(const std::vector<Measurement>&, SensorId, std::int64_t,
                                         std::int64_t)>
      apply;
};
const std::vector<FaultCase>& fault_cases();
// The fault case called `name`; throws std::out_of_range if there is none.
const FaultCase& fault_case(std::string_view name);

// First sensor of `kind` in table order, if any.
std::optional<SensorId> first_of_kind(const SensorTable& table, SensorKind kind);

}  // namespace sifted::eval
