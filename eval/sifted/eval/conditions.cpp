#include "sifted/eval/conditions.hpp"

#include <stdexcept>

#include "sifted/eval/faults.hpp"

namespace sifted::eval {

const std::vector<ImpairmentLevel>& impairment_levels() {
  static const std::vector<ImpairmentLevel> levels = [] {
    constexpr std::int64_t kMs = 1'000'000;
    net::LinkProfile clean;
    net::LinkProfile mild;
    mild.loss = 0.01;
    mild.delay_ns = 20 * kMs;
    mild.jitter_ns = 50 * kMs;
    mild.duplicate = 0.01;
    net::LinkProfile moderate;
    moderate.loss = 0.03;
    moderate.p_good_to_bad = 0.01;
    moderate.p_bad_to_good = 0.25;
    moderate.loss_bad = 0.6;
    moderate.delay_ns = 50 * kMs;
    moderate.jitter_ns = 400 * kMs;
    moderate.duplicate = 0.02;
    net::LinkProfile severe;
    severe.loss = 0.08;
    severe.p_good_to_bad = 0.03;
    severe.p_bad_to_good = 0.2;
    severe.loss_bad = 0.8;
    severe.delay_ns = 100 * kMs;
    severe.jitter_ns = 800 * kMs;
    severe.duplicate = 0.05;
    return std::vector<ImpairmentLevel>{
        {"clean", clean}, {"mild", mild}, {"moderate", moderate}, {"severe", severe}};
  }();
  return levels;
}

const net::LinkProfile& impairment_level(std::string_view name) {
  for (const ImpairmentLevel& l : impairment_levels()) {
    if (l.name == name) {
      return l.profile;
    }
  }
  throw std::out_of_range("no impairment level " + std::string(name));
}

const std::vector<FaultCase>& fault_cases() {
  // See fault_injection_test.cpp for the expected behaviour.
  static const std::vector<FaultCase> cases = {
      {"range-bearing dropout 6 s", "sends nothing for 6 s from 30 s after the first truth instant",
       SensorKind::RangeBearing, 30, 6, true,
       [](const std::vector<Measurement>& ms, SensorId s, std::int64_t a, std::int64_t b) {
         return faults::dropout(ms, s, a, b);
       }},
      {"bearing-only intermittent",
       "is switched off for whole 2 s periods, each with probability 0.4 (seed 17)",
       SensorKind::BearingOnly, 0, 0, false,
       [](const std::vector<Measurement>& ms, SensorId s, std::int64_t, std::int64_t) {
         return faults::intermittent(ms, s, 2 * kNsPerS, 0.4, 17);
       }},
  };
  return cases;
}

const FaultCase& fault_case(std::string_view name) {
  for (const FaultCase& f : fault_cases()) {
    if (f.name == name) {
      return f;
    }
  }
  throw std::out_of_range("no fault case " + std::string(name));
}

std::optional<SensorId> first_of_kind(const SensorTable& table, SensorKind kind) {
  for (const SensorConfig& s : table.sensors()) {
    if (s.kind == kind) {
      return s.id;
    }
  }
  return std::nullopt;
}

}  // namespace sifted::eval
