#include "sifted/association/lifecycle.hpp"

#include <cmath>

#include "sifted/estimation/cv_filter.hpp"

namespace sifted::association {

std::string_view to_string(TrackStatus s) noexcept {
  switch (s) {
    case TrackStatus::Tentative:
      return "tentative";
    case TrackStatus::Confirmed:
      return "confirmed";
    case TrackStatus::Coasting:
      return "coasting";
    case TrackStatus::Dropped:
      return "dropped";
  }
  return "unknown";
}

std::string_view to_string(DropReason r) noexcept {
  switch (r) {
    case DropReason::None:
      return "none";
    case DropReason::TentativeTimeout:
      return "tentative timeout";
    case DropReason::Stale:
      return "stale";
    case DropReason::Uncertainty:
      return "uncertainty";
    case DropReason::TimelineChange:
      return "timeline change";
  }
  return "unknown";
}

bool is_valid(const LifecycleConfig& c) noexcept {
  constexpr double kMax = estimation::ConstantVelocityFilter::kMaxPredictDt;
  auto positive = [](double v) { return v > 0.0 && std::isfinite(v); };
  return c.confirm_hits >= 1 && positive(c.tentative_timeout_s) && positive(c.coast_after_s) &&
         positive(c.drop_after_s) && positive(c.max_position_sigma_m) &&
         c.coast_after_s <= c.drop_after_s && c.drop_after_s <= kMax &&
         c.tentative_timeout_s <= kMax;
}

}  // namespace sifted::association
