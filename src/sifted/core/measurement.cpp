#include "sifted/core/measurement.hpp"

#include <bit>
#include <cmath>
#include <cstddef>

namespace sifted {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxRangeM = 1.0e7;
constexpr double kMinAltitudeM = -1000.0;
constexpr double kMaxAltitudeM = 100000.0;
constexpr double kMaxSigma = 1.0e6;

struct Bounds {
  double lo;
  double hi;
};

constexpr bool in(double v, Bounds b) noexcept {
  return v >= b.lo && v <= b.hi;  // false for NaN
}

bool values_in_range(const Measurement& m) noexcept {
  switch (m.kind) {
    case SensorKind::RangeBearing:
      return in(m.z[0], {0.0, kMaxRangeM}) && in(m.z[1], {-kPi, kPi});
    case SensorKind::BearingOnly:
      return in(m.z[0], {-kPi, kPi});
    case SensorKind::GeoPosition:
      return in(m.z[0], {-90.0, 90.0}) && in(m.z[1], {-180.0, 180.0}) &&
             in(m.z[2], {kMinAltitudeM, kMaxAltitudeM});
  }
  return false;
}

}  // namespace

bool is_valid(const Measurement& m) noexcept {
  const std::size_t used = used_components(m.kind);
  if (used == 0) {
    return false;
  }
  if (!values_in_range(m)) {
    return false;
  }
  for (std::size_t i = 0; i < m.z.size(); ++i) {
    if (i < used) {
      // `> 0` and `<= max` are both false for NaN, so NaN is rejected here.
      if (!(m.sigma[i] > 0.0 && m.sigma[i] <= kMaxSigma)) {
        return false;
      }
    } else if (std::bit_cast<std::uint64_t>(m.z[i]) != 0 ||
               std::bit_cast<std::uint64_t>(m.sigma[i]) != 0) {
      return false;  // unused slots must be canonical +0.0
    }
  }
  return true;
}

}  // namespace sifted
