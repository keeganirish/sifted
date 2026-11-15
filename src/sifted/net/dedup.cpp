#include "sifted/net/dedup.hpp"

#include <algorithm>

namespace sifted::net {

DuplicateFilter::DuplicateFilter(const SensorTable& sensors, std::size_t window)
    : sensors_(&sensors), window_(window), per_sensor_(sensors.size()) {}

bool DuplicateFilter::is_duplicate(SensorId sensor, std::uint32_t seq, Timestamp time) {
  const auto index = sensors_->index_of(sensor);
  if (!index || window_ == 0) {
    return false;
  }
  Window& w = per_sensor_[*index];
  const std::pair<std::uint32_t, std::int64_t> key{seq, time.ns};
  if (std::find(w.keys.begin(), w.keys.end(), key) != w.keys.end()) {
    ++suppressed_;
    return true;
  }
  if (w.keys.size() < window_) {
    w.keys.push_back(key);
  } else {
    w.keys[w.head] = key;
    w.head = (w.head + 1) % window_;
  }
  return false;
}

}  // namespace sifted::net
