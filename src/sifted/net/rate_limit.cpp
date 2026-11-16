#include "sifted/net/rate_limit.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sifted::net {

bool is_valid(const RateLimitConfig& c) noexcept {
  if (!c.enabled()) {
    return c.rate_per_s == 0.0;
  }
  return std::isfinite(c.rate_per_s) && c.burst >= 1.0 && std::isfinite(c.burst) &&
         c.max_sources >= 1 && c.max_sources <= (1U << 20);
}

SourceRateLimiter::SourceRateLimiter(RateLimitConfig config) : config_(config) {
  if (!is_valid(config_)) {
    throw std::invalid_argument("sifted::net::SourceRateLimiter: invalid RateLimitConfig");
  }
  if (config_.enabled()) {
    table_.reserve(std::min<std::size_t>(config_.max_sources, 4096));
  }
}

bool SourceRateLimiter::admit(std::uint64_t source, Timestamp now) {
  if (!config_.enabled()) {
    return true;
  }
  auto it = std::find_if(table_.begin(), table_.end(),
                         [&](const Entry& e) { return e.source == source; });
  if (it == table_.end()) {
    if (table_.size() >= config_.max_sources) {
      it = std::min_element(table_.begin(), table_.end(),
                            [](const Entry& a, const Entry& b) { return a.last < b.last; });
      *it = {source, config_.burst, now};
      ++evictions_;
    } else {
      table_.push_back({source, config_.burst, now});
      it = table_.end() - 1;
    }
  } else if (now > it->last) {
    const double elapsed = static_cast<double>(static_cast<std::uint64_t>(now.ns) -
                                               static_cast<std::uint64_t>(it->last.ns)) *
                           1e-9;
    it->tokens = std::min(config_.burst, it->tokens + elapsed * config_.rate_per_s);
    it->last = now;
  }
  if (it->tokens < 1.0) {
    return false;
  }
  it->tokens -= 1.0;
  return true;
}

}  // namespace sifted::net
