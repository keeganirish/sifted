#include "sifted/eval/gospa.hpp"

#include <cmath>
#include <stdexcept>

#include "sifted/association/assignment.hpp"

namespace sifted::eval {

bool is_valid(const GospaParams& g) noexcept {
  return std::isfinite(g.c) && g.c > 0.0 && std::isfinite(g.p) && g.p >= 1.0;
}

Gospa gospa(const std::vector<Point2>& x, const std::vector<Point2>& y, const GospaParams& params) {
  if (!is_valid(params)) {
    throw std::invalid_argument("GOSPA: c must be finite and > 0, p finite and >= 1");
  }
  for (const std::vector<Point2>* set : {&x, &y}) {
    for (const Point2& pt : *set) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) {
        throw std::invalid_argument("GOSPA: non-finite coordinate");
      }
    }
  }
  const double cp = std::pow(params.c, params.p);
  std::vector<double> cost(x.size() * y.size(), association::kForbidden);
  for (std::size_t i = 0; i < x.size(); ++i) {
    for (std::size_t j = 0; j < y.size(); ++j) {
      const double d = std::hypot(x[i].x - y[j].x, x[i].y - y[j].y);
      if (d < params.c) {
        cost[i * y.size() + j] = std::pow(d, params.p);
      }
    }
  }
  Gospa g;
  g.assignment = association::solve_assignment(cost, x.size(), y.size(), cp);
  for (std::size_t i = 0; i < x.size(); ++i) {
    const int j = g.assignment[i];
    if (j != association::kUnassigned) {
      g.localisation += cost[i * y.size() + static_cast<std::size_t>(j)];
      ++g.assigned;
    }
  }
  g.missed_count = x.size() - g.assigned;
  g.false_count = y.size() - g.assigned;
  g.missed = 0.5 * cp * static_cast<double>(g.missed_count);
  g.false_tracks = 0.5 * cp * static_cast<double>(g.false_count);
  g.total = std::pow(g.localisation + g.missed + g.false_tracks, 1.0 / params.p);
  return g;
}

void GospaSum::add(const Gospa& g, double p) {
  ++instants;
  sum_p += std::pow(g.total, p);
  localisation += g.localisation;
  missed += g.missed;
  false_tracks += g.false_tracks;
  missed_count += g.missed_count;
  false_count += g.false_count;
}

void GospaSum::merge(const GospaSum& o) {
  instants += o.instants;
  sum_p += o.sum_p;
  localisation += o.localisation;
  missed += o.missed;
  false_tracks += o.false_tracks;
  missed_count += o.missed_count;
  false_count += o.false_count;
}

double GospaSum::mean_total(double p) const {
  return instants ? std::pow(sum_p / static_cast<double>(instants), 1.0 / p) : 0.0;
}

}  // namespace sifted::eval
