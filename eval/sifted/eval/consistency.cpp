#include "sifted/eval/consistency.hpp"

#include <Eigen/Cholesky>
#include <cmath>
#include <limits>

#include "sifted/association/gating.hpp"

namespace sifted::eval {

double nees(const Eigen::Ref<const Eigen::VectorXd>& e,
            const Eigen::Ref<const Eigen::MatrixXd>& P) {
  const Eigen::LLT<Eigen::MatrixXd> llt(P);
  if (llt.info() != Eigen::Success || P.rows() != e.size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return e.dot(llt.solve(e));
}

double normal_quantile(double p) {
  if (!(p > 0.0 && p < 1.0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // Phi(x) = erfc(-x / sqrt 2) / 2 is increasing; |x| <= 40 covers every
  // representable p.
  double lo = -40.0;
  double hi = 40.0;
  for (int i = 0; i < 200; ++i) {
    const double mid = 0.5 * (lo + hi);
    (0.5 * std::erfc(-mid / std::sqrt(2.0)) < p ? lo : hi) = mid;
  }
  return 0.5 * (lo + hi);
}

Interval chi_square_interval(std::size_t dof, double probability) {
  const double tail = 0.5 * (1.0 - probability);
  if (dof == 0 || !(tail > 0.0 && tail < 0.5)) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return {nan, nan};
  }
  if (dof <= association::kMaxChiSquareDof) {
    return {*association::chi_square_quantile(dof, tail),
            *association::chi_square_quantile(dof, 1.0 - tail)};
  }
  // Wilson-Hilferty: (X / k)^(1/3) is approximately normal with mean
  // 1 - 2 / (9k) and variance 2 / (9k).
  const double k = static_cast<double>(dof);
  const double s = std::sqrt(2.0 / (9.0 * k));
  auto q = [&](double z) { return k * std::pow(1.0 - 2.0 / (9.0 * k) + z * s, 3); };
  return {q(normal_quantile(tail)), q(normal_quantile(1.0 - tail))};
}

Interval ChiSquareStat::interval(double probability) const {
  const Interval sum_interval = chi_square_interval(dof, probability);
  const double count = static_cast<double>(n);
  return {sum_interval.lo / count, sum_interval.hi / count};
}

}  // namespace sifted::eval
