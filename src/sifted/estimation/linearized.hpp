#pragma once

#include <Eigen/Core>

#include "sifted/estimation/cv_filter.hpp"

namespace sifted::estimation {

// A measurement model linearised at a predicted state: innovation
// y = z - h(x) (angles already wrapped), Jacobian H = dh/dx and noise R.
// Shared by gating (Mahalanobis distance) and the filter update.
template <int M>
struct Linearized {
  Eigen::Matrix<double, M, 1> y;
  Eigen::Matrix<double, M, kStateDim> H;
  Eigen::Matrix<double, M, M> R;
};

}  // namespace sifted::estimation
