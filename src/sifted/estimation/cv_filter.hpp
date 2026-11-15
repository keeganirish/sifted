#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string_view>

#include "sifted/core/result.hpp"

// Constant-velocity Kalman filter in the local x/y tracking frame (see
// sifted/core/sensor.hpp for the frame convention).
//
//   state  x = [px, py, vx, vy]^T     (m, m, m/s, m/s)
//   motion x_k = F(dt) x_{k-1} + w,   w ~ N(0, Q(dt))
//
// Q is the continuous white-noise-acceleration model: each axis is driven by
// an independent acceleration with power spectral density `accel_psd`
// [m^2/s^3], which gives, per axis,
//
//   Q_axis = accel_psd * [ dt^3/3  dt^2/2 ]
//                        [ dt^2/2  dt     ]
//
// The filter holds no clock and no globals: callers pass dt and measurements
// explicitly. Every operation either succeeds or leaves the state untouched
// (results are computed into temporaries and committed only if finite), so a
// bad input can never poison the track.
namespace sifted::estimation {

inline constexpr int kStateDim = 4;
using StateVector = Eigen::Matrix<double, kStateDim, 1>;
using StateCovariance = Eigen::Matrix<double, kStateDim, kStateDim>;

enum class FilterStatus : std::uint8_t {
  Ok,
  InvalidTimeStep,      // dt negative, non-finite, or above kMaxPredictDt
  InvalidInput,         // non-finite measurement, bad noise covariance, ...
  NotPositiveDefinite,  // innovation covariance could not be factorized
  NumericalFailure,     // result was non-finite; state left unchanged
};

std::string_view to_string(FilterStatus s) noexcept;

class ConstantVelocityFilter {
 public:
  // Longest single prediction step accepted. Longer gaps mean the track should
  // be re-initialised rather than extrapolated; the engine's lifecycle drops
  // stale tracks long before this (docs/design.md section 6).
  static constexpr double kMaxPredictDt = 3600.0;

  // Validates x0 (finite), P0 (finite, symmetric, positive definite) and
  // accel_psd (finite, >= 0).
  static Result<ConstantVelocityFilter, FilterStatus> create(const StateVector& x0,
                                                             const StateCovariance& P0,
                                                             double accel_psd);

  FilterStatus predict(double dt) noexcept;

  // Linear update with a direct position measurement z = [px, py] + v,
  // v ~ N(0, R).
  FilterStatus update(const Eigen::Vector2d& z, const Eigen::Matrix2d& R) noexcept;

  // General (E)KF update shared by every measurement model. The caller supplies
  // the innovation y = z - h(x) (already angle-normalised where needed), the
  // measurement Jacobian H evaluated at the current state, and the noise R.
  // Uses the Joseph form so P stays symmetric positive semi-definite.
  template <int M>
  FilterStatus update_with_innovation(const Eigen::Matrix<double, M, 1>& y,
                                      const Eigen::Matrix<double, M, kStateDim>& H,
                                      const Eigen::Matrix<double, M, M>& R) noexcept;

  // Squared Mahalanobis distance d^2 = y^T S^-1 y with S = H P H^T + R, the
  // gating statistic for a measurement linearised at the current state. Read
  // only. nullopt for non-finite input or an S that is not positive definite.
  template <int M>
  std::optional<double> mahalanobis2(const Eigen::Matrix<double, M, 1>& y,
                                     const Eigen::Matrix<double, M, kStateDim>& H,
                                     const Eigen::Matrix<double, M, M>& R) const noexcept;

  // Changes the process noise used by later predictions (e.g. while a
  // manoeuvre is detected). Returns false and changes nothing unless
  // accel_psd is finite and >= 0.
  bool set_accel_psd(double accel_psd) noexcept {
    if (!(accel_psd >= 0.0) || !std::isfinite(accel_psd)) {
      return false;
    }
    accel_psd_ = accel_psd;
    return true;
  }

  const StateVector& state() const noexcept { return x_; }
  const StateCovariance& covariance() const noexcept { return P_; }
  double accel_psd() const noexcept { return accel_psd_; }

  static StateCovariance transition(double dt) noexcept;
  static StateCovariance process_noise(double dt, double accel_psd) noexcept;

 private:
  ConstantVelocityFilter(const StateVector& x0, const StateCovariance& P0, double accel_psd)
      : x_(x0), P_(P0), accel_psd_(accel_psd) {}

  StateVector x_;
  StateCovariance P_;
  double accel_psd_;
};

// True iff `m` is finite, symmetric (to a relative tolerance) and positive
// definite.
template <int N>
bool is_covariance(const Eigen::Matrix<double, N, N>& m) noexcept {
  if (!m.allFinite()) {
    return false;
  }
  const double scale = m.cwiseAbs().maxCoeff();
  if (!((m - m.transpose()).cwiseAbs().maxCoeff() <= 1e-9 * scale)) {
    return false;
  }
  const Eigen::LLT<Eigen::Matrix<double, N, N>> llt(m);
  return llt.info() == Eigen::Success;
}

template <int M>
FilterStatus ConstantVelocityFilter::update_with_innovation(
    const Eigen::Matrix<double, M, 1>& y, const Eigen::Matrix<double, M, kStateDim>& H,
    const Eigen::Matrix<double, M, M>& R) noexcept {
  if (!y.allFinite() || !H.allFinite() || !is_covariance(R)) {
    return FilterStatus::InvalidInput;
  }
  using MatMM = Eigen::Matrix<double, M, M>;
  using MatNM = Eigen::Matrix<double, kStateDim, M>;

  const MatMM S = H * P_ * H.transpose() + R;
  const Eigen::LLT<MatMM> llt(S);
  if (llt.info() != Eigen::Success) {
    return FilterStatus::NotPositiveDefinite;
  }
  // K = P H^T S^-1, computed as (S^-1 (H P))^T since S and P are symmetric.
  const MatNM K = llt.solve(H * P_).transpose();

  const StateVector x_new = x_ + K * y;
  const StateCovariance I_KH = StateCovariance::Identity() - K * H;
  StateCovariance P_new = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
  P_new = 0.5 * (P_new + P_new.transpose());

  if (!x_new.allFinite() || !P_new.allFinite()) {
    return FilterStatus::NumericalFailure;
  }
  x_ = x_new;
  P_ = P_new;
  return FilterStatus::Ok;
}

template <int M>
std::optional<double> ConstantVelocityFilter::mahalanobis2(
    const Eigen::Matrix<double, M, 1>& y, const Eigen::Matrix<double, M, kStateDim>& H,
    const Eigen::Matrix<double, M, M>& R) const noexcept {
  if (!y.allFinite() || !H.allFinite() || !is_covariance(R)) {
    return std::nullopt;
  }
  using MatMM = Eigen::Matrix<double, M, M>;
  const MatMM S = H * P_ * H.transpose() + R;
  const Eigen::LLT<MatMM> llt(S);
  if (llt.info() != Eigen::Success) {
    return std::nullopt;
  }
  const Eigen::Matrix<double, M, 1> w = llt.solve(y);
  const double d2 = y.dot(w);
  if (!std::isfinite(d2)) {
    return std::nullopt;
  }
  return d2;
}

}  // namespace sifted::estimation
