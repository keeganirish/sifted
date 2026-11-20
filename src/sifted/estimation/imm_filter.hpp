#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "sifted/core/result.hpp"
#include "sifted/estimation/cv_filter.hpp"
#include "sifted/estimation/linearized.hpp"

// Interacting Multiple Model estimator (Blom and Bar-Shalom, 1988; Bar-Shalom,
// Li and Kirubarajan, "Estimation with Applications to Tracking and
// Navigation", 2001, sec. 11.6). Design and parameter reasoning: ADR 0014.
//
// Modes. kImmModes nearly-constant-velocity models in the same state
// x = [px, py, vx, vy] that differ only in their white-noise acceleration PSD
// q_i (ConstantVelocityFilter::process_noise): "quiet" (straight flight),
// "manoeuvre" (ordinary turns) and "hard" (high-g turns). Every mode has the
// same state, so mixing is exact moment matching (no augmentation or
// truncation), and every mode uses the shared measurement models unchanged.
//
// Mode switching is a continuous-time Markov chain: mode i is left at rate
// 1/tau_i (mean sojourn tau_i seconds) for each other mode with equal
// probability, so the generator is G_ii = -1/tau_i, G_ij = 1/((N-1) tau_i), and
// the transition matrix over dt is Pi(dt) = expm(G dt), with
// Pi_ij = P(mode j at t + dt | mode i at t). It depends on elapsed time, is a
// semigroup (Pi(a) Pi(b) = Pi(a + b)), and Pi(0) = I. Its stationary
// distribution is proportional to tau_i.
//
// One cycle (sec. 11.6.6):
//   predict(dt):  predicted mode probabilities  c_j = sum_i Pi_ij mu_i;
//                 mixing weights                mu_i|j = Pi_ij mu_i / c_j;
//                 mixed initial conditions      x0_j = sum_i mu_i|j x_i,
//                   P0_j = sum_i mu_i|j (P_i + (x_i - x0_j)(x_i - x0_j)^T);
//                 mode-matched prediction with q_j; mu <- c.
//   update:       mode-matched (E)KF update of each mode, each linearised at
//                 its own predicted state (Joseph form); mode likelihoods
//                 L_j = N(y_j; 0, S_j); mu_j <- c_j L_j / sum_k c_k L_k.
//   output:       x = sum_j mu_j x_j,
//                 P = sum_j mu_j (P_j + (x_j - x)(x_j - x)^T)  (spread of means).
// The output (state(), covariance()) is kept current after every operation.
//
// Robustness:
//   * likelihoods are combined in log space (log c_j + log L_j, shifted by
//     the maximum before exponentiation), so a measurement far from every
//     mode cannot underflow all weights to zero; if the normalisation is
//     still not finite and positive, the predicted probabilities are kept;
//   * every probability is floored at `probability_floor` and renormalised,
//     so no mode can become unreachable (relevant for repeated same-time
//     updates, where dt = 0 gives no transition mass);
//   * an update is all or nothing: if any mode cannot be linearised or its
//     update is not finite, nothing changes and the update is rejected;
//   * dt = 0 is an exact no-op (several sensor groups can share a time);
//   * results are computed into temporaries and committed only if finite.
namespace sifted::estimation {

inline constexpr std::size_t kImmModes = 3;
using ModeVector = Eigen::Matrix<double, kImmModes, 1>;
using ModeMatrix = Eigen::Matrix<double, kImmModes, kImmModes>;

struct ImmParams {
  // White-noise acceleration PSD per mode [m^2/s^3], non-decreasing: quiet,
  // manoeuvre, hard. Rule of thumb q ~ (0.5..1) a^2 T at T = 1 s (ADR 0014):
  //   quiet      ~1 m/s^2 of gusts and speed/height-keeping corrections;
  //   manoeuvre  a = 5 m/s^2, factor 1: ordinary turns (a standard-rate turn
  //              at 250 m/s is ~4 m/s^2);
  //   hard       a = 5 g, factor 0.5: high-g turns of agile aircraft.
  std::array<double, kImmModes> accel_psd{0.5, 25.0, 1200.0};
  // Mean sojourn time per mode [s]: straight legs last minutes, a turn of
  // up to 180 degrees lasts tens of seconds, and a high-g turn ~10-20 s.
  std::array<double, kImmModes> sojourn_s{60.0, 30.0, 15.0};
  // Lower bound on every mode probability after an update. Recovering from
  // the floor to an even chance needs a likelihood ratio of 1 / floor
  // (~9 nats, d^2 ~ 18 for 1e-4), i.e. one update far outside the gate.
  double probability_floor = 1e-4;

  friend bool operator==(const ImmParams&, const ImmParams&) = default;
};

inline constexpr double kMinSojournS = 0.1;
inline constexpr double kMaxSojournS = 1e6;

// q finite, >= 0 and non-decreasing; tau in [kMinSojournS, kMaxSojournS];
// floor in (0, 1 / (2 kImmModes)].
bool is_valid(const ImmParams& p) noexcept;

// Pi(dt) (rows: from, columns: to). Precondition: is_valid(p), 0 <= dt <=
// ConstantVelocityFilter::kMaxPredictDt. Exactly the identity for dt = 0.
// Scaling and squaring of a degree-12 Taylor series: every operation is a
// plain IEEE product or sum, so the result is deterministic. Rows are
// renormalised to sum to 1.
ModeMatrix transition_matrix(const ImmParams& p, double dt) noexcept;
// The chain's stationary distribution, proportional to the sojourn times.
ModeVector stationary_probabilities(const ImmParams& p) noexcept;

struct ModeEstimate {
  StateVector x;
  StateCovariance P;
};

class ImmFilter {
 public:
  // Every mode starts at (x0, P0) with the stationary mode probabilities: a
  // single position fix says nothing about how the target is moving.
  // Validates x0, P0 (as ConstantVelocityFilter::create) and params.
  static Result<ImmFilter, FilterStatus> create(const StateVector& x0, const StateCovariance& P0,
                                                const ImmParams& params);

  FilterStatus predict(double dt) noexcept;

  // Mode-matched update. `linearize_at(x)` returns the measurement linearised
  // at state x (std::optional<Linearized<M>>); it is called once per mode at
  // that mode's predicted state. All or nothing (see above). On success the
  // mode probabilities are updated with the mode likelihoods.
  template <int M, class Linearize>
  FilterStatus update(Linearize&& linearize_at) noexcept;

  // The combined estimate.
  const StateVector& state() const noexcept { return x_; }
  const StateCovariance& covariance() const noexcept { return P_; }
  const ModeVector& probabilities() const noexcept { return mu_; }
  const ModeEstimate& mode(std::size_t i) const noexcept { return modes_[i]; }
  const ImmParams& params() const noexcept { return params_; }

  // Squared Mahalanobis distance under the combined (moment-matched)
  // estimate: the gating statistic (ADR 0014). Read only.
  template <int M>
  std::optional<double> mahalanobis2(const Eigen::Matrix<double, M, 1>& y,
                                     const Eigen::Matrix<double, M, kStateDim>& H,
                                     const Eigen::Matrix<double, M, M>& R) const noexcept;

  // Output combination: x = sum mu_j x_j, P = sum mu_j (P_j + d_j d_j^T),
  // symmetrised. Exposed for tests.
  static void combine(const std::array<ModeEstimate, kImmModes>& modes, const ModeVector& mu,
                      StateVector& x, StateCovariance& P) noexcept;
  // Floors every entry at `floor` and renormalises. Exposed for tests.
  static ModeVector apply_floor(const ModeVector& mu, double floor) noexcept;

 private:
  ImmFilter(const StateVector& x0, const StateCovariance& P0, const ImmParams& params);

  std::array<ModeEstimate, kImmModes> modes_;
  ModeVector mu_;
  StateVector x_;
  StateCovariance P_;
  ImmParams params_;
};

template <int M, class Linearize>
FilterStatus ImmFilter::update(Linearize&& linearize_at) noexcept {
  using MatMM = Eigen::Matrix<double, M, M>;
  using MatNM = Eigen::Matrix<double, kStateDim, M>;
  constexpr double kLog2Pi = 1.8378770664093454836;

  std::array<ModeEstimate, kImmModes> out;
  ModeVector log_w;
  for (std::size_t j = 0; j < kImmModes; ++j) {
    const ModeEstimate& m = modes_[j];
    const std::optional<Linearized<M>> l = linearize_at(m.x);
    if (!l) {
      return FilterStatus::InvalidInput;  // e.g. this mode's prediction is too close to the sensor
    }
    if (!l->y.allFinite() || !l->H.allFinite() || !is_covariance(l->R)) {
      return FilterStatus::InvalidInput;
    }
    const MatMM S = l->H * m.P * l->H.transpose() + l->R;
    const Eigen::LLT<MatMM> llt(S);
    if (llt.info() != Eigen::Success) {
      return FilterStatus::NotPositiveDefinite;
    }
    const MatNM K = llt.solve(l->H * m.P).transpose();
    const StateVector x_new = m.x + K * l->y;
    const StateCovariance I_KH = StateCovariance::Identity() - K * l->H;
    StateCovariance P_new = I_KH * m.P * I_KH.transpose() + K * l->R * K.transpose();
    P_new = (0.5 * (P_new + P_new.transpose())).eval();  // eval: exactly symmetric, no aliasing
    const double d2 = l->y.dot(llt.solve(l->y));
    double log_det = 0.0;
    for (int k = 0; k < M; ++k) {
      log_det += 2.0 * std::log(llt.matrixL()(k, k));
    }
    // log N(y; 0, S) = -(d2 + log det S + M log 2 pi) / 2. The constant is the
    // same for every mode (same measurement), so 1-D bearings and 2-D
    // positions are treated alike: only ratios between modes matter.
    log_w(static_cast<Eigen::Index>(j)) =
        std::log(mu_(static_cast<Eigen::Index>(j))) - 0.5 * (d2 + log_det + M * kLog2Pi);
    if (!x_new.allFinite() || !P_new.allFinite() || !std::isfinite(d2) || !std::isfinite(log_det)) {
      return FilterStatus::NumericalFailure;
    }
    out[j] = {x_new, P_new};
  }
  ModeVector mu = mu_;
  const double top = log_w.maxCoeff();
  if (std::isfinite(top)) {
    ModeVector w;
    for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(kImmModes); ++j) {
      w(j) = std::exp(log_w(j) - top);  // the largest is exactly 1
    }
    const double sum = w.sum();
    if (std::isfinite(sum) && sum > 0.0) {
      mu = apply_floor(w / sum, params_.probability_floor);
    }
  }
  StateVector x;
  StateCovariance P;
  combine(out, mu, x, P);
  if (!x.allFinite() || !P.allFinite() || !mu.allFinite()) {
    return FilterStatus::NumericalFailure;
  }
  modes_ = out;
  mu_ = mu;
  x_ = x;
  P_ = P;
  return FilterStatus::Ok;
}

template <int M>
std::optional<double> ImmFilter::mahalanobis2(const Eigen::Matrix<double, M, 1>& y,
                                              const Eigen::Matrix<double, M, kStateDim>& H,
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
  const double d2 = y.dot(llt.solve(y));
  if (!std::isfinite(d2)) {
    return std::nullopt;
  }
  return d2;
}

}  // namespace sifted::estimation
