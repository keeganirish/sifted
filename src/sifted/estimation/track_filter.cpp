#include "sifted/estimation/track_filter.hpp"

#include <type_traits>

namespace sifted::estimation {

std::string_view to_string(MotionModel m) noexcept {
  switch (m) {
    case MotionModel::ConstantVelocity:
      return "cv";
    case MotionModel::Imm:
      return "imm";
  }
  return "unknown";
}

std::optional<MotionModel> motion_model_from_string(std::string_view s) noexcept {
  for (const auto m : {MotionModel::ConstantVelocity, MotionModel::Imm}) {
    if (s == to_string(m)) {
      return m;
    }
  }
  return std::nullopt;
}

FilterStatus TrackFilter::predict(double dt) noexcept {
  return std::visit([dt](auto& f) { return f.predict(dt); }, f_);
}

const StateVector& TrackFilter::state() const noexcept {
  return std::visit([](const auto& f) -> const StateVector& { return f.state(); }, f_);
}

const StateCovariance& TrackFilter::covariance() const noexcept {
  return std::visit([](const auto& f) -> const StateCovariance& { return f.covariance(); }, f_);
}

const StateVector& TrackFilter::component_state(std::size_t i) const noexcept {
  if (const auto* imm = std::get_if<ImmFilter>(&f_)) {
    return imm->mode(i).x;
  }
  return std::get<ConstantVelocityFilter>(f_).state();
}

const StateCovariance& TrackFilter::component_covariance(std::size_t i) const noexcept {
  if (const auto* imm = std::get_if<ImmFilter>(&f_)) {
    return imm->mode(i).P;
  }
  return std::get<ConstantVelocityFilter>(f_).covariance();
}

namespace {

struct Distance {
  double d2;
  double log_det;  // log det S
};

// d^2 = y^T S^-1 y with S = H P H^T + R (the computation of
// ConstantVelocityFilter::mahalanobis2, for any covariance P), and log det S.
template <int M>
std::optional<Distance> distance2(const StateCovariance& P, const Linearized<M>& l) noexcept {
  if (!l.y.allFinite() || !l.H.allFinite() || !is_covariance(l.R)) {
    return std::nullopt;
  }
  using MatMM = Eigen::Matrix<double, M, M>;
  const MatMM S = l.H * P * l.H.transpose() + l.R;
  const Eigen::LLT<MatMM> llt(S);
  if (llt.info() != Eigen::Success) {
    return std::nullopt;
  }
  const double d2 = l.y.dot(llt.solve(l.y));
  double log_det = 0.0;
  for (int k = 0; k < M; ++k) {
    log_det += 2.0 * std::log(llt.matrixL()(k, k));
  }
  if (!std::isfinite(d2) || !std::isfinite(log_det)) {
    return std::nullopt;
  }
  return Distance{d2, log_det};
}

// log sum exp(v_i) over the first n entries, shifted by the maximum.
double log_sum_exp(const std::array<double, kImmModes>& v, std::size_t n) noexcept {
  double top = v[0];
  for (std::size_t i = 1; i < n; ++i) {
    top = std::max(top, v[i]);
  }
  double sum = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    sum += std::exp(v[i] - top);
  }
  return top + std::log(sum);
}

}  // namespace

std::optional<GateResult> TrackFilter::gate(const MeasurementContext& ctx) const noexcept {
  if (const auto* cv = std::get_if<ConstantVelocityFilter>(&f_)) {
    auto lin = linearize(ctx, cv->state());
    if (!lin) {
      return std::nullopt;
    }
    const auto d2 = estimation::mahalanobis2(*cv, *lin);
    if (!d2) {
      return std::nullopt;
    }
    return GateResult{*d2, *lin};
  }
  // The mixture statistic (ADR 0014): with c_j the predicted mode
  // probabilities and d_j^2, S_j each mode's own distance and innovation
  // covariance,
  //   d2_mix = -2 ln( sum_j w_j exp(-d_j^2 / 2) ),
  //   w_j    = c_j |S_j|^(-1/2) / sum_k c_k |S_k|^(-1/2),
  // i.e. -2 ln of the IMM's predicted measurement density p(z) relative to
  // the value it would have if every mode predicted z exactly. Since the w_j
  // sum to 1, min_j d_j^2 <= d2_mix <= min_j (d_j^2 - 2 ln w_j); the lower
  // bound is what keeps the gate screen conservative, and is enforced below
  // against rounding. Modes that cannot be linearised are left out.
  const ImmFilter& imm = std::get<ImmFilter>(f_);
  const ModeVector& c = imm.probabilities();  // all > 0 (floor, transitions)
  std::array<double, kImmModes> log_w{};      // ln c_j - ln|S_j| / 2
  std::array<double, kImmModes> log_t{};      // ... - d_j^2 / 2
  std::size_t n = 0;
  std::optional<GateResult> best;  // smallest d_j^2, with its linearisation
  for (std::size_t i = 0; i < kImmModes; ++i) {
    auto lin = linearize(ctx, imm.mode(i).x);
    if (!lin) {
      continue;
    }
    const StateCovariance& P = imm.mode(i).P;
    const auto d = std::visit([&P](const auto& v) { return distance2(P, v); }, *lin);
    if (!d) {
      continue;
    }
    log_w[n] = std::log(c(static_cast<Eigen::Index>(i))) - 0.5 * d->log_det;
    log_t[n] = log_w[n] - 0.5 * d->d2;
    ++n;
    if (!best || d->d2 < best->d2) {
      best = GateResult{d->d2, *lin};
    }
  }
  if (!best) {
    return std::nullopt;
  }
  const double mix = -2.0 * (log_sum_exp(log_t, n) - log_sum_exp(log_w, n));
  if (!std::isfinite(mix)) {
    return std::nullopt;
  }
  best->d2 = std::max(mix, best->d2);
  return best;
}

std::optional<double> TrackFilter::mahalanobis2(const AnyLinearized& l) const noexcept {
  if (const auto* cv = std::get_if<ConstantVelocityFilter>(&f_)) {
    return estimation::mahalanobis2(*cv, l);
  }
  const ImmFilter& imm = std::get<ImmFilter>(f_);
  return std::visit([&imm](const auto& v) { return imm.mahalanobis2(v.y, v.H, v.R); }, l);
}

FilterStatus TrackFilter::update(const MeasurementContext& ctx, const AnyLinearized& l) noexcept {
  if (auto* cv = std::get_if<ConstantVelocityFilter>(&f_)) {
    return apply(*cv, l);
  }
  ImmFilter& imm = std::get<ImmFilter>(f_);
  return std::visit(
      [&](const auto& v) {
        using Lin = std::remove_cvref_t<decltype(v)>;
        constexpr int M = decltype(v.y)::RowsAtCompileTime;
        return imm.template update<M>([&ctx](const StateVector& x) -> std::optional<Lin> {
          const std::optional<AnyLinearized> at = linearize(ctx, x);
          if (!at || !std::holds_alternative<Lin>(*at)) {
            return std::nullopt;
          }
          return std::get<Lin>(*at);
        });
      },
      l);
}

bool TrackFilter::set_accel_psd(double accel_psd) noexcept {
  auto* cv = std::get_if<ConstantVelocityFilter>(&f_);
  return cv != nullptr && cv->set_accel_psd(accel_psd);
}

}  // namespace sifted::estimation
