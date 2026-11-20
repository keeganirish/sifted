// The IMM estimator (estimation/imm_filter.hpp, ADR 0014): hand-checkable
// mixing, transition and combination arithmetic, the reduction to a single
// Kalman filter, the zero-time-step no-op, robustness of the mode likelihoods,
// all-or-nothing updates, and covariance health over long random sequences.

#include <gtest/gtest.h>

#include <Eigen/LU>
#include <cmath>
#include <optional>

#include "sifted/estimation/bearing_only.hpp"
#include "sifted/estimation/imm_filter.hpp"
#include "sifted/estimation/measurement_model.hpp"
#include "sifted/estimation/range_bearing.hpp"
#include "sifted/estimation/track_filter.hpp"
#include "sifted/net/impairment.hpp"

namespace sifted::estimation {
namespace {

constexpr auto kModes = static_cast<Eigen::Index>(kImmModes);

StateVector state(double px, double py, double vx, double vy) {
  StateVector x;
  x << px, py, vx, vy;
  return x;
}

StateCovariance diag(double pos, double vel) {
  StateCovariance P = StateCovariance::Zero();
  P.diagonal() << pos, pos, vel, vel;
  return P;
}

ImmFilter make(const ImmParams& p = {}, const StateVector& x = state(4000.0, 3000.0, -50.0, 20.0),
               const StateCovariance& P = diag(100.0, 400.0)) {
  return ImmFilter::create(x, P, p).value();
}

// A position measurement as a linearize_at callback.
auto position(const Eigen::Vector2d& z, double sigma) {
  return [z, sigma](const StateVector& x) -> std::optional<Linearized<2>> {
    Linearized<2> l;
    l.H = Eigen::Matrix<double, 2, kStateDim>::Zero();
    l.H(0, 0) = 1.0;
    l.H(1, 1) = 1.0;
    l.y = z - x.head<2>();
    l.R = Eigen::Matrix2d::Identity() * sigma * sigma;
    return l;
  };
}

// Finite, positive-definite estimates; probabilities a distribution, every one
// positive (after an update: at least the renormalised floor).
bool healthy(const ImmFilter& f, std::optional<double> floor = std::nullopt) {
  if (!is_covariance(f.covariance()) || !f.state().allFinite()) {
    return false;
  }
  for (std::size_t j = 0; j < kImmModes; ++j) {
    if (!is_covariance(f.mode(j).P) || !f.mode(j).x.allFinite()) {
      return false;
    }
  }
  const ModeVector& mu = f.probabilities();
  const double lowest = floor ? *floor / (1.0 + 2.0 * *floor) * (1.0 - 1e-12) : 0.0;
  return std::abs(mu.sum() - 1.0) < 1e-12 && mu.minCoeff() > 0.0 && mu.minCoeff() >= lowest;
}

// ---- transition model --------------------------------------------------------

TEST(Imm, TransitionMatrixIsIdentityAtZeroAndStochastic) {
  const ImmParams p;
  EXPECT_TRUE(transition_matrix(p, 0.0) == ModeMatrix::Identity());  // exactly
  for (const double dt : {0.001, 0.5, 1.0, 7.3, 60.0, 3600.0}) {
    const ModeMatrix Pi = transition_matrix(p, dt);
    for (Eigen::Index i = 0; i < kModes; ++i) {
      EXPECT_NEAR(Pi.row(i).sum(), 1.0, 1e-15) << dt;
      EXPECT_GE(Pi.row(i).minCoeff(), 0.0) << dt;
    }
  }
  // Staying probabilities fall with elapsed time: transitions depend on dt.
  EXPECT_GT(transition_matrix(p, 0.5)(0, 0), transition_matrix(p, 1.0)(0, 0));
  EXPECT_GT(transition_matrix(p, 1.0)(0, 0), transition_matrix(p, 5.0)(0, 0));
}

TEST(Imm, TransitionMatrixMatchesClosedFormForEqualSojourns) {
  // All tau equal: G = (J - N I) / ((N - 1) tau), whose exponential is
  // Pi_ii = 1/N + (N-1)/N e^{-N dt / ((N-1) tau)}, Pi_ij = (1 - e^{...}) / N.
  ImmParams p;
  p.sojourn_s = {20.0, 20.0, 20.0};
  const double n = static_cast<double>(kImmModes);
  for (const double dt : {0.25, 1.0, 13.0, 200.0}) {
    const double e = std::exp(-n * dt / ((n - 1.0) * 20.0));
    const ModeMatrix Pi = transition_matrix(p, dt);
    for (Eigen::Index i = 0; i < kModes; ++i) {
      for (Eigen::Index j = 0; j < kModes; ++j) {
        const double want = i == j ? 1.0 / n + (n - 1.0) / n * e : (1.0 - e) / n;
        EXPECT_NEAR(Pi(i, j), want, 1e-14) << dt << " " << i << j;
      }
    }
  }
}

TEST(Imm, TransitionMatrixIsASemigroupWithTheSojournStationaryDistribution) {
  const ImmParams p;
  const ModeMatrix a = transition_matrix(p, 0.7);
  const ModeMatrix b = transition_matrix(p, 2.1);
  EXPECT_LT((a * b - transition_matrix(p, 2.8)).cwiseAbs().maxCoeff(), 1e-14);
  const ModeVector pi = stationary_probabilities(p);
  EXPECT_NEAR(pi(0), 60.0 / 105.0, 1e-15);
  EXPECT_NEAR(pi(2), 15.0 / 105.0, 1e-15);
  EXPECT_LT((transition_matrix(p, 3.0).transpose() * pi - pi).cwiseAbs().maxCoeff(), 1e-14);
  // After a long time any start forgets itself.
  EXPECT_LT((transition_matrix(p, 3600.0).row(0).transpose() - pi).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(Imm, ParamsAreValidated) {
  ImmParams p;
  EXPECT_TRUE(is_valid(p));
  p.accel_psd = {1.0, 0.5, 10.0};  // not non-decreasing
  EXPECT_FALSE(is_valid(p));
  p = {};
  p.accel_psd[2] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(is_valid(p));
  p = {};
  p.sojourn_s[1] = 0.0;
  EXPECT_FALSE(is_valid(p));
  p = {};
  p.probability_floor = 0.0;
  EXPECT_FALSE(is_valid(p));
  p.probability_floor = 0.5;  // above 1 / (2 N)
  EXPECT_FALSE(is_valid(p));
  EXPECT_FALSE(ImmFilter::create(state(0, 0, 0, 0), diag(1.0, 1.0), p));
}

// ---- mixing and combination ------------------------------------------------

TEST(Imm, CombinationIncludesTheSpreadOfMeans) {
  std::array<ModeEstimate, kImmModes> m;
  m[0] = {state(0.0, 0.0, 0.0, 0.0), StateCovariance::Identity()};
  m[1] = {state(2.0, 0.0, 0.0, 4.0), StateCovariance::Identity()};
  m[2] = {state(100.0, 100.0, 100.0, 100.0), StateCovariance::Identity() * 9.0};
  const ModeVector mu(0.5, 0.5, 0.0);
  StateVector x;
  StateCovariance P;
  ImmFilter::combine(m, mu, x, P);
  EXPECT_TRUE(x.isApprox(state(1.0, 0.0, 0.0, 2.0)));
  // P = sum mu (P_j + d_j d_j^T): d = (-1, 0, 0, -2) and (1, 0, 0, 2).
  StateCovariance want = StateCovariance::Identity();
  want(0, 0) += 1.0;
  want(3, 3) += 4.0;
  want(0, 3) = want(3, 0) = 2.0;
  EXPECT_LT((P - want).cwiseAbs().maxCoeff(), 1e-15);
}

TEST(Imm, PredictMixesWithHandComputedWeights) {
  ImmFilter f = make();
  ASSERT_EQ(f.predict(1.0), FilterStatus::Ok);
  // A measurement 60 m off the prediction separates the modes.
  ASSERT_EQ(f.update<2>(position(f.state().head<2>() + Eigen::Vector2d(60.0, -30.0), 10.0)),
            FilterStatus::Ok);
  const ImmFilter before = f;
  const double dt = 2.0;
  ASSERT_EQ(f.predict(dt), FilterStatus::Ok);

  const ModeMatrix Pi = transition_matrix(before.params(), dt);
  const ModeVector& mu = before.probabilities();
  for (std::size_t j = 0; j < kImmModes; ++j) {
    const auto jj = static_cast<Eigen::Index>(j);
    double c = 0.0;
    for (Eigen::Index i = 0; i < kModes; ++i) {
      c += Pi(i, jj) * mu(i);
    }
    StateVector x0 = StateVector::Zero();
    for (std::size_t i = 0; i < kImmModes; ++i) {
      x0 += Pi(static_cast<Eigen::Index>(i), jj) * mu(static_cast<Eigen::Index>(i)) / c *
            before.mode(i).x;
    }
    StateCovariance P0 = StateCovariance::Zero();
    for (std::size_t i = 0; i < kImmModes; ++i) {
      const StateVector d = before.mode(i).x - x0;
      P0 += Pi(static_cast<Eigen::Index>(i), jj) * mu(static_cast<Eigen::Index>(i)) / c *
            (before.mode(i).P + d * d.transpose());
    }
    const StateCovariance F = ConstantVelocityFilter::transition(dt);
    const StateVector xj = F * x0;
    const StateCovariance Pj = F * P0 * F.transpose() + ConstantVelocityFilter::process_noise(
                                                            dt, before.params().accel_psd[j]);
    EXPECT_LT((f.mode(j).x - xj).cwiseAbs().maxCoeff(), 1e-9) << j;
    EXPECT_LT((f.mode(j).P - Pj).cwiseAbs().maxCoeff(), 1e-7 * Pj.cwiseAbs().maxCoeff()) << j;
    EXPECT_NEAR(f.probabilities()(jj), c, 1e-15) << j;
  }
}

TEST(Imm, UpdateWeighsModesByTheirLikelihoods) {
  ImmFilter f = make();
  ASSERT_EQ(f.predict(1.0), FilterStatus::Ok);
  const ModeVector c = f.probabilities();
  const ImmFilter predicted = f;
  const Eigen::Vector2d z = f.state().head<2>() + Eigen::Vector2d(80.0, 40.0);
  ASSERT_EQ(f.update<2>(position(z, 10.0)), FilterStatus::Ok);
  // mu_j ~ c_j N(y_j; 0, S_j), by hand.
  ModeVector w;
  for (std::size_t j = 0; j < kImmModes; ++j) {
    const Eigen::Vector2d y = z - predicted.mode(j).x.head<2>();
    const Eigen::Matrix2d S =
        predicted.mode(j).P.topLeftCorner<2, 2>() + Eigen::Matrix2d::Identity() * 100.0;
    const double like = std::exp(-0.5 * y.dot(S.inverse() * y)) /
                        (2.0 * 3.14159265358979323846 * std::sqrt(S.determinant()));
    w(static_cast<Eigen::Index>(j)) = c(static_cast<Eigen::Index>(j)) * like;
  }
  w = ImmFilter::apply_floor(w / w.sum(), f.params().probability_floor);
  EXPECT_LT((f.probabilities() - w).cwiseAbs().maxCoeff(), 1e-12);
  // A large miss favours the high-noise modes.
  EXPECT_GT(f.probabilities()(2), c(2));
}

// ---- reduction and no-op -------------------------------------------------------

TEST(Imm, IdenticalModesReduceToTheSingleFilter) {
  ImmParams p;
  p.accel_psd = {4.0, 4.0, 4.0};
  const StateVector x0 = state(4000.0, 3000.0, -50.0, 20.0);
  const StateCovariance P0 = diag(100.0, 2500.0);
  ImmFilter imm = ImmFilter::create(x0, P0, p).value();
  ConstantVelocityFilter cv = ConstantVelocityFilter::create(x0, P0, 4.0).value();
  const SensorPose radar{0.0, 0.0, 0.0};
  const SensorPose camera{-2000.0, 1000.0, 0.3};
  net::SplitMix64 rng(3);
  for (int k = 0; k < 200; ++k) {
    const double dt = 0.2 + 1.5 * rng.uniform();
    ASSERT_EQ(imm.predict(dt), FilterStatus::Ok);
    ASSERT_EQ(cv.predict(dt), FilterStatus::Ok);
    const StateVector truth = cv.state();
    if (k % 3 == 2) {
      const double b = predict_bearing(truth, camera) + 0.003 * (rng.uniform() - 0.5);
      const auto l = bearing_innovation(cv.state(), camera, b, 0.002);
      ASSERT_TRUE(l);
      ASSERT_EQ(cv.update_with_innovation<1>(l->y, l->H, l->R), FilterStatus::Ok);
      ASSERT_EQ(imm.update<1>(
                    [&](const StateVector& x) { return bearing_innovation(x, camera, b, 0.002); }),
                FilterStatus::Ok);
    } else {
      const RangeBearing t = predict_range_bearing(truth, radar);
      const RangeBearing z{t.range_m + 20.0 * (rng.uniform() - 0.5),
                           t.bearing_rad + 0.006 * (rng.uniform() - 0.5)};
      const auto l = range_bearing_innovation(cv.state(), radar, z, {10.0, 0.0035});
      ASSERT_TRUE(l);
      ASSERT_EQ(cv.update_with_innovation<2>(l->y, l->H, l->R), FilterStatus::Ok);
      ASSERT_EQ(imm.update<2>([&](const StateVector& x) {
        return range_bearing_innovation(x, radar, z, {10.0, 0.0035});
      }),
                FilterStatus::Ok);
    }
    ASSERT_LT((imm.state() - cv.state()).cwiseAbs().maxCoeff(), 1e-6) << k;
    ASSERT_LT((imm.covariance() - cv.covariance()).cwiseAbs().maxCoeff(),
              1e-9 * cv.covariance().cwiseAbs().maxCoeff())
        << k;
    // Equal likelihoods: the probabilities stay where the chain puts them.
    ASSERT_LT((imm.probabilities() - stationary_probabilities(p)).cwiseAbs().maxCoeff(), 1e-9);
  }
}

TEST(Imm, ZeroTimeStepIsAnExactNoOp) {
  ImmFilter f = make();
  ASSERT_EQ(f.predict(1.0), FilterStatus::Ok);
  ASSERT_EQ(f.update<2>(position(f.state().head<2>() + Eigen::Vector2d(30.0, 5.0), 10.0)),
            FilterStatus::Ok);
  const ImmFilter before = f;
  ASSERT_EQ(f.predict(0.0), FilterStatus::Ok);
  EXPECT_TRUE(f.state() == before.state());
  EXPECT_TRUE(f.covariance() == before.covariance());
  EXPECT_TRUE(f.probabilities() == before.probabilities());
  for (std::size_t j = 0; j < kImmModes; ++j) {
    EXPECT_TRUE(f.mode(j).x == before.mode(j).x);
    EXPECT_TRUE(f.mode(j).P == before.mode(j).P);
  }
  // Invalid steps are refused without change.
  for (const double dt : {-1.0, std::nan(""), ConstantVelocityFilter::kMaxPredictDt * 2.0}) {
    EXPECT_EQ(f.predict(dt), FilterStatus::InvalidTimeStep);
  }
  EXPECT_TRUE(f.state() == before.state());
}

TEST(Imm, CoastingRelaxesTowardsTheStationaryDistribution) {
  ImmFilter f = make();
  for (int k = 0; k < 30; ++k) {  // straight flight, exact measurements: quiet wins
    ASSERT_EQ(f.predict(1.0), FilterStatus::Ok);
    const StateVector t = state(4000.0 - 50.0 * (k + 1), 3000.0 + 20.0 * (k + 1), 0, 0);
    ASSERT_EQ(f.update<2>(position(t.head<2>(), 10.0)), FilterStatus::Ok);
  }
  ASSERT_GT(f.probabilities()(0), 0.8);
  ImmFilter coast = f;
  ASSERT_EQ(coast.predict(600.0), FilterStatus::Ok);  // no measurement for 10 minutes
  EXPECT_LT((coast.probabilities() - stationary_probabilities(f.params())).cwiseAbs().maxCoeff(),
            1e-3);
}

// ---- robustness ---------------------------------------------------------------

TEST(Imm, ExtremeInnovationsKeepProbabilitiesFiniteAndFloored) {
  ImmFilter f = make();
  ASSERT_EQ(f.predict(1.0), FilterStatus::Ok);
  // So far off that every mode's likelihood underflows in linear space.
  ASSERT_EQ(f.update<2>(position(Eigen::Vector2d(1e7, -1e7), 1.0)), FilterStatus::Ok);
  EXPECT_TRUE(healthy(f, f.params().probability_floor));
  EXPECT_GT(f.probabilities()(2), 0.9);  // the widest mode explains it best
  // Many same-time updates (dt = 0: no transition mass) cannot push a mode
  // below the floor, and the floored mode can still win back.
  for (int k = 0; k < 50; ++k) {
    ASSERT_EQ(f.predict(0.0), FilterStatus::Ok);
    ASSERT_EQ(f.update<2>(position(f.state().head<2>(), 1.0)), FilterStatus::Ok);
    ASSERT_TRUE(healthy(f, f.params().probability_floor)) << k;
  }
}

TEST(Imm, UpdateIsAllOrNothing) {
  ImmFilter f = make();
  ASSERT_EQ(f.predict(1.0), FilterStatus::Ok);
  const ImmFilter before = f;
  // A linearisation that succeeds for some modes and fails for another.
  int calls = 0;
  const auto status = f.update<2>([&](const StateVector& x) -> std::optional<Linearized<2>> {
    return ++calls == 2 ? std::nullopt : position(x.head<2>(), 10.0)(x);
  });
  EXPECT_EQ(status, FilterStatus::InvalidInput);
  EXPECT_TRUE(f.state() == before.state());
  EXPECT_TRUE(f.probabilities() == before.probabilities());
  // Non-finite measurement: rejected, unchanged.
  EXPECT_NE(f.update<2>(position(Eigen::Vector2d(std::nan(""), 0.0), 10.0)), FilterStatus::Ok);
  EXPECT_TRUE(f.covariance() == before.covariance());
}

TEST(Imm, NoGateWhenNoModeCanBeLinearised) {
  // Every mode within kMinRangeForUpdateM (1 m) of the sensor: no statistic,
  // so the pair is never a candidate (a mode that alone fails is covered by
  // UpdateIsAllOrNothing).
  ImmParams p;
  StateCovariance P = diag(1e-4, 1e-4);
  ImmFilter imm = ImmFilter::create(state(0.6, 0.0, 0.0, 0.0), P, p).value();
  ASSERT_EQ(imm.predict(1e-3), FilterStatus::Ok);
  TrackFilter f(imm);
  Measurement m;
  m.sensor_id = 1;
  m.kind = SensorKind::RangeBearing;
  m.z = {0.6, 0.0, 0.0};
  m.sigma = {0.1, 0.01, 0.0};
  const auto ctx = make_context(m, SensorPose{0.0, 0.0, 0.0}, std::nullopt);
  ASSERT_TRUE(ctx);
  EXPECT_FALSE(f.gate(*ctx).has_value());
}

TEST(Imm, StaysPositiveDefiniteOverLongRandomSequences) {
  const SensorPose radar{0.0, 0.0, 0.0};
  const SensorPose camera{-3000.0, 2000.0, -0.6};
  for (std::uint64_t seed = 1; seed <= 20; ++seed) {
    net::SplitMix64 rng(seed);
    ImmFilter f = make({}, state(5000.0 * rng.uniform() + 500.0, 5000.0 * rng.uniform(), 0, 0),
                       diag(400.0, 10000.0));
    for (int k = 0; k < 500; ++k) {
      const double r = rng.uniform();
      const double dt = r < 0.1 ? 0.0 : r < 0.95 ? 2.0 * rng.uniform() : 60.0 * rng.uniform();
      ASSERT_EQ(f.predict(dt), FilterStatus::Ok);
      const RangeBearing t = predict_range_bearing(f.state(), radar);
      const double kick = rng.uniform() < 0.05 ? 2000.0 : 30.0;  // outliers too
      if (rng.uniform() < 0.3) {
        const double b = predict_bearing(f.state(), camera) + 0.01 * (rng.uniform() - 0.5);
        (void)f.update<1>(
            [&](const StateVector& x) { return bearing_innovation(x, camera, b, 0.002); });
      } else {
        const RangeBearing z{std::max(1.0, t.range_m + kick * (rng.uniform() - 0.5)),
                             t.bearing_rad + 0.01 * (rng.uniform() - 0.5)};
        (void)f.update<2>([&](const StateVector& x) {
          return range_bearing_innovation(x, radar, z, {10.0, 0.0035});
        });
      }
      ASSERT_TRUE(healthy(f)) << "seed " << seed << " step " << k;
      ASSERT_LE((f.covariance() - f.covariance().transpose()).cwiseAbs().maxCoeff(), 0.0)
          << "not exactly symmetric";
    }
  }
}

}  // namespace
}  // namespace sifted::estimation
