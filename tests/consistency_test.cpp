// Filter consistency: is the covariance the filter reports (and the operator
// map draws as the 1-sigma circle) statistically honest?
//
// Definitions, the acceptance interval and the caveat on correlated samples:
// consistency.hpp. Which fixture samples are scored: Score in
// scenario_support.hpp. Results are recorded in docs/design.md section 7
// ("Filter consistency").
//
// Two kinds of evidence, asserted differently because theory says different
// things about them:
//
// 1. Monte Carlo on a matched model (ConsistencyMonteCarlo.*). Truth follows
//    exactly the model the filter assumes (constant velocity driven by white
//    acceleration of the filter's own PSD), the radar has exactly the noise it
//    reports, and runs are independent. Here theory says the filter must be
//    consistent and the chi-square interval is exact (up to the EKF's
//    linearisation, negligible at 5 km with 0.2 deg, and gate truncation), so
//    NEES and NIS are asserted inside their two-sided 95% intervals.
//    Manoeuvre switching is turned off: it changes q on purpose, which is a
//    model mismatch by design.
//
// 2. The fixture scenarios (ConsistencyScenario.*). One realisation each, with
//    time-correlated samples, and truth that is NOT the filter's model: the
//    simulator flies exactly straight legs and exactly constant-rate turns
//    (zero acceleration, then a fixed one), while the filter assumes white
//    acceleration noise of PSD q. Consequences, from theory:
//      * straight flight: assumed process noise that is not there makes P
//        larger than the actual error covariance, so the filter must not be
//        optimistic. Asserted: average <= the interval's upper bound. How
//        pessimistic it is does not shrink with q: at steady state about half
//        of P is process noise for any q > 0 (ZeroAccelerationTruth shows it).
//        A loose lower sanity limit (n / 4: covariance at most ~4x, sigma ~2x,
//        too large) catches a covariance that blows up.
//      * manoeuvres: a CV filter lags in a turn and is optimistic there; the
//        NIS-based switch raises q to compensate. Only loose sanity limits are
//        asserted: the average NEES must stay below the engine's own gate
//        quantile (truth on average inside the region the engine treats as
//        plausible), and the average NIS per degree of freedom must stay below
//        the level at which the manoeuvre detector would be switched on
//        permanently (its statistic rho has mean E[d2] / (1 - alpha), so a
//        mean d2 above on * (1 - alpha) per 2 dof means switching fails to
//        hold the innovations down).
//    Whether a scenario is straight is read from its truth (no target's
//    velocity ever changes), never from its name.

#include "sifted/eval/consistency.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "scenario_support.hpp"
#include "sifted/association/gating.hpp"
#include "sifted/net/impairment.hpp"

namespace sifted {
namespace {

using testing::ChiSquareStat;
using testing::Interval;

constexpr double kPi = 3.14159265358979323846;

const char* kind_name(SensorKind k) {
  switch (k) {
    case SensorKind::RangeBearing:
      return "range_bearing";
    case SensorKind::BearingOnly:
      return "bearing_only";
    case SensorKind::GeoPosition:
      return "geo_position";
  }
  return "?";
}

void print_stat(const std::string& name, const std::string& what, const ChiSquareStat& s) {
  if (s.n == 0) {
    std::printf("[ scenario ] %s: %s: no samples\n", name.c_str(), what.c_str());
    return;
  }
  const Interval iv = s.interval();
  const char* verdict = s.average() < iv.lo   ? " (below: pessimistic)"
                        : s.average() > iv.hi ? " (above: optimistic)"
                                              : "";
  std::printf("[ scenario ] %s: %s %.3f (expect %.0f, 95%% [%.3f, %.3f], n=%zu)%s\n", name.c_str(),
              what.c_str(), s.average(), s.expected(), iv.lo, iv.hi, s.n, verdict);
}

// ---- unit tests of the helpers -------------------------------------------

TEST(Consistency, NeesOfKnownErrorAndCovariance) {
  // Diagonal: 1^2 / 4 + 2^2 / 1.
  EXPECT_DOUBLE_EQ(testing::nees(Eigen::Vector2d(1.0, 2.0),
                                 Eigen::Matrix2d(Eigen::Vector2d(4.0, 1.0).asDiagonal())),
                   4.25);
  // Correlated: P = [[2, 1], [1, 2]], P^-1 = [[2, -1], [-1, 2]] / 3; e = (1, 0) -> 2/3.
  Eigen::Matrix2d P;
  P << 2.0, 1.0, 1.0, 2.0;
  EXPECT_NEAR(testing::nees(Eigen::Vector2d(1.0, 0.0), P), 2.0 / 3.0, 1e-15);
  // 4-D, as the scorer uses it: e = sigma along every axis gives n.
  Eigen::Vector4d sig(10.0, 20.0, 3.0, 4.0);
  Eigen::Matrix4d P4 = sig.cwiseProduct(sig).asDiagonal();
  EXPECT_NEAR(testing::nees(sig, P4), 4.0, 1e-12);
  // Not positive definite: NaN, never a silently wrong number.
  EXPECT_TRUE(std::isnan(testing::nees(Eigen::Vector2d(1.0, 0.0), Eigen::Matrix2d::Zero())));
}

TEST(Consistency, ChiSquareIntervalMatchesTables) {
  // Two-sided 95%: 2.5% and 97.5% quantiles (standard tables).
  struct Row {
    std::size_t dof;
    double lo;
    double hi;
  };
  for (const Row& r : {Row{1, 0.000982069, 5.023886}, Row{2, 0.0506356, 7.377759},
                       Row{10, 3.246973, 20.483177}, Row{100, 74.221927, 129.561197}}) {
    const Interval iv = testing::chi_square_interval(r.dof);
    EXPECT_NEAR(iv.lo, r.lo, 1e-5 * std::max(1.0, r.lo)) << r.dof;
    EXPECT_NEAR(iv.hi, r.hi, 1e-5 * r.hi) << r.dof;
  }
  // Beyond kMaxChiSquareDof: Wilson-Hilferty, within 1e-3 relative.
  for (const Row& r : {Row{200, 162.728, 241.058}, Row{1000, 914.257, 1089.531}}) {
    const Interval iv = testing::chi_square_interval(r.dof);
    EXPECT_NEAR(iv.lo, r.lo, 1e-3 * r.lo) << r.dof;
    EXPECT_NEAR(iv.hi, r.hi, 1e-3 * r.hi) << r.dof;
  }
  // The approximation joins the exact quantiles without a jump at the switch:
  // one more dof moves the quantile k + z sqrt(2k) by about 1 + z / sqrt(2k).
  const std::size_t k = association::kMaxChiSquareDof;
  const Interval exact = testing::chi_square_interval(k);
  const Interval approx = testing::chi_square_interval(k + 1);
  const double z = testing::normal_quantile(0.975);
  const double step = z / std::sqrt(2.0 * static_cast<double>(k));
  EXPECT_NEAR(approx.hi - exact.hi, 1.0 + step, 0.01);
  EXPECT_NEAR(approx.lo - exact.lo, 1.0 - step, 0.01);
  EXPECT_NEAR(testing::normal_quantile(0.975), 1.959963984540054, 1e-9);

  // Average of N samples of dimension n: chi-square(N n) / N.
  ChiSquareStat s;
  for (int i = 0; i < 50; ++i) {
    s.add(2.0, 2);
  }
  EXPECT_DOUBLE_EQ(s.average(), 2.0);
  EXPECT_DOUBLE_EQ(s.expected(), 2.0);
  EXPECT_NEAR(s.interval().lo, 74.221927 / 50.0, 1e-6);
  EXPECT_NEAR(s.interval().hi, 129.561197 / 50.0, 1e-6);
}

// ---- Monte Carlo on a matched model ----------------------------------------

double gaussian(net::SplitMix64& rng) {  // Box-Muller
  const double u1 = 1.0 - rng.uniform();
  const double u2 = rng.uniform();
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}

struct MonteCarlo {
  // Per evaluation time (every kMcEvalEvery steps), over runs.
  std::vector<std::size_t> step;
  std::vector<ChiSquareStat> prior;       // 4-dof, predicted to t_k before the update
  std::vector<ChiSquareStat> posterior;   // 4-dof, after the update at t_k
  std::vector<ChiSquareStat> posterior2;  // 2-dof position, after the update
  ChiSquareStat nis;                      // every associated update past the burn-in
  std::size_t lost = 0;                   // runs whose track was lost (matched IMM model only)
};

constexpr std::size_t kMcRuns = 200;
constexpr std::size_t kMcSteps = 91;  // t = 0 .. 90 s at 1 Hz, as the fixtures
constexpr std::size_t kMcBurnIn = static_cast<std::size_t>(testing::kRmseBurnInS);
constexpr std::size_t kMcEvalEvery = 10;
constexpr double kSigmaRange = 10.0;  // the fixtures' radar
constexpr double kSigmaBearing = 0.2 * kPi / 180.0;

// One range-bearing radar at the origin; one target ~5 km out at ~60 m/s whose
// velocity is driven by white acceleration noise of PSD q_truth (exact
// discretisation for dt = 1 s). Run r uses seed r: independent runs.
//
// With `modes` (the IMM's matched model), q_truth is instead the PSD of the
// truth's current mode, which starts from the stationary distribution and
// switches every second with the IMM's own transition matrix Pi(1 s). A run
// whose track is lost (the truth can random-walk to hundreds of m/s in the
// hard mode) is counted in `lost` and contributes no samples.
void monte_carlo(double q_truth, const EngineConfig& cfg, MonteCarlo& mc,
                 const estimation::ImmParams* modes = nullptr) {
  constexpr SensorId kRadar = 1;
  SensorTable table;
  EXPECT_FALSE(table.add({kRadar, SensorKind::RangeBearing, {0.0, 0.0, 0.0}}));
  // Q = q [[1/3, 1/2], [1/2, 1]] per axis, and its Cholesky factor.
  struct Chol {
    double l11, l21, l22;
  };
  auto chol = [](double q) {
    const double l11 = std::sqrt(q / 3.0);
    const double l21 = l11 > 0.0 ? (q / 2.0) / l11 : 0.0;
    return Chol{l11, l21, std::sqrt(std::max(0.0, q - l21 * l21))};
  };
  // Draws a mode from probabilities p (one uniform draw).
  auto draw = [](net::SplitMix64& rng, const estimation::ModeVector& p) {
    const double u = rng.uniform();
    double acc = 0.0;
    for (Eigen::Index i = 0; i + 1 < p.size(); ++i) {
      acc += p(i);
      if (u < acc) {
        return i;
      }
    }
    return p.size() - 1;
  };
  const estimation::ModeMatrix Pi =
      modes ? estimation::transition_matrix(*modes, 1.0) : estimation::ModeMatrix::Identity();
  mc = {};
  for (std::size_t k = kMcBurnIn; k < kMcSteps; k += kMcEvalEvery) {
    mc.step.push_back(k);
  }
  mc.prior.resize(mc.step.size());
  mc.posterior.resize(mc.step.size());
  mc.posterior2.resize(mc.step.size());
  MonteCarlo local;  // one run's samples (matched IMM model: merged if the run completes)
  for (std::size_t run = 0; run < kMcRuns; ++run) {
    local = {};
    local.prior.resize(mc.step.size());
    local.posterior.resize(mc.step.size());
    local.posterior2.resize(mc.step.size());
    MonteCarlo& acc = modes ? local : mc;
    bool lost = false;
    net::SplitMix64 rng(run);
    Engine engine(table, cfg);
    Eigen::Vector4d x(3000.0, 4000.0, -60.0, 20.0);
    TrackId id = 0;  // the track started by the first measurement
    Eigen::Index mode = modes ? draw(rng, estimation::stationary_probabilities(*modes)) : 0;
    for (std::size_t k = 0; k < kMcSteps; ++k) {
      if (k > 0) {
        if (modes) {
          mode = draw(rng, Pi.row(mode).transpose());
        }
        const Chol L = chol(modes ? modes->accel_psd[static_cast<std::size_t>(mode)] : q_truth);
        for (Eigen::Index axis = 0; axis < 2; ++axis) {
          const double n1 = gaussian(rng);
          const double n2 = gaussian(rng);
          x(axis) += x(2 + axis) + L.l11 * n1;
          x(2 + axis) += L.l21 * n1 + L.l22 * n2;
        }
      }
      const Timestamp t{static_cast<std::int64_t>(k) * 1'000'000'000};
      Measurement m;
      m.sensor_id = kRadar;
      m.kind = SensorKind::RangeBearing;
      m.seq = static_cast<std::uint32_t>(k);
      m.time = t;
      const double r = std::hypot(x(0), x(1)) + kSigmaRange * gaussian(rng);
      const double b =
          std::remainder(std::atan2(x(1), x(0)) + kSigmaBearing * gaussian(rng), 2.0 * kPi);
      m.z = {r, b, 0.0};
      m.sigma = {kSigmaRange, kSigmaBearing, 0.0};
      const auto eval = std::find(mc.step.begin(), mc.step.end(), k);
      const auto e = static_cast<std::size_t>(eval - mc.step.begin());
      if (modes && k > 0 && !engine.track_at(id, t)) {
        lost = true;
        break;
      }
      if (eval != mc.step.end()) {
        const auto before = engine.track_at(id, t);
        EXPECT_TRUE(before.has_value()) << "run " << run << " step " << k;
        if (before) {
          acc.prior[e].add(testing::nees(before->state - x, before->covariance), 4);
        }
      }
      const MeasurementResult res = engine.on_measurement_result(m);
      if (k == 0) {
        ASSERT_EQ(res.outcome, MeasurementOutcome::Initialized);
        id = res.track;
      }
      // NIS of this update, as the engine traced it. A measurement outside
      // the gate (0.1%) is not associated and has no NIS: gating truncates
      // the distribution slightly (mean 1.986 instead of 2 at P_G = 0.999).
      const store::TrackRecord* rec = engine.store().find(id);
      if (modes && (rec == nullptr || (k >= kMcBurnIn && res.track != id))) {
        lost = true;  // dropped, or the measurement went to another track
        break;
      }
      ASSERT_NE(rec, nullptr) << "run " << run << " step " << k;
      if (k >= kMcBurnIn && res.track == id) {
        // The newest Associated event (lifecycle events may follow it).
        const store::TraceEvent* assoc = nullptr;
        for (std::size_t i = rec->recent.size(); i-- > 0 && assoc == nullptr;) {
          if (rec->recent.at(i).kind == store::TraceEventKind::Associated) {
            assoc = &rec->recent.at(i);
          }
        }
        ASSERT_TRUE(assoc != nullptr && assoc->time == t) << "run " << run << " step " << k;
        // The NIS under the track estimate (equal to the gating d^2 for the
        // constant-velocity filter; TraceEvent::nis).
        acc.nis.add(assoc->nis, 2);
      }
      if (eval != mc.step.end()) {
        const auto after = engine.track(id);
        ASSERT_TRUE(after.has_value());
        const Eigen::Vector4d err = after->state - x;
        acc.posterior[e].add(testing::nees(err, after->covariance), 4);
        acc.posterior2[e].add(testing::nees(err.head<2>(), after->covariance.topLeftCorner<2, 2>()),
                              2);
      }
    }
    if (modes) {
      if (lost) {
        ++mc.lost;
        continue;
      }
      for (std::size_t i = 0; i < mc.step.size(); ++i) {
        mc.prior[i].merge(local.prior[i]);
        mc.posterior[i].merge(local.posterior[i]);
        mc.posterior2[i].merge(local.posterior2[i]);
      }
      mc.nis.merge(local.nis);
    }
  }
}

void print_monte_carlo(const std::string& name, const MonteCarlo& mc) {
  for (std::size_t e = 0; e < mc.step.size(); ++e) {
    const std::string at = "t=" + std::to_string(mc.step[e]) + "s ";
    print_stat(name, at + "NEES4 prior", mc.prior[e]);
    print_stat(name, at + "NEES4 posterior", mc.posterior[e]);
    print_stat(name, at + "NEES2 posterior", mc.posterior2[e]);
  }
  print_stat(name, "NIS range_bearing", mc.nis);
}

// The legacy constant-velocity filter (whatever the default motion model) with
// its manoeuvre switching off.
EngineConfig without_manoeuvre_switching() {
  EngineConfig cfg;
  cfg.motion_model = estimation::MotionModel::ConstantVelocity;
  cfg.manoeuvre.accel_psd_factor = 1.0;
  return cfg;
}

TEST(ConsistencyMonteCarlo, MatchedModelIsConsistent) {
  const EngineConfig cfg = without_manoeuvre_switching();
  MonteCarlo mc;
  monte_carlo(cfg.accel_psd, cfg, mc);
  if (HasFatalFailure()) {
    return;
  }
  print_monte_carlo("monte_carlo matched", mc);
  // Asserted at one fixed time, the end of the run (steady state), over
  // independent runs, so each interval is exact. The intermediate times are
  // printed, not asserted: they reuse the same runs, so testing them all would
  // be many correlated tests at 95% each. Four assertions here: Bonferroni,
  // each at 1 - 0.05 / 4.
  constexpr double kP = 1.0 - 0.05 / 4.0;
  const std::size_t last = mc.step.size() - 1;
  for (const ChiSquareStat* s : {&mc.prior[last], &mc.posterior[last], &mc.posterior2[last]}) {
    EXPECT_TRUE(s->interval(kP).contains(s->average()))
        << s->average() << " outside [" << s->interval(kP).lo << ", " << s->interval(kP).hi << "]";
  }
  // Innovations of a consistent filter are white, so every update is an
  // independent sample and the pooled interval is exact (bar truncation).
  EXPECT_TRUE(mc.nis.interval(kP).contains(mc.nis.average())) << mc.nis.average();
  // Pooled over every evaluation time: printed only. The times share runs, so
  // these samples are correlated and the interval printed with them is too narrow.
  ChiSquareStat pooled;
  for (const ChiSquareStat& s : mc.posterior) {
    pooled.merge(s);
  }
  print_stat("monte_carlo matched", "NEES4 posterior, all times", pooled);
}

// Truth with no acceleration at all (what the simulator flies on straight
// legs), filter with its default q: the filter over-models process noise and
// must be pessimistic, never optimistic. This is the fixture regime in
// isolation, reproduced without single-run correlation.
TEST(ConsistencyMonteCarlo, ZeroAccelerationTruthIsPessimistic) {
  const EngineConfig cfg = without_manoeuvre_switching();
  MonteCarlo mc;
  monte_carlo(0.0, cfg, mc);
  if (HasFatalFailure()) {
    return;
  }
  print_monte_carlo("monte_carlo q_truth=0", mc);
  const std::size_t last = mc.step.size() - 1;
  for (const ChiSquareStat* s : {&mc.prior[last], &mc.posterior[last], &mc.posterior2[last]}) {
    EXPECT_LT(s->average(), s->interval().lo);  // pessimistic, beyond chance
  }
  EXPECT_LE(mc.nis.average(), mc.nis.interval().hi);
}

// The IMM on its own matched model: truth switches between the quiet,
// manoeuvre and hard modes with the IMM's transition matrix and is driven, in
// each mode, by that mode's white acceleration (up to ~3.5 g RMS in the hard
// mode). Two questions, kept apart:
//
// 1. Is the estimator consistent? The ImmFilter is driven directly, every
//    measurement applied (no gate, no association). Unlike the Kalman filter
//    above the IMM is not exact for this system (it approximates a
//    3^k-component Gaussian mixture by three moment-matched Gaussians), so the
//    chi-square intervals are not exact tests and are printed only. Asserted:
//    NEES and NIS pooled over the steady-state part of every run within 15% of
//    their expected values. A defect in mixing or combination shows up here:
//    without the spread-of-means term the posterior NEES4 is far above 4.
// 2. How often does the engine lose such a target? Its gate is the mixture
//    statistic (ADR 0014), which keeps a detection from a mode the track
//    thinks improbable only when the evidence beats that mode's prior: the
//    price of not chasing a neighbour's detections in dense traffic. Printed,
//    with a loose bound; docs/design.md section 7 records the measured rate.
TEST(ConsistencyMonteCarlo, ImmMatchedModelIsNearlyConsistent) {
  using namespace estimation;
  const ImmParams params;
  const ModeMatrix Pi = transition_matrix(params, 1.0);
  const SensorPose radar{0.0, 0.0, 0.0};
  constexpr std::size_t kRuns = 1000;
  constexpr std::size_t kFrom = 30;  // steady state
  ChiSquareStat prior;
  ChiSquareStat posterior;
  ChiSquareStat nis;
  for (std::size_t run = 0; run < kRuns; ++run) {
    net::SplitMix64 rng(run);
    Eigen::Vector4d x(3000.0, 4000.0, -60.0, 20.0);
    auto draw = [&rng](const ModeVector& p) {
      const double u = rng.uniform();
      double acc = 0.0;
      for (Eigen::Index i = 0; i + 1 < p.size(); ++i) {
        acc += p(i);
        if (u < acc) {
          return i;
        }
      }
      return p.size() - 1;
    };
    Eigen::Index mode = draw(stationary_probabilities(params));
    auto measure = [&] {
      return RangeBearing{
          std::hypot(x(0), x(1)) + kSigmaRange * gaussian(rng),
          std::remainder(std::atan2(x(1), x(0)) + kSigmaBearing * gaussian(rng), 2.0 * kPi)};
    };
    const RangeBearing sigma{kSigmaRange, kSigmaBearing};
    const LocalPosition p0 = range_bearing_to_local(measure(), sigma, radar);
    StateVector x0 = StateVector::Zero();
    x0.head<2>() = p0.position;
    StateCovariance P0 = StateCovariance::Zero();
    P0.topLeftCorner<2, 2>() = p0.covariance;
    P0(2, 2) = P0(3, 3) = 100.0 * 100.0;  // EngineConfig::initial_velocity_sigma
    ImmFilter f = ImmFilter::create(x0, P0, params).value();
    for (std::size_t k = 1; k < kMcSteps; ++k) {
      mode = draw(Pi.row(mode).transpose());
      const double q = params.accel_psd[static_cast<std::size_t>(mode)];
      const double l11 = std::sqrt(q / 3.0);
      const double l21 = l11 > 0.0 ? (q / 2.0) / l11 : 0.0;
      const double l22 = std::sqrt(std::max(0.0, q - l21 * l21));
      for (Eigen::Index axis = 0; axis < 2; ++axis) {
        const double n1 = gaussian(rng);
        const double n2 = gaussian(rng);
        x(axis) += x(2 + axis) + l11 * n1;
        x(2 + axis) += l21 * n1 + l22 * n2;
      }
      ASSERT_EQ(f.predict(1.0), FilterStatus::Ok);
      const RangeBearing z = measure();
      const auto at = range_bearing_innovation(f.state(), radar, z, sigma);
      ASSERT_TRUE(at);
      if (k >= kFrom) {
        prior.add(testing::nees(f.state() - x, f.covariance()), 4);
        nis.add(*f.mahalanobis2<2>(at->y, at->H, at->R), 2);
      }
      ASSERT_EQ(f.update<2>([&](const StateVector& s) {
        return range_bearing_innovation(s, radar, z, sigma);
      }),
                FilterStatus::Ok);
      if (k >= kFrom) {
        posterior.add(testing::nees(f.state() - x, f.covariance()), 4);
      }
    }
  }
  print_stat("imm matched (estimator)", "NEES4 prior, t >= 30 s", prior);
  print_stat("imm matched (estimator)", "NEES4 posterior, t >= 30 s", posterior);
  print_stat("imm matched (estimator)", "NIS range_bearing, t >= 30 s", nis);
  for (const ChiSquareStat* s : {&prior, &posterior, &nis}) {
    EXPECT_NEAR(s->average(), s->expected(), 0.15 * s->expected());
  }

  // 2. The engine on the same model (its default IMM).
  EngineConfig cfg;
  cfg.motion_model = MotionModel::Imm;
  MonteCarlo mc;
  monte_carlo(0.0, cfg, mc, &cfg.imm.model);
  if (HasFatalFailure()) {
    return;
  }
  print_monte_carlo("monte_carlo imm matched (engine)", mc);
  std::printf("[ scenario ] monte_carlo imm matched (engine): %zu of %zu runs lost their track\n",
              mc.lost, kMcRuns);
  EXPECT_LE(mc.lost, kMcRuns * 2 / 5);
}

// ---- fixture scenarios ------------------------------------------------------

bool all_targets_straight(const testing::Scenario& sc) {
  std::vector<int> targets;
  for (const testing::TruthRow& r : sc.truth) {
    targets.push_back(r.target);
  }
  std::sort(targets.begin(), targets.end());
  targets.erase(std::unique(targets.begin(), targets.end()), targets.end());
  // Truth velocities are written with 15+ significant digits; 1e-6 m/s is
  // rounding, not a manoeuvre.
  return std::all_of(targets.begin(), targets.end(), [&](int tgt) {
    return testing::velocity_change(sc, tgt, INT64_MIN, INT64_MAX) < 1e-6;
  });
}

void check_scenario(const std::string& scenario, estimation::MotionModel model) {
  const std::string name = scenario + " " + std::string(estimation::to_string(model));
  const testing::Scenario sc = testing::load_scenario(scenario);
  ASSERT_FALSE(sc.measurements.empty());
  EngineConfig cfg;
  cfg.motion_model = model;
  const testing::Score s = testing::run_engine(sc, cfg);
  const bool straight = all_targets_straight(sc);
  std::printf("[ scenario ] %s: consistency (%s)\n", name.c_str(),
              straight ? "straight flight: no target manoeuvres" : "manoeuvring targets");
  const testing::NeesStats* regimes[] = {&s.nees.updated, &s.nees.extrapolated};
  const char* regime_names[] = {"updated", "extrapolated"};
  for (std::size_t i = 0; i < 2; ++i) {
    print_stat(name, std::string("NEES4 ") + regime_names[i], regimes[i]->full);
    print_stat(name, std::string("NEES2 ") + regime_names[i], regimes[i]->position);
  }
  for (const auto& [kind, st] : s.nis) {
    print_stat(name, std::string("NIS ") + kind_name(kind), st);
  }
  for (const auto& [tgt, r] : s.nees_by_target) {
    ChiSquareStat all = r.updated.full;
    all.merge(r.extrapolated.full);
    print_stat(name, "target " + std::to_string(tgt) + " NEES4", all);
  }
  EXPECT_EQ(s.nis_lost, 0U);  // the trace window kept every update
  ASSERT_GT(s.nees.updated.full.n, 0U);
  ASSERT_FALSE(s.nis.empty());

  const Engine reference(sc.sensors);
  const double gate_p = reference.config().association.gate_probability;
  const double alpha = reference.config().manoeuvre.fading;
  std::vector<const ChiSquareStat*> nees;
  for (const testing::NeesStats* r : regimes) {
    for (const ChiSquareStat* st : {&r->full, &r->position}) {
      if (st->n > 0) {
        nees.push_back(st);
      }
    }
  }
  if (straight) {
    for (const ChiSquareStat* st : nees) {
      EXPECT_LE(st->average(), st->interval().hi) << name << ": optimistic in straight flight";
      EXPECT_GE(st->average(), st->expected() / 4.0) << name << ": covariance blown up";
    }
    for (const auto& [kind, st] : s.nis) {
      EXPECT_LE(st.average(), st.interval().hi) << name << " " << kind_name(kind);
      EXPECT_GE(st.average(), st.expected() / 4.0) << name << " " << kind_name(kind);
    }
    return;
  }
  for (const ChiSquareStat* st : nees) {
    const auto dof = static_cast<std::size_t>(std::lround(st->expected()));
    EXPECT_LT(st->average(), *association::chi_square_quantile(dof, gate_p))
        << name << ": truth on average outside the gate region";
  }
  // Per degree of freedom, the mean d2 at which the detector's statistic sits
  // at its switch-on threshold.
  const double per_dof = reference.manoeuvre_on_threshold() * (1.0 - alpha) / 2.0;
  for (const auto& [kind, st] : s.nis) {
    EXPECT_LT(st.average(), per_dof * st.expected())
        << name << " " << kind_name(kind) << ": manoeuvre switching not holding NIS down";
  }
}

// Both estimators: the IMM (the default) and the legacy constant-velocity
// filter, which old recordings replay with (ADR 0014).
void check_scenario(const std::string& name) {
  for (const auto m : {estimation::MotionModel::Imm, estimation::MotionModel::ConstantVelocity}) {
    check_scenario(name, m);
  }
}

TEST(ConsistencyScenario, StraightCrossing) { check_scenario("straight_crossing"); }
TEST(ConsistencyScenario, Turn) { check_scenario("turn"); }
TEST(ConsistencyScenario, CrossingPair) { check_scenario("crossing_pair"); }
TEST(ConsistencyScenario, ClosePass) { check_scenario("close_pass"); }
TEST(ConsistencyScenario, EnterLeave) { check_scenario("enter_leave"); }
TEST(ConsistencyScenario, FormationSplit) { check_scenario("formation_split"); }
TEST(ConsistencyScenario, Demo) { check_scenario("demo"); }

}  // namespace
}  // namespace sifted
