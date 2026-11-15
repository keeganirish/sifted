#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "sifted/estimation/cv_filter.hpp"
#include "sifted/estimation/range_bearing.hpp"

namespace sifted::estimation {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

StateVector state(double px, double py, double vx, double vy) {
  StateVector x;
  x(0) = px;
  x(1) = py;
  x(2) = vx;
  x(3) = vy;
  return x;
}

StateCovariance diag(double pos_var, double vel_var) {
  StateCovariance P = StateCovariance::Zero();
  P(0, 0) = pos_var;
  P(1, 1) = pos_var;
  P(2, 2) = vel_var;
  P(3, 3) = vel_var;
  return P;
}

ConstantVelocityFilter make(const StateVector& x, const StateCovariance& P, double q = 1.0) {
  const auto r = ConstantVelocityFilter::create(x, P, q);
  EXPECT_TRUE(r.ok());
  return r.value();
}

Eigen::Matrix2d iso2(double var) {
  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = var;
  R(1, 1) = var;
  return R;
}

bool same(const ConstantVelocityFilter& a, const ConstantVelocityFilter& b) {
  return a.state() == b.state() && a.covariance() == b.covariance();
}

}  // namespace

// ---- linear constant-velocity filter -------------------------------------

TEST(CvFilter, CreateValidatesInputs) {
  EXPECT_TRUE(ConstantVelocityFilter::create(state(0, 0, 0, 0), diag(1, 1), 1.0).ok());
  EXPECT_FALSE(ConstantVelocityFilter::create(state(kNaN, 0, 0, 0), diag(1, 1), 1.0).ok());
  EXPECT_FALSE(ConstantVelocityFilter::create(state(0, 0, 0, 0), diag(1, -1), 1.0).ok());
  EXPECT_FALSE(ConstantVelocityFilter::create(state(0, 0, 0, 0), diag(1, 1), -1.0).ok());
  EXPECT_FALSE(ConstantVelocityFilter::create(state(0, 0, 0, 0), diag(1, 1), kNaN).ok());
  StateCovariance asym = diag(1, 1);
  asym(0, 2) = 0.5;  // not mirrored
  EXPECT_FALSE(ConstantVelocityFilter::create(state(0, 0, 0, 0), asym, 1.0).ok());
}

TEST(CvFilter, PredictMovesStateAndMatchesClosedFormCovariance) {
  const double q = 2.0;
  const double dt = 3.0;
  ConstantVelocityFilter f = make(state(10, -5, 2, 4), diag(4, 1), q);
  ASSERT_EQ(f.predict(dt), FilterStatus::Ok);
  EXPECT_DOUBLE_EQ(f.state()(0), 16.0);
  EXPECT_DOUBLE_EQ(f.state()(1), 7.0);
  EXPECT_DOUBLE_EQ(f.state()(2), 2.0);
  // Per axis: P = F P0 F^T + Q; with P0 = diag(4, 1):
  //   P_pp = 4 + dt^2 + q dt^3/3,  P_pv = dt + q dt^2/2,  P_vv = 1 + q dt
  EXPECT_NEAR(f.covariance()(0, 0), 4 + dt * dt + q * dt * dt * dt / 3, 1e-12);
  EXPECT_NEAR(f.covariance()(0, 2), dt + q * dt * dt / 2, 1e-12);
  EXPECT_NEAR(f.covariance()(2, 2), 1 + q * dt, 1e-12);
  EXPECT_EQ(f.covariance()(0, 1), 0.0);  // axes stay independent
  EXPECT_TRUE(f.covariance() == f.covariance().transpose());
}

TEST(CvFilter, VariableTimeStepsComposeOnTheMean) {
  ConstantVelocityFilter a = make(state(0, 0, 3, -1), diag(1, 1));
  ConstantVelocityFilter b = a;
  ASSERT_EQ(a.predict(0.25), FilterStatus::Ok);
  ASSERT_EQ(a.predict(1.75), FilterStatus::Ok);
  ASSERT_EQ(b.predict(2.0), FilterStatus::Ok);
  EXPECT_NEAR((a.state() - b.state()).norm(), 0.0, 1e-12);
  // White-noise acceleration is consistent under splitting: Q(t1+t2) equals
  // F(t2) Q(t1) F(t2)^T + Q(t2).
  EXPECT_NEAR((a.covariance() - b.covariance()).cwiseAbs().maxCoeff(), 0.0, 1e-12);
}

TEST(CvFilter, PredictRejectsBadTimeStepsWithoutChangingState) {
  ConstantVelocityFilter f = make(state(1, 2, 3, 4), diag(1, 1));
  const ConstantVelocityFilter before = f;
  EXPECT_EQ(f.predict(-1.0), FilterStatus::InvalidTimeStep);
  EXPECT_EQ(f.predict(kNaN), FilterStatus::InvalidTimeStep);
  EXPECT_EQ(f.predict(std::numeric_limits<double>::infinity()), FilterStatus::InvalidTimeStep);
  EXPECT_EQ(f.predict(ConstantVelocityFilter::kMaxPredictDt * 2), FilterStatus::InvalidTimeStep);
  EXPECT_TRUE(same(f, before));
  EXPECT_EQ(f.predict(0.0), FilterStatus::Ok);
  EXPECT_TRUE(same(f, before));  // dt = 0 is an exact no-op
}

TEST(CvFilter, UpdateMatchesScalarKalmanGain) {
  // Prior var 100, measurement var 25: gain 0.8, posterior var 20.
  ConstantVelocityFilter f = make(state(0, 0, 0, 0), diag(100, 1));
  ASSERT_EQ(f.update(Eigen::Vector2d(10.0, -5.0), iso2(25.0)), FilterStatus::Ok);
  EXPECT_NEAR(f.state()(0), 8.0, 1e-12);
  EXPECT_NEAR(f.state()(1), -4.0, 1e-12);
  EXPECT_NEAR(f.covariance()(0, 0), 20.0, 1e-12);
  EXPECT_NEAR(f.covariance()(2, 2), 1.0, 1e-12);  // uncorrelated velocity untouched
}

TEST(CvFilter, UpdateRejectsBadInputWithoutChangingState) {
  ConstantVelocityFilter f = make(state(1, 2, 3, 4), diag(1, 1));
  const ConstantVelocityFilter before = f;
  EXPECT_EQ(f.update(Eigen::Vector2d(kNaN, 0.0), iso2(1.0)), FilterStatus::InvalidInput);
  EXPECT_EQ(f.update(Eigen::Vector2d(0.0, 0.0), iso2(-1.0)), FilterStatus::InvalidInput);
  EXPECT_EQ(f.update(Eigen::Vector2d(0.0, 0.0), iso2(kNaN)), FilterStatus::InvalidInput);
  EXPECT_TRUE(same(f, before));
}

TEST(CvFilter, ConvergesOnNoiselessConstantVelocityTarget) {
  ConstantVelocityFilter f = make(state(0, 0, 0, 0), diag(1e4, 1e4), 0.01);
  for (int k = 1; k <= 40; ++k) {
    ASSERT_EQ(f.predict(1.0), FilterStatus::Ok);
    ASSERT_EQ(f.update(Eigen::Vector2d(100.0 + 5.0 * k, -2.0 * k), iso2(1.0)), FilterStatus::Ok);
  }
  EXPECT_NEAR(f.state()(0), 300.0, 0.5);
  EXPECT_NEAR(f.state()(2), 5.0, 0.05);
  EXPECT_NEAR(f.state()(3), -2.0, 0.05);
  EXPECT_LT(f.covariance()(0, 0), 1.0);  // tighter than one measurement
}

// ---- range-bearing EKF ---------------------------------------------------

TEST(RangeBearing, WrapAngle) {
  EXPECT_DOUBLE_EQ(wrap_angle(0.5), 0.5);
  EXPECT_NEAR(wrap_angle(3 * kPi / 2), -kPi / 2, 1e-15);
  EXPECT_NEAR(wrap_angle(-3 * kPi / 2), kPi / 2, 1e-15);
  EXPECT_NEAR(wrap_angle(10 * kPi + 0.1), 0.1, 1e-13);
  const double w = wrap_angle(kPi + 1e-9);
  EXPECT_NEAR(w, -kPi + 1e-9, 1e-15);
}

TEST(RangeBearing, PredictionUsesPoseAndYaw) {
  const SensorPose pose{100.0, 50.0, kPi / 2};  // boresight pointing north
  const RangeBearing z = predict_range_bearing(state(100.0, 150.0, 0, 0), pose);
  EXPECT_NEAR(z.range_m, 100.0, 1e-12);
  EXPECT_NEAR(z.bearing_rad, 0.0, 1e-15);  // straight ahead
  const RangeBearing east = predict_range_bearing(state(200.0, 50.0, 0, 0), pose);
  EXPECT_NEAR(east.bearing_rad, -kPi / 2, 1e-15);  // to the right of boresight
}

TEST(RangeBearing, JacobianMatchesFiniteDifferences) {
  const SensorPose pose{-30.0, 20.0, 0.7};
  const StateVector x = state(400.0, -250.0, 5.0, 1.0);
  const auto H = range_bearing_jacobian(x, pose);
  const double h = 1e-4;
  for (int j = 0; j < kStateDim; ++j) {
    StateVector xp = x;
    StateVector xm = x;
    xp(j) += h;
    xm(j) -= h;
    const RangeBearing p = predict_range_bearing(xp, pose);
    const RangeBearing m = predict_range_bearing(xm, pose);
    EXPECT_NEAR(H(0, j), (p.range_m - m.range_m) / (2 * h), 1e-7) << "col " << j;
    EXPECT_NEAR(H(1, j), wrap_angle(p.bearing_rad - m.bearing_rad) / (2 * h), 1e-9) << "col " << j;
  }
}

TEST(RangeBearing, InverseModelRoundTrips) {
  const SensorPose pose{1000.0, -500.0, 0.5};
  const StateVector x = state(-2000.0, 3000.0, 0, 0);
  const RangeBearing z = predict_range_bearing(x, pose);
  const LocalPosition p = range_bearing_to_local(z, {10.0, 0.0035}, pose);
  EXPECT_NEAR(p.position(0), -2000.0, 1e-8);
  EXPECT_NEAR(p.position(1), 3000.0, 1e-8);
  // Covariance: 10 m along the line of sight, r * 0.0035 across it.
  const Eigen::Vector2d los = (p.position - Eigen::Vector2d(pose.x_m, pose.y_m)) / z.range_m;
  EXPECT_NEAR(los.dot(p.covariance * los), 100.0, 1e-6);
  const Eigen::Vector2d across(-los(1), los(0));
  const double cross_sd = z.range_m * 0.0035;
  EXPECT_NEAR(across.dot(p.covariance * across), cross_sd * cross_sd, 1e-6);
}

TEST(RangeBearing, UpdateAcrossBearingSeamIsASmallCorrection) {
  // Target due west of the sensor (bearing ~ +pi). The prior is just north of
  // the seam, the measurement just south of it: the true bearing difference
  // is ~0.002 rad, not ~2*pi.
  const SensorPose pose{0.0, 0.0, 0.0};
  ConstantVelocityFilter f = make(state(-5000.0, 5.0, 0, 0), diag(400.0, 100.0));
  const RangeBearing z{5000.0, -kPi + 0.001};  // point at about (-5000, -5)
  ASSERT_EQ(update_range_bearing(f, pose, z, {10.0, 0.0035}), FilterStatus::Ok);
  EXPECT_NEAR(f.state()(0), -5000.0, 1.0);
  EXPECT_LT(f.state()(1), 5.0);
  EXPECT_GT(f.state()(1), -6.0);  // moved toward -5, never flung across the plane
}

TEST(RangeBearing, UpdateConvergesToStaticTarget) {
  const SensorPose pose{1000.0, -500.0, 0.5};
  const StateVector truth = state(-2000.0, 3000.0, 0, 0);
  const RangeBearing z = predict_range_bearing(truth, pose);
  ConstantVelocityFilter f = make(state(-1900.0, 2950.0, 0, 0), diag(1e4, 1.0), 0.0);
  for (int k = 0; k < 10; ++k) {
    ASSERT_EQ(update_range_bearing(f, pose, z, {10.0, 0.0035}), FilterStatus::Ok);
  }
  EXPECT_NEAR(f.state()(0), -2000.0, 1.0);
  EXPECT_NEAR(f.state()(1), 3000.0, 1.0);
}

TEST(RangeBearing, UpdateRejectsDegenerateOrInvalidInput) {
  const SensorPose pose{0.0, 0.0, 0.0};
  ConstantVelocityFilter at_sensor = make(state(0.2, 0.0, 0, 0), diag(1, 1));
  const ConstantVelocityFilter before = at_sensor;
  EXPECT_EQ(update_range_bearing(at_sensor, pose, {5.0, 0.1}, {1.0, 0.01}),
            FilterStatus::InvalidInput);  // predicted range below kMinRangeForUpdateM
  EXPECT_TRUE(same(at_sensor, before));

  ConstantVelocityFilter f = make(state(100.0, 0.0, 0, 0), diag(1, 1));
  EXPECT_EQ(update_range_bearing(f, pose, {kNaN, 0.0}, {1.0, 0.01}), FilterStatus::InvalidInput);
  EXPECT_EQ(update_range_bearing(f, pose, {100.0, 0.0}, {0.0, 0.01}), FilterStatus::InvalidInput);
  EXPECT_EQ(update_range_bearing(f, pose, {-1.0, 0.0}, {1.0, 0.01}), FilterStatus::InvalidInput);
}

}  // namespace sifted::estimation
