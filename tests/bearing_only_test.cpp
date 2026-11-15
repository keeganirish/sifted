// Bearing-only EKF update and the shared measurement-model layer.

#include "sifted/estimation/bearing_only.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "sifted/estimation/measurement_model.hpp"
#include "sifted/estimation/range_bearing.hpp"

namespace sifted::estimation {
namespace {

constexpr double kPi = 3.14159265358979323846;

StateVector state(double px, double py, double vx, double vy) {
  StateVector x;
  x(0) = px;
  x(1) = py;
  x(2) = vx;
  x(3) = vy;
  return x;
}

ConstantVelocityFilter make(const StateVector& x, double pos_var) {
  StateCovariance P = StateCovariance::Zero();
  P(0, 0) = pos_var;
  P(1, 1) = pos_var;
  P(2, 2) = 100.0;
  P(3, 3) = 100.0;
  return ConstantVelocityFilter::create(x, P, 1.0).value();
}

}  // namespace

TEST(BearingOnly, PredictionMatchesGeometryAndYaw) {
  const SensorPose pose{100.0, -50.0, 0.5};
  const StateVector x = state(1100.0, 950.0, 0.0, 0.0);  // 45 deg from the sensor
  EXPECT_NEAR(predict_bearing(x, pose), kPi / 4 - 0.5, 1e-12);
}

TEST(BearingOnly, JacobianMatchesFiniteDifferences) {
  const SensorPose pose{-300.0, 200.0, -1.2};
  const StateVector x = state(2500.0, -1800.0, 30.0, 5.0);
  const auto H = bearing_jacobian(x, pose);
  const double h = 1e-3;
  for (int i = 0; i < 2; ++i) {
    StateVector xp = x;
    StateVector xm = x;
    xp(i) += h;
    xm(i) -= h;
    const double fd = wrap_angle(predict_bearing(xp, pose) - predict_bearing(xm, pose)) / (2 * h);
    EXPECT_NEAR(H(0, i), fd, 1e-9);
  }
  EXPECT_EQ(H(0, 2), 0.0);
  EXPECT_EQ(H(0, 3), 0.0);
}

TEST(BearingOnly, InnovationIsWrappedAcrossTheSeam) {
  const SensorPose pose{0.0, 0.0, 0.0};
  const StateVector x = state(-5000.0, 1.0, 0.0, 0.0);  // just above the -x axis: bearing ~ +pi
  const auto l = bearing_innovation(x, pose, -kPi + 0.001, 0.003);
  ASSERT_TRUE(l.has_value());
  EXPECT_LT(std::abs(l->y(0)), 0.01);  // a small correction, not ~2 pi
}

TEST(BearingOnly, UpdateShrinksCrossRangeButNotRangeUncertainty) {
  const SensorPose pose{0.0, 0.0, 0.0};
  ConstantVelocityFilter f = make(state(5000.0, 0.0, 0.0, 0.0), 400.0);
  ASSERT_EQ(update_bearing_only(f, pose, 0.002, 0.001), FilterStatus::Ok);
  // Target on +x: y is cross-range, x is range.
  EXPECT_LT(f.covariance()(1, 1), 0.2 * 400.0);
  EXPECT_NEAR(f.covariance()(0, 0), 400.0, 1e-6);
  EXPECT_GT(f.state()(1), 0.0);  // pulled towards the measured bearing
}

TEST(BearingOnly, RefusesDegenerateInputWithoutChangingState) {
  const SensorPose pose{0.0, 0.0, 0.0};
  ConstantVelocityFilter f = make(state(0.5, 0.0, 0.0, 0.0), 1.0);  // 0.5 m from the sensor
  const StateVector x0 = f.state();
  EXPECT_EQ(update_bearing_only(f, pose, 0.1, 0.01), FilterStatus::InvalidInput);
  ConstantVelocityFilter g = make(state(1000.0, 0.0, 0.0, 0.0), 1.0);
  EXPECT_EQ(update_bearing_only(g, pose, std::numeric_limits<double>::quiet_NaN(), 0.01),
            FilterStatus::InvalidInput);
  EXPECT_EQ(update_bearing_only(g, pose, 0.1, 0.0), FilterStatus::InvalidInput);
  EXPECT_TRUE(f.state() == x0);
}

TEST(MeasurementModel, CrossingBearingsFromTwoSensorsLocateTheTarget) {
  // Range is unobservable from one bearing, but two separated cameras fix it.
  const SensorPose a{0.0, 0.0, 0.0};
  const SensorPose b{4000.0, 0.0, 0.0};
  const StateVector truth = state(2000.0, 3000.0, 0.0, 0.0);
  ConstantVelocityFilter f = make(state(2400.0, 2500.0, 0.0, 0.0), 1.0e6);
  for (int k = 0; k < 4; ++k) {
    ASSERT_EQ(update_bearing_only(f, a, predict_bearing(truth, a), 0.001), FilterStatus::Ok);
    ASSERT_EQ(update_bearing_only(f, b, predict_bearing(truth, b), 0.001), FilterStatus::Ok);
  }
  EXPECT_NEAR(f.state()(0), 2000.0, 20.0);
  EXPECT_NEAR(f.state()(1), 3000.0, 20.0);
}

TEST(MeasurementModel, MahalanobisDistanceIsConsistent) {
  ConstantVelocityFilter f = make(state(1000.0, 0.0, 0.0, 0.0), 100.0);
  Linearized<2> l;
  l.H = Eigen::Matrix<double, 2, kStateDim>::Zero();
  l.H(0, 0) = 1.0;
  l.H(1, 1) = 1.0;
  l.R = Eigen::Matrix2d::Zero();
  l.R(0, 0) = 300.0;
  l.R(1, 1) = 300.0;
  l.y(0) = 40.0;  // S = 400 I, so d2 = (40^2 + 20^2) / 400 = 5
  l.y(1) = 20.0;
  const auto d2 = mahalanobis2(f, AnyLinearized{l});
  ASSERT_TRUE(d2.has_value());
  EXPECT_NEAR(*d2, 5.0, 1e-12);
  EXPECT_EQ(dimension(AnyLinearized{l}), 2U);
  const StateVector before = f.state();
  (void)f.mahalanobis2<2>(l.y, l.H, l.R);
  EXPECT_TRUE(f.state() == before);  // read only
}

}  // namespace sifted::estimation
