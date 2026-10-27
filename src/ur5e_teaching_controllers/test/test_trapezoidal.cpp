// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "profiles/trapezoidal.hpp"

using ur5e_teaching_controllers::profiles::JointSample;
using ur5e_teaching_controllers::profiles::min_duration;
using ur5e_teaching_controllers::profiles::sample_segment;
using ur5e_teaching_controllers::profiles::TrapezoidalTrajectory;

namespace
{
constexpr double kTol = 1e-9;

TrapezoidalTrajectory two_joint(
  const std::vector<double> & goal, double t, double v_max = 1.0, double a_max = 2.0)
{
  return TrapezoidalTrajectory({0.0, 0.0}, {goal}, {t}, {v_max, v_max}, {a_max, a_max});
}
}  // namespace

TEST(Trapezoidal, EndpointsExactAndAtRest)
{
  const double d = 0.8;
  const double T = 2.0;
  const auto s0 = sample_segment(d, T, 0.0);
  const auto s1 = sample_segment(d, T, T);
  EXPECT_NEAR(s0.position, 0.0, kTol);
  EXPECT_NEAR(s0.velocity, 0.0, kTol);
  EXPECT_NEAR(s1.position, d, kTol);
  EXPECT_NEAR(s1.velocity, 0.0, kTol);
}

TEST(Trapezoidal, PositionAndVelocityContinuous)
{
  // Velocity is the derivative of position and neither jumps (acceleration may, by design).
  const double d = -1.3;
  const double T = 1.7;
  const double h = 1e-6;
  for (double t = h; t < T - h; t += T / 997.0) {
    const auto a = sample_segment(d, T, t - h);
    const auto b = sample_segment(d, T, t + h);
    const auto m = sample_segment(d, T, t);
    EXPECT_NEAR((b.position - a.position) / (2 * h), m.velocity, 1e-5) << "t=" << t;
    EXPECT_NEAR(b.velocity, a.velocity, 1e-4) << "t=" << t;
    EXPECT_NEAR(b.position, a.position, 1e-5) << "t=" << t;
  }
}

TEST(Trapezoidal, VelocityAndAccelerationLimitsRespected)
{
  const double v_max = 0.7;
  const double a_max = 1.1;
  auto traj = two_joint({2.0, -0.5}, 0.1, v_max, a_max);  // requested time far too short
  std::vector<JointSample> s;
  for (double t = 0.0; t <= traj.duration(); t += 1e-3) {
    traj.sample(t, s);
    for (const auto & js : s) {
      EXPECT_LE(std::abs(js.velocity), v_max + 1e-9);
      EXPECT_LE(std::abs(js.acceleration), a_max + 1e-9);
    }
  }
}

TEST(Trapezoidal, RequestedDurationKeptWhenLimitsDoNotBind)
{
  auto traj = two_joint({0.3, 0.1}, 3.0);
  EXPECT_NEAR(traj.duration(), 3.0, kTol);
}

TEST(Trapezoidal, DurationStretchedToMinimumWhenLimitsBind)
{
  // D = 2: velocity-bound T = 1.5*2/1 = 3.0, acceleration-bound T = sqrt(4.5*2/2) = 2.121
  EXPECT_NEAR(min_duration(2.0, 1.0, 2.0), 3.0, kTol);
  // D = 2, a_max = 0.5: acceleration-bound T = sqrt(18) = 4.243 > 3.0
  EXPECT_NEAR(min_duration(2.0, 1.0, 0.5), std::sqrt(18.0), kTol);
  auto traj = two_joint({2.0, 0.0}, 0.5);
  EXPECT_NEAR(traj.duration(), 3.0, kTol);
}

TEST(Trapezoidal, JointsAreSynchronised)
{
  // Joint 0 needs much longer than joint 1; both must arrive at the same time and at rest.
  auto traj = two_joint({2.0, 0.05}, 0.5);
  std::vector<JointSample> s;
  traj.sample(traj.duration() * 0.999, s);
  EXPECT_GT(std::abs(s[0].velocity), 0.0);
  EXPECT_GT(std::abs(s[1].velocity), 0.0);
  traj.sample(traj.duration(), s);
  EXPECT_NEAR(s[0].position, 2.0, kTol);
  EXPECT_NEAR(s[1].position, 0.05, kTol);
  EXPECT_NEAR(s[0].velocity, 0.0, kTol);
  EXPECT_NEAR(s[1].velocity, 0.0, kTol);
}

TEST(Trapezoidal, ZeroDistanceHoldsStill)
{
  TrapezoidalTrajectory traj({0.4}, {{0.4}}, {1.0}, {1.0}, {1.0});
  std::vector<JointSample> s;
  for (double t = 0.0; t <= 1.0; t += 0.1) {
    traj.sample(t, s);
    EXPECT_NEAR(s[0].position, 0.4, kTol);
    EXPECT_NEAR(s[0].velocity, 0.0, kTol);
  }
}

TEST(Trapezoidal, MultiWaypointPassesThroughWaypointsAtRest)
{
  TrapezoidalTrajectory traj({0.0}, {{1.0}, {-0.5}, {0.25}}, {2.0, 4.0, 6.0}, {2.0}, {4.0});
  ASSERT_EQ(traj.segment_count(), 3u);
  std::vector<JointSample> s;
  traj.sample(2.0, s);
  EXPECT_NEAR(s[0].position, 1.0, 1e-9);
  EXPECT_NEAR(s[0].velocity, 0.0, 1e-9);
  traj.sample(4.0, s);
  EXPECT_NEAR(s[0].position, -0.5, 1e-9);
  traj.sample(100.0, s);  // after the end: hold the last waypoint at rest
  EXPECT_NEAR(s[0].position, 0.25, kTol);
  EXPECT_NEAR(s[0].velocity, 0.0, kTol);
  EXPECT_NEAR(s[0].acceleration, 0.0, kTol);
}

TEST(Trapezoidal, NegativeMoveIsMirrorOfPositive)
{
  for (double t = 0.0; t <= 2.0; t += 0.05) {
    const auto p = sample_segment(0.9, 2.0, t);
    const auto n = sample_segment(-0.9, 2.0, t);
    EXPECT_NEAR(p.position, -n.position, kTol);
    EXPECT_NEAR(p.velocity, -n.velocity, kTol);
    EXPECT_NEAR(p.acceleration, -n.acceleration, kTol);
  }
}

TEST(Trapezoidal, InvalidInputsThrow)
{
  EXPECT_THROW(min_duration(1.0, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(
    TrapezoidalTrajectory({0.0}, {{1.0}}, {1.0, 2.0}, {1.0}, {1.0}), std::invalid_argument);
  EXPECT_THROW(
    TrapezoidalTrajectory({0.0, 0.0}, {{1.0}}, {1.0}, {1.0, 1.0}, {1.0, 1.0}),
    std::invalid_argument);
  EXPECT_THROW(
    TrapezoidalTrajectory({0.0}, {{1.0}, {2.0}}, {2.0, 1.0}, {1.0}, {1.0}), std::invalid_argument);
}

TEST(Trapezoidal, NonFiniteInputsAreRejected)
{
  // std::max(T, NaN) would silently keep T, so NaN must be caught explicitly.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(TrapezoidalTrajectory({0.0}, {{nan}}, {1.0}, {1.0}, {1.0}), std::invalid_argument);
  EXPECT_THROW(TrapezoidalTrajectory({0.0}, {{inf}}, {1.0}, {1.0}, {1.0}), std::invalid_argument);
  EXPECT_THROW(TrapezoidalTrajectory({nan}, {{1.0}}, {1.0}, {1.0}, {1.0}), std::invalid_argument);
  EXPECT_THROW(TrapezoidalTrajectory({0.0}, {{1.0}}, {nan}, {1.0}, {1.0}), std::invalid_argument);
  const TrapezoidalTrajectory ok({0.25}, {{1.0}}, {1.0}, {1.0}, {1.0});
  ASSERT_EQ(ok.start().size(), 1u);
  EXPECT_DOUBLE_EQ(ok.start()[0], 0.25);
}
