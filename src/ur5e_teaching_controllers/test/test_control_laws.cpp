// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "control/control_laws.hpp"

using ur5e_teaching_controllers::control::first_speed_violation;
using ur5e_teaching_controllers::control::limit_repulsion;
using ur5e_teaching_controllers::control::Pid;

TEST(Limits, RepulsionIsZeroInTheSafeInterior)
{
  EXPECT_DOUBLE_EQ(limit_repulsion(0.0, -1.0, 1.0, 0.2, 50.0), 0.0);
  EXPECT_DOUBLE_EQ(limit_repulsion(0.79, -1.0, 1.0, 0.2, 50.0), 0.0);
  EXPECT_DOUBLE_EQ(limit_repulsion(-0.79, -1.0, 1.0, 0.2, 50.0), 0.0);
}

TEST(Limits, RepulsionPushesAwayFromUpperLimitProportionally)
{
  // 0.1 rad into the 0.2 rad margin below the upper limit at 1.0 -> -50 * 0.1
  EXPECT_NEAR(limit_repulsion(0.9, -1.0, 1.0, 0.2, 50.0), -5.0, 1e-12);
  EXPECT_NEAR(limit_repulsion(1.0, -1.0, 1.0, 0.2, 50.0), -10.0, 1e-12);
}

TEST(Limits, RepulsionPushesAwayFromLowerLimit)
{
  EXPECT_NEAR(limit_repulsion(-0.95, -1.0, 1.0, 0.2, 50.0), 7.5, 1e-12);
}

TEST(Limits, RepulsionDisabledForInfiniteLimitsOrZeroGains)
{
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_DOUBLE_EQ(limit_repulsion(100.0, -inf, inf, 0.2, 50.0), 0.0);
  EXPECT_DOUBLE_EQ(limit_repulsion(0.95, -1.0, 1.0, 0.0, 50.0), 0.0);
  EXPECT_DOUBLE_EQ(limit_repulsion(0.95, -1.0, 1.0, 0.2, 0.0), 0.0);
}

TEST(Limits, SpeedCapReportsFirstViolatingJoint)
{
  const std::vector<double> cap{1.0, 1.0, 2.0};
  EXPECT_EQ(first_speed_violation({0.5, -0.9, 1.9}, cap), -1);
  EXPECT_EQ(first_speed_violation({0.5, -1.2, 2.5}, cap), 1);
  EXPECT_EQ(first_speed_violation({0.5, 0.0, -2.01}, cap), 2);
  // A cap of 0 disables the check for that joint.
  EXPECT_EQ(first_speed_violation({5.0}, {0.0}), -1);
}

TEST(Limits, TorqueSaturationIsSymmetric)
{
  EXPECT_DOUBLE_EQ(Pid::saturate(200.0, 150.0), 150.0);
  EXPECT_DOUBLE_EQ(Pid::saturate(-200.0, 150.0), -150.0);
  EXPECT_DOUBLE_EQ(Pid::saturate(12.0, 150.0), 12.0);
  EXPECT_DOUBLE_EQ(Pid::saturate(1e6, 0.0), 1e6);  // limit <= 0 disables saturation
}

TEST(Pid, ProportionalDerivativeAndFeedforwardAdd)
{
  Pid pid{10.0, 0.0, 2.0, 0.0, 0.0};
  EXPECT_NEAR(pid.update(0.5, -1.0, 0.002, 3.0, 0.0), 10.0 * 0.5 - 2.0 + 3.0, 1e-12);
}

TEST(Pid, IntegralIsClampedByIClamp)
{
  Pid pid{0.0, 100.0, 0.0, 5.0, 0.0};
  double u = 0.0;
  for (int k = 0; k < 10000; ++k) {
    u = pid.update(1.0, 0.0, 0.002, 0.0, 0.0);
  }
  EXPECT_NEAR(u, 5.0, 1e-9);
}

TEST(Pid, NoWindupWhileSaturated)
{
  // Saturated for a long time, then the error reverses: the output must respond at once
  // instead of waiting for a wound-up integral to unwind.
  Pid pid{10.0, 50.0, 0.0, 1000.0, 0.0};
  for (int k = 0; k < 5000; ++k) {
    EXPECT_NEAR(pid.update(10.0, 0.0, 0.002, 0.0, 20.0), 20.0, 1e-12);
  }
  EXPECT_LT(pid.integral * pid.ki, 20.0 + 1e-9);
  const double u = pid.update(-1.0, 0.0, 0.002, 0.0, 20.0);
  EXPECT_LT(u, 20.0);
}
