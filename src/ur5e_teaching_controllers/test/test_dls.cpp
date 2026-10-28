// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Damped least squares with manipulability-scheduled damping (control/dls.hpp), on the UR5e
// Jacobian from the URDF: exact away from singularities, bounded joint speed near one.
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "control/dls.hpp"
#include "dynamics/pinocchio_model.hpp"

using ur5e_teaching_controllers::control::damped_least_squares;
using ur5e_teaching_controllers::control::damping_squared;
using ur5e_teaching_controllers::control::limit_scale;
using ur5e_teaching_controllers::control::manipulability;
using ur5e_teaching_controllers::control::Matrix6;
using ur5e_teaching_controllers::control::uniform_scale;
using ur5e_teaching_controllers::control::Vector6;
using ur5e_teaching_controllers::dynamics::PinocchioModel;

namespace
{
const std::vector<std::string> kJoints{"shoulder_pan_joint", "shoulder_lift_joint",
                                       "elbow_joint",        "wrist_1_joint",
                                       "wrist_2_joint",      "wrist_3_joint"};
// The shipped schedule (tcp_jog_controller in controllers.yaml).
constexpr double kW0 = 0.02;
constexpr double kLambdaMax = 0.05;

std::string read_urdf()
{
  std::ifstream f(UR5E_TEST_URDF);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

Eigen::VectorXd pose(double wrist_2)
{
  Eigen::VectorXd q(6);
  q << 0.0, -1.57, 1.57, -1.57, wrist_2, 0.0;
  return q;
}
}  // namespace

class DlsTest : public ::testing::Test
{
protected:
  PinocchioModel model_{read_urdf(), kJoints, "tool0"};
  Matrix6 jacobian(const Eigen::VectorXd & q) { return Matrix6(model_.tip_jacobian(q)); }
};

TEST_F(DlsTest, ManipulabilityIsSqrtDetJJtAndDropsAtTheWristSingularity)
{
  const Matrix6 j = jacobian(pose(-1.57));
  EXPECT_NEAR(manipulability(j), std::sqrt((j * j.transpose()).determinant()), 1e-12);
  // wrist_2 = 0 aligns the wrist_1 and wrist_3 axes: a wrist singularity.
  EXPECT_GT(manipulability(j), kW0);
  EXPECT_LT(manipulability(jacobian(pose(-0.05))), 0.25 * kW0);
  EXPECT_LT(manipulability(jacobian(pose(0.0))), 1e-9);
}

TEST(DampingSchedule, ZeroAboveThresholdFullAtZeroAndMonotone)
{
  EXPECT_DOUBLE_EQ(damping_squared(kW0, kW0, kLambdaMax), 0.0);
  EXPECT_DOUBLE_EQ(damping_squared(2.0 * kW0, kW0, kLambdaMax), 0.0);
  EXPECT_DOUBLE_EQ(damping_squared(0.0, kW0, kLambdaMax), kLambdaMax * kLambdaMax);
  double previous = kLambdaMax * kLambdaMax;
  for (double w = 0.001; w < kW0; w += 0.001) {
    const double l2 = damping_squared(w, kW0, kLambdaMax);
    EXPECT_LE(l2, previous);
    previous = l2;
  }
  EXPECT_DOUBLE_EQ(damping_squared(0.0, 0.0, kLambdaMax), 0.0);  // w0 = 0 disables damping
}

TEST_F(DlsTest, WithoutDampingItIsTheExactInverse)
{
  const Matrix6 j = jacobian(pose(-1.57));
  Vector6 v;
  v << 0.05, -0.02, 0.03, 0.1, -0.2, 0.3;
  const Vector6 qd = damped_least_squares(j, v, 0.0);
  EXPECT_TRUE((j * qd).isApprox(v, 1e-9));
  EXPECT_TRUE(qd.isApprox(j.inverse() * v, 1e-9));
}

TEST_F(DlsTest, NearTheSingularityDampingBoundsJointSpeed)
{
  // Near the wrist singularity, a 0.05 (m/s, rad/s) twist along the Jacobian's weakest output
  // direction (left singular vector of the smallest singular value): the one direction the arm
  // can barely produce. A Cartesian axis need not be that direction (base x here needs only
  // ~0.17 rad/s), so the test picks it from the SVD.
  const Matrix6 j = jacobian(pose(-0.02));
  const Eigen::JacobiSVD<Matrix6> svd(j, Eigen::ComputeFullU);
  const Vector6 v = 0.05 * svd.matrixU().col(5);
  const Vector6 qd_exact = damped_least_squares(j, v, 0.0);
  const double l2 = damping_squared(manipulability(j), kW0, kLambdaMax);
  ASSERT_GT(l2, 0.0);
  const Vector6 qd_dls = damped_least_squares(j, v, l2);
  // ||J^T (J J^T + l^2 I)^-1|| <= 1 / (2 l): the classic DLS bound.
  EXPECT_LE(qd_dls.norm(), v.norm() / (2.0 * std::sqrt(l2)) + 1e-12);
  EXPECT_GT(qd_exact.norm(), 3.0 * qd_dls.norm());
  std::cout << "[dls] near the wrist singularity: |qd| exact " << qd_exact.norm()
            << " rad/s, damped " << qd_dls.norm() << " rad/s, TCP error " << (j * qd_dls - v).norm()
            << " m/s" << std::endl;
}

TEST(UniformScale, KeepsDirectionAndRespectsEveryLimit)
{
  Vector6 x;
  x << 2.0, -0.5, 0.1, 3.0, 0.0, -1.0;
  const Vector6 limit = Vector6::Constant(1.0);
  const double s = uniform_scale(x, limit);
  EXPECT_DOUBLE_EQ(s, 1.0 / 3.0);
  EXPECT_LE((s * x).cwiseAbs().maxCoeff(), 1.0 + 1e-15);
  EXPECT_DOUBLE_EQ(uniform_scale(0.5 * Vector6::Ones(), limit), 1.0);
}

TEST(LimitScale, StopsAtTheMarginOnlyForJointsMovingTowardsALimit)
{
  const Vector6 lower = Vector6::Constant(-1.0);
  const Vector6 upper = Vector6::Constant(1.0);
  Vector6 q = Vector6::Zero();
  q[0] = 0.85;  // 0.05 inside the 0.1 margin band's edge at 0.9
  Vector6 step = Vector6::Zero();
  step[0] = 0.1;
  step[1] = -0.1;
  EXPECT_DOUBLE_EQ(limit_scale(q, step, lower, upper, 0.1), 0.5);
  step[0] = -0.1;  // moving away from the upper limit: unrestricted
  EXPECT_DOUBLE_EQ(limit_scale(q, step, lower, upper, 0.1), 1.0);
  q[0] = 0.95;  // already inside the margin and moving further out: stop
  step[0] = 0.1;
  EXPECT_DOUBLE_EQ(limit_scale(q, step, lower, upper, 0.1), 0.0);
}
