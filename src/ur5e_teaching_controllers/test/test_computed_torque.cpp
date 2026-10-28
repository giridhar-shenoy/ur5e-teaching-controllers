// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Computed-torque control on the UR5e model, with either backend as the controller's model and a
// Pinocchio forward-dynamics plant: with an exact model every joint's error must follow its own
// critically damped second-order response, decoupled from the other joints.
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "control/computed_torque.hpp"
#include "dynamics/dynamics_model.hpp"

using ur5e_teaching_controllers::control::computed_torque;
using ur5e_teaching_controllers::control::computed_torque_gains;
using ur5e_teaching_controllers::dynamics::DynamicsModel;
using ur5e_teaching_controllers::dynamics::make_dynamics_model;

namespace
{
const std::vector<std::string> kJoints{"shoulder_pan_joint", "shoulder_lift_joint",
                                       "elbow_joint",        "wrist_1_joint",
                                       "wrist_2_joint",      "wrist_3_joint"};
constexpr double kDt = 0.002;  // controller period
constexpr int kSubsteps = 2;   // 1 ms physics steps with the torque held, as in Gazebo

std::string read_urdf()
{
  std::ifstream f(UR5E_TEST_URDF);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

Eigen::VectorXd home()
{
  Eigen::VectorXd q(6);
  q << 0.0, -1.57, 1.57, -1.57, -1.57, 0.0;
  return q;
}

/// Rigid-body plant: M(q) qdd + C(q, qd) qd + g(q) = tau, semi-implicit Euler.
struct Plant
{
  std::unique_ptr<DynamicsModel> model;
  Eigen::VectorXd q, qd;
  void step(const Eigen::VectorXd & tau, double dt)
  {
    const Eigen::MatrixXd m = model->mass_matrix(q);
    const Eigen::VectorXd nle = model->nonlinear_effects(q, qd);
    qd += m.ldlt().solve(tau - nle) * dt;
    q += qd * dt;
  }
};

struct Gains
{
  Eigen::VectorXd omega, zeta, kp, kd;
};

Gains default_gains()
{
  Gains g;
  g.omega.resize(6);
  g.omega << 20.0, 20.0, 20.0, 30.0, 30.0, 30.0;  // controllers.yaml
  g.zeta = Eigen::VectorXd::Ones(6);
  computed_torque_gains(g.omega, g.zeta, g.kp, g.kd);
  return g;
}
}  // namespace

class ComputedTorqueTest : public ::testing::TestWithParam<std::string>
{
protected:
  std::string urdf_{read_urdf()};
  std::unique_ptr<DynamicsModel> model_{make_dynamics_model(GetParam(), urdf_, kJoints, "tool0")};
  Gains gains_{default_gains()};
  Eigen::VectorXd a_{Eigen::VectorXd::Zero(6)}, tau_model_{Eigen::VectorXd::Zero(6)},
    tau_{Eigen::VectorXd::Zero(6)};
};

TEST(ComputedTorqueGains, PolesAtOmegaWithDampingZeta)
{
  Eigen::VectorXd omega(2), zeta(2), kp, kd;
  omega << 20.0, 30.0;
  zeta << 1.0, 0.7;
  computed_torque_gains(omega, zeta, kp, kd);
  EXPECT_DOUBLE_EQ(kp[0], 400.0);
  EXPECT_DOUBLE_EQ(kd[0], 40.0);
  EXPECT_DOUBLE_EQ(kp[1], 900.0);
  EXPECT_DOUBLE_EQ(kd[1], 42.0);
}

TEST_P(ComputedTorqueTest, HoldingWithoutErrorCommandsGravity)
{
  const Eigen::VectorXd q = home();
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  computed_torque(
    *model_, q, zero, q, zero, zero, gains_.kp, gains_.kd, model_->effort_limits(), a_, tau_model_,
    tau_);
  EXPECT_TRUE(a_.isZero(0.0));
  EXPECT_TRUE(tau_.isApprox(model_->gravity(q), 1e-12));
}

TEST_P(ComputedTorqueTest, ErrorDecaysAsADecoupledCriticallyDampedSystem)
{
  // Start 0.05 rad off on every joint (signs alternate), hold q_des, run 1 s of closed loop.
  // Each joint must follow e(t) = e0 (1 + w t) exp(-w t) with its own w.
  Plant plant{make_dynamics_model("pinocchio", urdf_, kJoints, "tool0"), home(), {}};
  plant.qd = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd e0(6);
  e0 << 0.05, -0.05, 0.05, -0.05, 0.05, -0.05;
  const Eigen::VectorXd q_des = home();
  plant.q = q_des - e0;
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);

  double worst = 0.0;
  for (int k = 1; k <= 500; ++k) {
    computed_torque(
      *model_, plant.q, plant.qd, q_des, zero, zero, gains_.kp, gains_.kd, model_->effort_limits(),
      a_, tau_model_, tau_);
    for (int s = 0; s < kSubsteps; ++s) {
      plant.step(tau_, kDt / kSubsteps);
    }
    const double t = k * kDt;
    for (int j = 0; j < 6; ++j) {
      const double w = gains_.omega[j];
      const double expected = e0[j] * (1.0 + w * t) * std::exp(-w * t);
      const double e = q_des[j] - plant.q[j];
      worst = std::max(worst, std::abs(e - expected) / std::abs(e0[j]));
    }
  }
  std::cout << "[computed_torque] " << GetParam()
            << ": worst deviation from the continuous-time response " << worst << " x |e0|"
            << std::endl;
  // A 2 ms zero-order hold on poles at 20-30 rad/s (w dt <= 0.06) shifts the response by a few
  // percent of e0; coupling through a wrong M or C would show up as tens of percent.
  EXPECT_LT(worst, 0.05);
  EXPECT_LT((q_des - plant.q).cwiseAbs().maxCoeff(), 1e-5);
}

TEST_P(ComputedTorqueTest, TracksAMovingReferenceWithTheExactModel)
{
  // Sinusoidal reference on every joint starting at rest; with an exact model and a matching
  // initial state the only error left comes from holding the torque over the 2 ms period
  // (a Python replica of this loop gives ~0.1 mrad).
  Plant plant{make_dynamics_model("pinocchio", urdf_, kJoints, "tool0"), home(), {}};
  plant.qd = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd amp(6);
  amp << 0.4, 0.3, 0.3, 0.5, 0.5, 0.6;
  const double w_ref = 2.0;  // rad/s; peak qd 1.2 rad/s, peak qdd 2.4 rad/s^2
  double max_err = 0.0;
  for (int k = 0; k < 1500; ++k) {
    const double t = k * kDt;
    // q_des = home + amp (1 - cos(w t)): starts at rest at home
    const Eigen::VectorXd q_des = home() + amp * (1.0 - std::cos(w_ref * t));
    const Eigen::VectorXd qd_des = amp * (w_ref * std::sin(w_ref * t));
    const Eigen::VectorXd qdd_des = amp * (w_ref * w_ref * std::cos(w_ref * t));
    // Error at the sampling instant, as the controller (and bench_tracking) see it.
    max_err = std::max(max_err, (q_des - plant.q).cwiseAbs().maxCoeff());
    computed_torque(
      *model_, plant.q, plant.qd, q_des, qd_des, qdd_des, gains_.kp, gains_.kd,
      model_->effort_limits(), a_, tau_model_, tau_);
    for (int s = 0; s < kSubsteps; ++s) {
      plant.step(tau_, kDt / kSubsteps);
    }
  }
  std::cout << "[computed_torque] " << GetParam() << ": max |e| on the sinusoid " << max_err
            << " rad" << std::endl;
  EXPECT_LT(max_err, 2e-4);
}

TEST_P(ComputedTorqueTest, CommandIsSaturatedAtTheTorqueLimit)
{
  const Eigen::VectorXd q = home();
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  const Eigen::VectorXd q_des = q + Eigen::VectorXd::Constant(6, 1.0);  // 1 rad error everywhere
  const Eigen::VectorXd limit = Eigen::VectorXd::Constant(6, 5.0);
  computed_torque(
    *model_, q, zero, q_des, zero, zero, gains_.kp, gains_.kd, limit, a_, tau_model_, tau_);
  EXPECT_GT(tau_model_.cwiseAbs().maxCoeff(), 5.0);
  EXPECT_LE(tau_.cwiseAbs().maxCoeff(), 5.0);
  for (int j = 0; j < 6; ++j) {
    EXPECT_DOUBLE_EQ(tau_[j], std::clamp(tau_model_[j], -5.0, 5.0));
  }
}

INSTANTIATE_TEST_SUITE_P(
  BothBackends, ComputedTorqueTest, ::testing::Values("pinocchio", "kdl"),
  [](const ::testing::TestParamInfo<std::string> & info) { return info.param; });
