// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// The two dynamics backends must be interchangeable: on the UR5e description shipped by
// ur_description, Pinocchio and KDL have to give the same gravity, RNEA, Coriolis, mass matrix,
// tip Jacobian and tip pose for random states inside the joint limits, with and without a payload.
// Tolerances are fixed by the sprint brief and are not to be loosened: a disagreement means a
// frame, axis-sign or inertia-convention bug in a wrapper.
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "dynamics/dynamics_model.hpp"
#include "dynamics/kdl_model.hpp"
#include "dynamics/pinocchio_model.hpp"

using ur5e_teaching_controllers::dynamics::DynamicsModel;
using ur5e_teaching_controllers::dynamics::KdlModel;
using ur5e_teaching_controllers::dynamics::make_dynamics_model;
using ur5e_teaching_controllers::dynamics::PinocchioModel;

namespace
{
const std::vector<std::string> kJoints{"shoulder_pan_joint", "shoulder_lift_joint",
                                       "elbow_joint",        "wrist_1_joint",
                                       "wrist_2_joint",      "wrist_3_joint"};
constexpr int kSamples = 200;
constexpr unsigned kSeed = 20261029;
// |qdd| bound for the random accelerations [rad/s^2]; the URDF has no acceleration limits. This
// is above anything the tracking profiles command (max_acceleration 4-6 rad/s^2).
constexpr double kMaxAcceleration = 10.0;

constexpr double kTorqueTol = 1e-6;    // N m, per joint
constexpr double kMassTol = 1e-6;      // per element
constexpr double kJacobianTol = 1e-9;  // per element
constexpr double kPositionTol = 1e-9;  // m and rotation-matrix elements

std::string read_urdf()
{
  std::ifstream f(UR5E_TEST_URDF);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

struct State
{
  Eigen::VectorXd q, v, a;
};

/// kSamples states: q uniform within the URDF position limits, qd within the URDF velocity
/// limits, qdd within +-kMaxAcceleration.
std::vector<State> random_states(const DynamicsModel & model)
{
  std::mt19937 rng(kSeed);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  const auto n = static_cast<Eigen::Index>(model.dof());
  std::vector<State> states(kSamples);
  for (auto & s : states) {
    s.q.resize(n);
    s.v.resize(n);
    s.a.resize(n);
    for (Eigen::Index j = 0; j < n; ++j) {
      const double lo = model.lower_limits()[j];
      const double hi = model.upper_limits()[j];
      s.q[j] = lo + (hi - lo) * 0.5 * (unit(rng) + 1.0);
      s.v[j] = model.velocity_limits()[j] * unit(rng);
      s.a[j] = kMaxAcceleration * unit(rng);
    }
  }
  return states;
}

double max_abs_diff(const Eigen::MatrixXd & x, const Eigen::MatrixXd & y)
{
  return (x - y).cwiseAbs().maxCoeff();
}

/// Largest deviation per quantity over all samples, printed so the test log carries the numbers.
struct Worst
{
  double gravity{0}, rnea{0}, nle{0}, mass{0}, jacobian{0}, position{0}, rotation{0};
  void print(const std::string & label) const
  {
    std::cout << "[equivalence] " << label << ": max |pinocchio - kdl| gravity " << gravity
              << " N m, rnea " << rnea << " N m, nle " << nle << " N m, M " << mass << ", J "
              << jacobian << ", p " << position << " m, R " << rotation << std::endl;
  }
};

/// Compare both backends on every random state; returns the worst deviations.
Worst compare(
  DynamicsModel & pin, DynamicsModel & kdl, const std::vector<State> & states,
  const std::string & label)
{
  Worst w;
  int i = 0;
  for (const auto & s : states) {
    SCOPED_TRACE(label + ", sample " + std::to_string(i++));
    const Eigen::VectorXd g_pin = pin.gravity(s.q);
    const Eigen::VectorXd g_kdl = kdl.gravity(s.q);
    const Eigen::VectorXd r_pin = pin.rnea(s.q, s.v, s.a);
    const Eigen::VectorXd r_kdl = kdl.rnea(s.q, s.v, s.a);
    const Eigen::VectorXd n_pin = pin.nonlinear_effects(s.q, s.v);
    const Eigen::VectorXd n_kdl = kdl.nonlinear_effects(s.q, s.v);
    const Eigen::MatrixXd m_pin = pin.mass_matrix(s.q);
    const Eigen::MatrixXd m_kdl = kdl.mass_matrix(s.q);
    const Eigen::MatrixXd j_pin = pin.tip_jacobian(s.q);
    const Eigen::MatrixXd j_kdl = kdl.tip_jacobian(s.q);
    const Eigen::Vector3d p_pin = pin.tip_position(s.q);
    const Eigen::Vector3d p_kdl = kdl.tip_position(s.q);
    const Eigen::Matrix3d rot_pin = pin.tip_rotation(s.q);
    const Eigen::Matrix3d rot_kdl = kdl.tip_rotation(s.q);

    const double dg = max_abs_diff(g_pin, g_kdl);
    const double dr = max_abs_diff(r_pin, r_kdl);
    const double dn = max_abs_diff(n_pin, n_kdl);
    const double dm = max_abs_diff(m_pin, m_kdl);
    const double dj = max_abs_diff(j_pin, j_kdl);
    const double dp = max_abs_diff(p_pin, p_kdl);
    const double drot = max_abs_diff(rot_pin, rot_kdl);
    EXPECT_LE(dg, kTorqueTol) << "gravity\npin " << g_pin.transpose() << "\nkdl "
                              << g_kdl.transpose();
    EXPECT_LE(dr, kTorqueTol) << "rnea\npin " << r_pin.transpose() << "\nkdl " << r_kdl.transpose();
    EXPECT_LE(dn, kTorqueTol) << "nonlinear effects\npin " << n_pin.transpose() << "\nkdl "
                              << n_kdl.transpose();
    EXPECT_LE(dm, kMassTol) << "mass matrix\npin\n" << m_pin << "\nkdl\n" << m_kdl;
    EXPECT_LE(dj, kJacobianTol) << "jacobian\npin\n" << j_pin << "\nkdl\n" << j_kdl;
    EXPECT_LE(dp, kPositionTol) << "tip position";
    EXPECT_LE(drot, kPositionTol) << "tip rotation";

    w.gravity = std::max(w.gravity, dg);
    w.rnea = std::max(w.rnea, dr);
    w.nle = std::max(w.nle, dn);
    w.mass = std::max(w.mass, dm);
    w.jacobian = std::max(w.jacobian, dj);
    w.position = std::max(w.position, dp);
    w.rotation = std::max(w.rotation, drot);
  }
  w.print(label);
  return w;
}
}  // namespace

class DynamicsEquivalenceTest : public ::testing::Test
{
protected:
  std::string urdf_{read_urdf()};
  PinocchioModel pin_{urdf_, kJoints, "tool0"};
  KdlModel kdl_{urdf_, kJoints, "tool0"};
};

TEST_F(DynamicsEquivalenceTest, SameLimitsFromTheUrdf)
{
  EXPECT_EQ(pin_.dof(), kdl_.dof());
  EXPECT_TRUE(pin_.effort_limits() == kdl_.effort_limits());
  EXPECT_TRUE(pin_.velocity_limits() == kdl_.velocity_limits());
  EXPECT_TRUE(pin_.lower_limits() == kdl_.lower_limits());
  EXPECT_TRUE(pin_.upper_limits() == kdl_.upper_limits());
}

TEST_F(DynamicsEquivalenceTest, AgreeOnRandomStatesWithoutPayload)
{
  compare(pin_, kdl_, random_states(pin_), "payload 0.0 kg");
}

TEST_F(DynamicsEquivalenceTest, AgreeOnRandomStatesWithOneKilogramPayload)
{
  // The controllers' default payload_com: on the tool0 z axis.
  pin_.set_payload(1.0, Eigen::Vector3d(0.0, 0.0, 0.03));
  kdl_.set_payload(1.0, Eigen::Vector3d(0.0, 0.0, 0.03));
  compare(pin_, kdl_, random_states(pin_), "payload 1.0 kg");
}

TEST_F(DynamicsEquivalenceTest, AgreeWithAnOffAxisPayloadOnARotatedTipFrame)
{
  // ft_frame is rotated by pi about x relative to wrist_3_link, and the CoM is off every axis:
  // catches a payload placed in the wrong frame or a CoM/inertia-origin mix-up in either backend.
  PinocchioModel pin(urdf_, kJoints, "ft_frame");
  KdlModel kdl(urdf_, kJoints, "ft_frame");
  pin.set_payload(1.0, Eigen::Vector3d(0.02, -0.03, 0.06));
  kdl.set_payload(1.0, Eigen::Vector3d(0.02, -0.03, 0.06));
  compare(pin, kdl, random_states(pin), "payload 1.0 kg off-axis on ft_frame");
}

TEST_F(DynamicsEquivalenceTest, AgreeWithANonDefaultGravity)
{
  // gravity_z reaches both backends (e.g. SDF's default -9.8 instead of -9.81).
  PinocchioModel pin(urdf_, kJoints, "tool0", -9.8);
  KdlModel kdl(urdf_, kJoints, "tool0", -9.8);
  const auto states = random_states(pin);
  compare(pin, kdl, states, "gravity_z -9.8");
  // ... and it is actually used: g scales with it.
  const Eigen::VectorXd g981 = pin_.gravity(states[0].q);
  EXPECT_TRUE(kdl.gravity(states[0].q).isApprox(g981 * (9.8 / 9.81), 1e-12));
}

TEST_F(DynamicsEquivalenceTest, PayloadChangesOnACloneLeaveTheOriginalUntouched)
{
  // The controllers change the payload on a clone outside the realtime loop; the model the loop
  // is still using must not change underneath it.
  const auto states = random_states(pin_);
  for (DynamicsModel * model :
       {static_cast<DynamicsModel *>(&pin_), static_cast<DynamicsModel *>(&kdl_)}) {
    SCOPED_TRACE(model->backend());
    const Eigen::VectorXd g0 = model->gravity(states[1].q);
    auto copy = model->clone();
    copy->set_payload(2.0, Eigen::Vector3d(0.0, 0.0, 0.05));
    EXPECT_FALSE(copy->gravity(states[1].q).isApprox(g0, 1e-6));
    EXPECT_TRUE(model->gravity(states[1].q) == g0);
    EXPECT_DOUBLE_EQ(model->payload_mass(), 0.0);
    EXPECT_DOUBLE_EQ(copy->payload_mass(), 2.0);
    copy->set_payload(0.0, Eigen::Vector3d(0.0, 0.0, 0.05));
    EXPECT_TRUE(copy->gravity(states[1].q).isApprox(g0, 1e-12));
  }
}

TEST_F(DynamicsEquivalenceTest, KdlRespectsTheControllerJointOrder)
{
  std::vector<std::string> rev(kJoints.rbegin(), kJoints.rend());
  KdlModel bwd(urdf_, rev, "tool0");
  const Eigen::VectorXd q = random_states(pin_)[2].q;
  const Eigen::VectorXd q_rev = q.reverse();
  EXPECT_TRUE(kdl_.gravity(q).reverse().isApprox(bwd.gravity(q_rev), 1e-12));
  EXPECT_TRUE(kdl_.tip_jacobian(q).rowwise().reverse().isApprox(bwd.tip_jacobian(q_rev), 1e-12));
}

TEST(DynamicsFactory, BuildsEitherBackendByNameAndRejectsOthers)
{
  const std::string urdf = read_urdf();
  EXPECT_STREQ(make_dynamics_model("pinocchio", urdf, kJoints, "tool0")->backend(), "pinocchio");
  EXPECT_STREQ(make_dynamics_model("kdl", urdf, kJoints, "tool0")->backend(), "kdl");
  EXPECT_THROW(make_dynamics_model("rbdl", urdf, kJoints, "tool0"), std::runtime_error);
}

TEST(DynamicsFactory, KdlRejectsUnknownJointOrFrameOrDofMismatch)
{
  const std::string urdf = read_urdf();
  auto bad = kJoints;
  bad[0] = "not_a_joint";
  EXPECT_THROW(KdlModel(urdf, bad, "tool0"), std::runtime_error);
  EXPECT_THROW(KdlModel(urdf, kJoints, "no_such_frame"), std::runtime_error);
  auto five = kJoints;
  five.pop_back();
  EXPECT_THROW(KdlModel(urdf, five, "tool0"), std::runtime_error);
  EXPECT_THROW(KdlModel("<robot", kJoints, "tool0"), std::runtime_error);
  EXPECT_THROW(
    KdlModel(urdf, kJoints, "tool0").set_payload(-1.0, Eigen::Vector3d::Zero()),
    std::invalid_argument);
}
