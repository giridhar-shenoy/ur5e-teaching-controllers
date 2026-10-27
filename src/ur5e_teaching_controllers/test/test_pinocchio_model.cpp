// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Checks of the Pinocchio wrapper against identities that must hold for any rigid-body model,
// evaluated on the UR5e description shipped by ur_description.
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "dynamics/pinocchio_model.hpp"

using ur5e_teaching_controllers::dynamics::PinocchioModel;

namespace
{
const std::vector<std::string> kJoints{"shoulder_pan_joint", "shoulder_lift_joint",
                                       "elbow_joint",        "wrist_1_joint",
                                       "wrist_2_joint",      "wrist_3_joint"};

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

Eigen::VectorXd stretched()
{
  Eigen::VectorXd q(6);
  q << 0.3, -0.2, 0.1, -1.0, 0.7, 0.2;
  return q;
}
}  // namespace

class PinocchioModelTest : public ::testing::Test
{
protected:
  PinocchioModel model{read_urdf(), kJoints, "tool0"};
};

TEST_F(PinocchioModelTest, LoadsSixDofAndUrdfLimits)
{
  EXPECT_EQ(model.dof(), 6u);
  // Values come from ur_description's joint_limits.yaml through the URDF, not from this repo.
  for (int j = 0; j < 3; ++j) {
    EXPECT_DOUBLE_EQ(model.effort_limits()[j], 150.0);
  }
  for (int j = 3; j < 6; ++j) {
    EXPECT_DOUBLE_EQ(model.effort_limits()[j], 28.0);
  }
  EXPECT_NEAR(model.velocity_limits()[0], M_PI, 1e-9);
  EXPECT_NEAR(model.upper_limits()[2], M_PI, 1e-9);  // elbow is limited to +-pi
}

TEST_F(PinocchioModelTest, GravityEqualsRneaAtRest)
{
  const Eigen::VectorXd q = stretched();
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  const Eigen::VectorXd g = model.gravity(q);
  const Eigen::VectorXd r = model.rnea(q, zero, zero);
  EXPECT_TRUE(g.isApprox(r, 1e-12));
  EXPECT_GT(g.norm(), 1.0);  // a stretched arm does feel gravity
}

TEST_F(PinocchioModelTest, MassMatrixIsSymmetricPositiveDefinite)
{
  const Eigen::MatrixXd M = model.mass_matrix(stretched());
  EXPECT_TRUE(M.isApprox(M.transpose(), 1e-12));
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(M);
  EXPECT_GT(eig.eigenvalues().minCoeff(), 0.0);
}

TEST_F(PinocchioModelTest, RneaMinusGravityIsMassTimesAccelerationAtZeroVelocity)
{
  const Eigen::VectorXd q = stretched();
  Eigen::VectorXd a(6);
  a << 0.5, -1.0, 2.0, 0.3, -0.4, 1.0;
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  const Eigen::VectorXd g = model.gravity(q);
  const Eigen::VectorXd tau = model.rnea(q, zero, a);
  const Eigen::MatrixXd M = model.mass_matrix(q);
  EXPECT_TRUE((tau - g).isApprox(M * a, 1e-9));
}

TEST_F(PinocchioModelTest, NonlinearEffectsEqualRneaWithZeroAcceleration)
{
  const Eigen::VectorXd q = stretched();
  Eigen::VectorXd v(6);
  v << 0.4, -0.3, 0.8, 1.0, -0.5, 0.2;
  const Eigen::VectorXd nle = model.nonlinear_effects(q, v);
  const Eigen::VectorXd r = model.rnea(q, v, Eigen::VectorXd::Zero(6));
  EXPECT_TRUE(nle.isApprox(r, 1e-12));
}

TEST_F(PinocchioModelTest, TipJacobianMatchesFiniteDifferenceOfTipPosition)
{
  const Eigen::VectorXd q = stretched();
  const Eigen::MatrixXd J = model.tip_jacobian(q);
  const double h = 1e-7;
  for (int j = 0; j < 6; ++j) {
    Eigen::VectorXd qp = q, qm = q;
    qp[j] += h;
    qm[j] -= h;
    const Eigen::Vector3d dp = (model.tip_position(qp) - model.tip_position(qm)) / (2 * h);
    EXPECT_TRUE(dp.isApprox(J.block<3, 1>(0, j), 1e-5)) << "column " << j;
  }
}

TEST_F(PinocchioModelTest, PayloadGravityEqualsJacobianTransposeOfItsWeight)
{
  // Adding a point-like payload m at the tip must add J_tip^T [0, 0, m g] to g(q)
  // (com at the tip origin, so the linear Jacobian of the tip is the payload's).
  const Eigen::VectorXd q = stretched();
  const Eigen::VectorXd g0 = model.gravity(q);
  const double m = 2.0;
  model.set_payload(m, Eigen::Vector3d::Zero());
  const Eigen::VectorXd g1 = model.gravity(q);
  const Eigen::MatrixXd J = model.tip_jacobian(q);
  const Eigen::Vector3d weight(0.0, 0.0, m * 9.81);
  EXPECT_TRUE((g1 - g0).isApprox(J.topRows<3>().transpose() * weight, 1e-3));
}

TEST_F(PinocchioModelTest, ZeroPayloadRestoresTheOriginalModel)
{
  const Eigen::VectorXd q = stretched();
  const Eigen::VectorXd g0 = model.gravity(q);
  model.set_payload(3.0, Eigen::Vector3d(0.01, -0.02, 0.08));
  EXPECT_FALSE(model.gravity(q).isApprox(g0, 1e-6));
  model.set_payload(0.0, Eigen::Vector3d(0.01, -0.02, 0.08));
  EXPECT_TRUE(model.gravity(q).isApprox(g0, 1e-12));
  EXPECT_DOUBLE_EQ(model.payload_mass(), 0.0);
}

TEST_F(PinocchioModelTest, PayloadComOffsetChangesWristTorque)
{
  // Moving the payload CoM sideways changes its lever arms, so g(q) must change: checks that
  // payload_com is actually used, not only payload_mass.
  const Eigen::VectorXd q = home();
  model.set_payload(1.0, Eigen::Vector3d(0.0, 0.0, 0.05));
  const Eigen::VectorXd ga = model.gravity(q);
  model.set_payload(1.0, Eigen::Vector3d(0.10, 0.0, 0.05));
  const Eigen::VectorXd gb = model.gravity(q);
  EXPECT_GT((ga - gb).norm(), 0.05);
}

TEST(PinocchioModelPayload, OffsetComOnARotatedTipFrameIsTransformedIntoTheJointFrame)
{
  // tool0 coincides with the wrist_3 joint frame, so the tests above cannot tell whether the
  // payload is moved from the tip frame into the joint frame. ft_frame is rotated by pi about x
  // relative to wrist_3_link: with a CoM offset c, the weight acts at p_tip + R_tip c, and
  // dg = J_c^T [0, 0, m g] with J_c = J_lin - [R_tip c]x J_ang (world-aligned).
  PinocchioModel model(read_urdf(), kJoints, "ft_frame");
  const Eigen::VectorXd q = stretched();
  const Eigen::VectorXd g0 = model.gravity(q);
  const double m = 1.5;
  const Eigen::Vector3d c(0.02, -0.03, 0.06);
  model.set_payload(m, c);
  const Eigen::VectorXd dg = model.gravity(q) - g0;

  const Eigen::MatrixXd J = model.tip_jacobian(q);
  const Eigen::Vector3d r = model.tip_rotation(q) * c;
  Eigen::Matrix3d r_cross;
  r_cross << 0, -r.z(), r.y(), r.z(), 0, -r.x(), -r.y(), r.x(), 0;
  const Eigen::MatrixXd J_c = J.topRows<3>() - r_cross * J.bottomRows<3>();
  const Eigen::VectorXd expected = J_c.transpose() * Eigen::Vector3d(0.0, 0.0, m * 9.81);
  EXPECT_TRUE(dg.isApprox(expected, 1e-6))
    << "dg = " << dg.transpose() << "\nexpected = " << expected.transpose();
}

TEST(PinocchioModelErrors, UnknownJointOrFrameOrDofMismatchThrows)
{
  const std::string urdf = read_urdf();
  auto bad = kJoints;
  bad[0] = "not_a_joint";
  EXPECT_THROW(PinocchioModel(urdf, bad, "tool0"), std::runtime_error);
  EXPECT_THROW(PinocchioModel(urdf, kJoints, "no_such_frame"), std::runtime_error);
  auto five = kJoints;
  five.pop_back();
  EXPECT_THROW(PinocchioModel(urdf, five, "tool0"), std::runtime_error);
  EXPECT_THROW(PinocchioModel("<robot", kJoints, "tool0"), std::runtime_error);
}

TEST(PinocchioModelErrors, ControllerJointOrderIsRespected)
{
  // Reversing the joint order must reverse the gravity vector, not scramble it.
  const std::string urdf = read_urdf();
  PinocchioModel fwd(urdf, kJoints, "tool0");
  std::vector<std::string> rev(kJoints.rbegin(), kJoints.rend());
  PinocchioModel bwd(urdf, rev, "tool0");
  const Eigen::VectorXd q = stretched();
  const Eigen::VectorXd q_rev = q.reverse();
  EXPECT_TRUE(fwd.gravity(q).reverse().isApprox(bwd.gravity(q_rev), 1e-12));
}
