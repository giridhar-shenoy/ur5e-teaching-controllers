// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// DynamicsModel backend on Pinocchio (the default): build the model from the robot_description
// URDF, map the controller's joint order onto Pinocchio's, and expose gravity, RNEA, non-linear
// effects, the mass matrix, frame Jacobians and a payload added to the tip body's inertia.
//
// All query methods are allocation-free after construction (they write into pre-sized members),
// so they can be called from the controller's realtime update().
#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include <pinocchio/multibody.hpp>
#include <pinocchio/spatial.hpp>

#include "dynamics/dynamics_model.hpp"

namespace ur5e_teaching_controllers::dynamics
{

class PinocchioModel final : public DynamicsModel
{
public:
  /// @param urdf        robot_description XML
  /// @param joint_names controller joint order; every name must be a 1-DoF joint in the URDF and
  ///                    together they must be all of the model's DoFs
  /// @param tip_frame   frame the payload is attached to and Jacobians refer to (e.g. "tool0")
  /// @param gravity_z   z component of gravity in the URDF root frame [m/s^2]
  /// @throws std::runtime_error on a malformed URDF, unknown joint/frame or DoF mismatch
  PinocchioModel(
    const std::string & urdf, const std::vector<std::string> & joint_names,
    const std::string & tip_frame, double gravity_z = kDefaultGravityZ);

  std::unique_ptr<DynamicsModel> clone() const override;
  const char * backend() const override { return "pinocchio"; }

  /// In-place and allocation-free here, but the controllers still call it on a clone (KDL's is
  /// not), so both backends take the same path.
  void set_payload(double mass, const Eigen::Vector3d & com_in_tip) override;

  const Vector & gravity(const Vector & q) override;
  const Vector & rnea(const Vector & q, const Vector & v, const Vector & a) override;
  const Vector & nonlinear_effects(const Vector & q, const Vector & v) override;
  const Eigen::MatrixXd & mass_matrix(const Vector & q) override;
  const Matrix6x & tip_jacobian(const Vector & q) override;
  Eigen::Vector3d tip_position(const Vector & q) override;
  Eigen::Matrix3d tip_rotation(const Vector & q) override;

private:
  void to_pinocchio(const Vector & in, Eigen::VectorXd & out) const;
  void from_pinocchio(const Eigen::VectorXd & in, Vector & out) const;

  PinocchioModel(const PinocchioModel &) = default;

  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<int> idx_;  // controller joint j -> Pinocchio q/v index

  pinocchio::FrameIndex tip_frame_id_{0};
  pinocchio::JointIndex tip_joint_id_{0};
  pinocchio::Inertia tip_body_inertia_without_payload_;

  Eigen::VectorXd q_, v_, a_;  // Pinocchio order scratch
  Vector tau_;                 // controller order output
  Eigen::MatrixXd mass_;
  Matrix6x jacobian_pin_, jacobian_;
};

}  // namespace ur5e_teaching_controllers::dynamics
