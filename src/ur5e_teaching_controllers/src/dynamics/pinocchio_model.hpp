// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Thin wrapper around Pinocchio for the controllers: build the model from the robot_description
// URDF (no hard-coded UR5e numbers anywhere), map the controller's joint order onto Pinocchio's,
// and expose gravity, RNEA, non-linear effects, the mass matrix, frame Jacobians and a payload
// that can be changed at runtime.
//
// All query methods are allocation-free after construction (they write into pre-sized members),
// so they can be called from the controller's realtime update().
#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

#include <pinocchio/multibody.hpp>
#include <pinocchio/spatial.hpp>

namespace ur5e_teaching_controllers::dynamics
{

class PinocchioModel
{
public:
  using Vector = Eigen::VectorXd;
  using Matrix6x = Eigen::Matrix<double, 6, Eigen::Dynamic>;

  /// Edge length of the solid cube used for the payload's rotational inertia. The xacro uses the
  /// same cube for the simulated payload, so model and simulation agree when the masses agree.
  static constexpr double kPayloadCubeSide = 0.06;

  /// @param urdf        robot_description XML
  /// @param joint_names controller joint order; every name must be a 1-DoF joint in the URDF and
  ///                    together they must be all of the model's DoFs
  /// @param tip_frame   frame the payload is attached to and Jacobians refer to (e.g. "tool0")
  /// @throws std::runtime_error on a malformed URDF, unknown joint/frame or DoF mismatch
  PinocchioModel(
    const std::string & urdf, const std::vector<std::string> & joint_names,
    const std::string & tip_frame);

  std::size_t dof() const { return joint_names_.size(); }

  /// Replace the payload: `mass` [kg] with its centre of mass at `com_in_tip` [m] expressed in the
  /// tip frame, rotational inertia of a solid cube of side kPayloadCubeSide. mass = 0 removes it.
  void set_payload(double mass, const Eigen::Vector3d & com_in_tip);
  double payload_mass() const { return payload_mass_; }

  /// g(q): generalised gravity torque [N m], controller joint order.
  const Vector & gravity(const Vector & q);
  /// RNEA: M(q) a + C(q, v) v + g(q).
  const Vector & rnea(const Vector & q, const Vector & v, const Vector & a);
  /// C(q, v) v + g(q).
  const Vector & nonlinear_effects(const Vector & q, const Vector & v);
  /// Joint-space mass matrix M(q) (symmetric, full).
  const Eigen::MatrixXd & mass_matrix(const Vector & q);
  /// 6xN Jacobian of the tip frame, rows [linear; angular], expressed in a frame with the tip's
  /// origin and the base (world) orientation (Pinocchio LOCAL_WORLD_ALIGNED).
  const Matrix6x & tip_jacobian(const Vector & q);
  /// Tip position in the base frame.
  Eigen::Vector3d tip_position(const Vector & q);

  /// Limits read from the URDF, controller joint order.
  const Vector & effort_limits() const { return effort_limit_; }
  const Vector & velocity_limits() const { return velocity_limit_; }
  const Vector & lower_limits() const { return lower_limit_; }
  const Vector & upper_limits() const { return upper_limit_; }

private:
  void to_pinocchio(const Vector & in, Eigen::VectorXd & out) const;
  void from_pinocchio(const Eigen::VectorXd & in, Vector & out) const;

  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<std::string> joint_names_;
  std::vector<int> idx_;  // controller joint j -> Pinocchio q/v index

  pinocchio::FrameIndex tip_frame_id_{0};
  pinocchio::JointIndex tip_joint_id_{0};
  pinocchio::Inertia tip_body_inertia_without_payload_;
  double payload_mass_{0.0};

  Eigen::VectorXd q_, v_, a_;  // Pinocchio order scratch
  Vector tau_;                 // controller order output
  Eigen::MatrixXd mass_;
  Matrix6x jacobian_pin_, jacobian_;
  Vector effort_limit_, velocity_limit_, lower_limit_, upper_limit_;
};

}  // namespace ur5e_teaching_controllers::dynamics
