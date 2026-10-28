// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// The dynamics interface the controllers program against. Two backends implement it:
// PinocchioModel (the default) and KdlModel (orocos KDL). Both build their model from the
// robot_description URDF (no hard-coded UR5e numbers anywhere), map the controller's joint order
// onto their own, and express gravity, Jacobians and the tip pose in the URDF root frame, so the
// controllers cannot tell them apart beyond rounding (test_dynamics_equivalence checks this).
//
// Query methods are allocation-free after construction (they write into pre-sized members), so
// they can be called from the controller's realtime update(). set_payload() and clone() are not:
// KdlModel rebuilds its solvers on a payload change. The controllers therefore change the payload
// on a clone outside the realtime loop and swap the finished model in (EffortControllerBase).
#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

namespace ur5e_teaching_controllers::dynamics
{

class DynamicsModel
{
public:
  using Vector = Eigen::VectorXd;
  using Matrix6x = Eigen::Matrix<double, 6, Eigen::Dynamic>;

  /// Edge length of the solid cube used for the payload's rotational inertia. The xacro uses the
  /// same cube for the simulated payload, so model and simulation agree when the masses agree.
  static constexpr double kPayloadCubeSide = 0.06;
  /// Default z component of gravity in the URDF root frame [m/s^2]. SDF's default is -9.8, so
  /// the worlds set <gravity> to this value explicitly (see docs/controllers.md).
  static constexpr double kDefaultGravityZ = -9.81;

  virtual ~DynamicsModel() = default;

  /// Deep copy, payload included. Allocates; never call it from update().
  virtual std::unique_ptr<DynamicsModel> clone() const = 0;

  /// "pinocchio" or "kdl".
  virtual const char * backend() const = 0;

  std::size_t dof() const { return joint_names_.size(); }
  const std::vector<std::string> & joint_names() const { return joint_names_; }

  /// Replace the payload: `mass` [kg] with its centre of mass at `com_in_tip` [m] expressed in the
  /// tip frame, rotational inertia of a solid cube of side kPayloadCubeSide. mass = 0 removes it.
  /// May allocate (KdlModel rebuilds its solvers); never call it from update().
  virtual void set_payload(double mass, const Eigen::Vector3d & com_in_tip) = 0;
  double payload_mass() const { return payload_mass_; }
  double gravity_z() const { return gravity_z_; }

  /// g(q): generalised gravity torque [N m], controller joint order.
  virtual const Vector & gravity(const Vector & q) = 0;
  /// RNEA: M(q) a + C(q, v) v + g(q).
  virtual const Vector & rnea(const Vector & q, const Vector & v, const Vector & a) = 0;
  /// C(q, v) v + g(q).
  virtual const Vector & nonlinear_effects(const Vector & q, const Vector & v) = 0;
  /// Joint-space mass matrix M(q) (symmetric, full).
  virtual const Eigen::MatrixXd & mass_matrix(const Vector & q) = 0;
  /// 6xN Jacobian of the tip frame, rows [linear; angular], expressed in a frame with the tip's
  /// origin and the URDF root's orientation (Pinocchio's LOCAL_WORLD_ALIGNED).
  virtual const Matrix6x & tip_jacobian(const Vector & q) = 0;
  /// Tip position in the URDF root frame.
  virtual Eigen::Vector3d tip_position(const Vector & q) = 0;
  /// Tip orientation in the URDF root frame.
  virtual Eigen::Matrix3d tip_rotation(const Vector & q) = 0;

  /// Limits read from the URDF, controller joint order.
  const Vector & effort_limits() const { return effort_limit_; }
  const Vector & velocity_limits() const { return velocity_limit_; }
  const Vector & lower_limits() const { return lower_limit_; }
  const Vector & upper_limits() const { return upper_limit_; }

protected:
  DynamicsModel(const std::vector<std::string> & joint_names, double gravity_z)
  : joint_names_(joint_names), gravity_z_(gravity_z)
  {
  }
  DynamicsModel(const DynamicsModel &) = default;
  DynamicsModel & operator=(const DynamicsModel &) = default;

  std::vector<std::string> joint_names_;
  double gravity_z_;
  double payload_mass_{0.0};
  Vector effort_limit_, velocity_limit_, lower_limit_, upper_limit_;
};

/// Backend names accepted by make_dynamics_model() (and by the dynamics_backend parameter).
inline const std::vector<std::string> & dynamics_backends()
{
  static const std::vector<std::string> names{"pinocchio", "kdl"};
  return names;
}

/// Build the backend called `backend` ("pinocchio" or "kdl").
/// @param urdf        robot_description XML
/// @param joint_names controller joint order; every name must be a 1-DoF joint in the URDF and
///                    together they must be all of the model's DoFs
/// @param tip_frame   frame the payload is attached to and Jacobians refer to (e.g. "tool0")
/// @param gravity_z   z component of gravity in the URDF root frame [m/s^2]
/// @throws std::runtime_error on an unknown backend, a malformed URDF, an unknown joint or frame,
///         or a DoF mismatch
std::unique_ptr<DynamicsModel> make_dynamics_model(
  const std::string & backend, const std::string & urdf,
  const std::vector<std::string> & joint_names, const std::string & tip_frame,
  double gravity_z = DynamicsModel::kDefaultGravityZ);

}  // namespace ur5e_teaching_controllers::dynamics
