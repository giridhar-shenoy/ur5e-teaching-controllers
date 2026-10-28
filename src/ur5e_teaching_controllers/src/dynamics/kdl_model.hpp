// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// DynamicsModel backend on orocos KDL: kdl_parser turns the robot_description URDF into a tree,
// the serial chain base_link -> tip_frame is extracted, and
//   - RNEA comes from KDL::ChainIdSolver_RNE,
//   - gravity, the mass matrix and the Coriolis term from KDL::ChainDynParam,
//   - the tip Jacobian from KDL::ChainJntToJacSolver, the tip pose from ChainFkSolverPos_recursive.
// KDL works in the chain's root frame (base_link); this wrapper rotates gravity into it and the
// Jacobian and tip pose out of it, so every quantity matches PinocchioModel's (URDF root frame).
// Joint limits are read with the urdf package (kdl_parser does not carry them).
//
// The payload is added to the inertia of the chain's last segment (rigidly attached to the tip
// frame), and set_payload() rebuilds the solvers: it allocates and must not run in update().
#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

#include "dynamics/dynamics_model.hpp"

namespace ur5e_teaching_controllers::dynamics
{

class KdlModel final : public DynamicsModel
{
public:
  /// Root of the KDL chain (the arm's base link in ur_description).
  static constexpr const char * kBaseLink = "base_link";

  /// Same contract as PinocchioModel's constructor.
  KdlModel(
    const std::string & urdf, const std::vector<std::string> & joint_names,
    const std::string & tip_frame, double gravity_z = kDefaultGravityZ);
  ~KdlModel() override;

  std::unique_ptr<DynamicsModel> clone() const override;
  const char * backend() const override { return "kdl"; }

  /// Rebuilds the last segment's inertia and every solver. Allocates.
  void set_payload(double mass, const Eigen::Vector3d & com_in_tip) override;

  const Vector & gravity(const Vector & q) override;
  const Vector & rnea(const Vector & q, const Vector & v, const Vector & a) override;
  const Vector & nonlinear_effects(const Vector & q, const Vector & v) override;
  const Eigen::MatrixXd & mass_matrix(const Vector & q) override;
  const Matrix6x & tip_jacobian(const Vector & q) override;
  Eigen::Vector3d tip_position(const Vector & q) override;
  Eigen::Matrix3d tip_rotation(const Vector & q) override;

private:
  KdlModel(const KdlModel & other);
  /// (Re)create every solver on chain_. The solvers keep a reference to chain_, so this runs
  /// after any change to chain_ and after a copy.
  void build_solvers();
  void to_kdl(const Vector & in, KDL::JntArray & out) const;
  void from_kdl(const KDL::JntArray & in, Vector & out) const;
  void tip_frame_in_root(const Vector & q);

  KDL::Chain chain_;
  KDL::RigidBodyInertia last_segment_inertia_without_payload_;
  KDL::Frame root_to_base_;  // URDF root -> base_link (fixed joints only)
  Eigen::Matrix3d rot_root_base_;
  KDL::Vector gravity_in_base_;
  std::vector<int> idx_;  // controller joint j -> KDL chain joint index

  std::unique_ptr<KDL::ChainIdSolver_RNE> rne_;
  std::unique_ptr<KDL::ChainDynParam> dyn_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;

  // KDL-order scratch, pre-sized so the queries never allocate.
  KDL::JntArray q_, v_, a_, tau_kdl_, coriolis_;
  KDL::Wrenches f_ext_;
  KDL::JntSpaceInertiaMatrix mass_kdl_;
  KDL::Jacobian jacobian_kdl_;
  KDL::Frame tip_;

  Vector tau_;  // controller order output
  Eigen::MatrixXd mass_;
  Matrix6x jacobian_;
};

}  // namespace ur5e_teaching_controllers::dynamics
