// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "dynamics/kdl_model.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.hpp>

namespace ur5e_teaching_controllers::dynamics
{

namespace
{
Eigen::Matrix3d to_eigen(const KDL::Rotation & r)
{
  Eigen::Matrix3d m;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      m(i, j) = r(i, j);
    }
  }
  return m;
}
}  // namespace

KdlModel::KdlModel(
  const std::string & urdf, const std::vector<std::string> & joint_names,
  const std::string & tip_frame, double gravity_z)
: DynamicsModel(joint_names, gravity_z)
{
  urdf::Model urdf_model;
  if (!urdf_model.initString(urdf)) {
    throw std::runtime_error("KdlModel: cannot parse URDF");
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
    throw std::runtime_error("KdlModel: kdl_parser cannot build a tree from the URDF");
  }
  if (!tree.getChain(kBaseLink, tip_frame, chain_)) {
    throw std::runtime_error(
      std::string("KdlModel: no chain ") + kBaseLink + " -> '" + tip_frame + "' in the URDF");
  }

  // KDL computes in base_link; the controllers (and Pinocchio) work in the URDF root frame.
  const std::string root = tree.getRootSegment()->first;
  if (root != kBaseLink) {
    KDL::Chain root_chain;
    if (!tree.getChain(root, kBaseLink, root_chain) || root_chain.getNrOfJoints() != 0) {
      throw std::runtime_error(
        std::string("KdlModel: ") + kBaseLink + " must hang off the URDF root '" + root +
        "' through fixed joints only");
    }
    for (const auto & segment : root_chain.segments) {
      root_to_base_ = root_to_base_ * segment.pose(0.0);
    }
  }
  gravity_in_base_ = root_to_base_.M.Inverse(KDL::Vector(0.0, 0.0, gravity_z_));

  std::vector<std::string> chain_joints;
  for (const auto & segment : chain_.segments) {
    if (segment.getJoint().getType() != KDL::Joint::Fixed) {
      chain_joints.push_back(segment.getJoint().getName());
    }
  }
  const std::size_t n = joint_names_.size();
  if (chain_joints.size() != n) {
    throw std::runtime_error(
      "KdlModel: the chain " + std::string(kBaseLink) + " -> " + tip_frame + " has " +
      std::to_string(chain_joints.size()) + " DoF but " + std::to_string(n) +
      " joints were configured");
  }
  idx_.resize(n);
  const auto en = static_cast<Eigen::Index>(n);
  effort_limit_.resize(en);
  velocity_limit_.resize(en);
  lower_limit_.resize(en);
  upper_limit_.resize(en);
  constexpr double kInf = std::numeric_limits<double>::infinity();
  for (std::size_t j = 0; j < n; ++j) {
    const auto it = std::find(chain_joints.begin(), chain_joints.end(), joint_names_[j]);
    if (it == chain_joints.end()) {
      throw std::runtime_error(
        "KdlModel: joint '" + joint_names_[j] + "' not on the chain " + kBaseLink + " -> " +
        tip_frame);
    }
    idx_[j] = static_cast<int>(it - chain_joints.begin());
    const auto joint = urdf_model.getJoint(joint_names_[j]);
    const auto e = static_cast<Eigen::Index>(j);
    const bool limited = joint->limits && joint->type != urdf::Joint::CONTINUOUS;
    effort_limit_[e] = joint->limits ? joint->limits->effort : kInf;
    velocity_limit_[e] = joint->limits ? joint->limits->velocity : kInf;
    lower_limit_[e] = limited ? joint->limits->lower : -kInf;
    upper_limit_[e] = limited ? joint->limits->upper : kInf;
  }

  last_segment_inertia_without_payload_ = chain_.segments.back().getInertia();
  rot_root_base_ = to_eigen(root_to_base_.M);

  const unsigned int nj = chain_.getNrOfJoints();
  q_.resize(nj);
  v_.resize(nj);
  a_.resize(nj);
  tau_kdl_.resize(nj);
  coriolis_.resize(nj);
  f_ext_.assign(chain_.getNrOfSegments(), KDL::Wrench::Zero());
  mass_kdl_.resize(nj);
  jacobian_kdl_.resize(nj);
  tau_ = Vector::Zero(en);
  mass_ = Eigen::MatrixXd::Zero(en, en);
  jacobian_ = Matrix6x::Zero(6, en);
  build_solvers();
}

KdlModel::KdlModel(const KdlModel & other)
: DynamicsModel(other),
  chain_(other.chain_),
  last_segment_inertia_without_payload_(other.last_segment_inertia_without_payload_),
  root_to_base_(other.root_to_base_),
  rot_root_base_(other.rot_root_base_),
  gravity_in_base_(other.gravity_in_base_),
  idx_(other.idx_),
  q_(other.q_),
  v_(other.v_),
  a_(other.a_),
  tau_kdl_(other.tau_kdl_),
  coriolis_(other.coriolis_),
  f_ext_(other.f_ext_),
  mass_kdl_(other.mass_kdl_),
  jacobian_kdl_(other.jacobian_kdl_),
  tip_(other.tip_),
  tau_(other.tau_),
  mass_(other.mass_),
  jacobian_(other.jacobian_)
{
  build_solvers();  // the copied solvers would still point at other.chain_
}

KdlModel::~KdlModel() = default;

std::unique_ptr<DynamicsModel> KdlModel::clone() const
{
  return std::unique_ptr<DynamicsModel>(new KdlModel(*this));
}

void KdlModel::build_solvers()
{
  rne_ = std::make_unique<KDL::ChainIdSolver_RNE>(chain_, gravity_in_base_);
  dyn_ = std::make_unique<KDL::ChainDynParam>(chain_, gravity_in_base_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
}

void KdlModel::set_payload(double mass, const Eigen::Vector3d & com_in_tip)
{
  if (mass < 0.0) {
    throw std::invalid_argument("KdlModel: payload mass must be >= 0");
  }
  // KDL's RigidBodyInertia takes the rotational inertia about the centre of mass, and a
  // segment's inertia is expressed in its tip frame, which for the last segment is tip_frame.
  const double i = mass * kPayloadCubeSide * kPayloadCubeSide / 6.0;
  const KDL::RigidBodyInertia payload(
    mass, KDL::Vector(com_in_tip.x(), com_in_tip.y(), com_in_tip.z()),
    KDL::RotationalInertia(i, i, i, 0.0, 0.0, 0.0));
  chain_.segments.back().setInertia(last_segment_inertia_without_payload_ + payload);
  build_solvers();
  payload_mass_ = mass;
}

void KdlModel::to_kdl(const Vector & in, KDL::JntArray & out) const
{
  for (std::size_t j = 0; j < idx_.size(); ++j) {
    out(static_cast<unsigned int>(idx_[j])) = in[static_cast<Eigen::Index>(j)];
  }
}

void KdlModel::from_kdl(const KDL::JntArray & in, Vector & out) const
{
  for (std::size_t j = 0; j < idx_.size(); ++j) {
    out[static_cast<Eigen::Index>(j)] = in(static_cast<unsigned int>(idx_[j]));
  }
}

// The solvers only fail on size mismatches, and every size is fixed at construction, so their
// return codes are not checked in these realtime-path queries.

const KdlModel::Vector & KdlModel::gravity(const Vector & q)
{
  to_kdl(q, q_);
  (void)dyn_->JntToGravity(q_, tau_kdl_);
  from_kdl(tau_kdl_, tau_);
  return tau_;
}

const KdlModel::Vector & KdlModel::rnea(const Vector & q, const Vector & v, const Vector & a)
{
  to_kdl(q, q_);
  to_kdl(v, v_);
  to_kdl(a, a_);
  (void)rne_->CartToJnt(q_, v_, a_, f_ext_, tau_kdl_);
  from_kdl(tau_kdl_, tau_);
  return tau_;
}

const KdlModel::Vector & KdlModel::nonlinear_effects(const Vector & q, const Vector & v)
{
  to_kdl(q, q_);
  to_kdl(v, v_);
  (void)dyn_->JntToCoriolis(q_, v_, coriolis_);
  (void)dyn_->JntToGravity(q_, tau_kdl_);
  tau_kdl_.data += coriolis_.data;
  from_kdl(tau_kdl_, tau_);
  return tau_;
}

const Eigen::MatrixXd & KdlModel::mass_matrix(const Vector & q)
{
  to_kdl(q, q_);
  (void)dyn_->JntToMass(q_, mass_kdl_);
  for (std::size_t r = 0; r < idx_.size(); ++r) {
    for (std::size_t c = 0; c < idx_.size(); ++c) {
      mass_(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) =
        mass_kdl_(static_cast<unsigned int>(idx_[r]), static_cast<unsigned int>(idx_[c]));
    }
  }
  return mass_;
}

const KdlModel::Matrix6x & KdlModel::tip_jacobian(const Vector & q)
{
  to_kdl(q, q_);
  jacobian_kdl_.data.setZero();
  // Reference point: the tip origin; orientation: base_link. Rotate both halves into the root.
  (void)jac_solver_->JntToJac(q_, jacobian_kdl_);
  for (std::size_t j = 0; j < idx_.size(); ++j) {
    const auto col = jacobian_kdl_.data.col(idx_[j]);
    const auto e = static_cast<Eigen::Index>(j);
    jacobian_.col(e).head<3>() = rot_root_base_ * col.head<3>();
    jacobian_.col(e).tail<3>() = rot_root_base_ * col.tail<3>();
  }
  return jacobian_;
}

void KdlModel::tip_frame_in_root(const Vector & q)
{
  to_kdl(q, q_);
  (void)fk_->JntToCart(q_, tip_);
  tip_ = root_to_base_ * tip_;
}

Eigen::Vector3d KdlModel::tip_position(const Vector & q)
{
  tip_frame_in_root(q);
  return Eigen::Vector3d(tip_.p.x(), tip_.p.y(), tip_.p.z());
}

Eigen::Matrix3d KdlModel::tip_rotation(const Vector & q)
{
  tip_frame_in_root(q);
  return to_eigen(tip_.M);
}

}  // namespace ur5e_teaching_controllers::dynamics
