// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "dynamics/pinocchio_model.hpp"

#include <stdexcept>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace ur5e_teaching_controllers::dynamics
{

PinocchioModel::PinocchioModel(
  const std::string & urdf, const std::vector<std::string> & joint_names,
  const std::string & tip_frame, double gravity_z)
: DynamicsModel(joint_names, gravity_z)
{
  try {
    pinocchio::urdf::buildModelFromXML(urdf, model_);
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("PinocchioModel: cannot parse URDF: ") + e.what());
  }
  if (static_cast<std::size_t>(model_.nv) != joint_names_.size() || model_.nq != model_.nv) {
    throw std::runtime_error(
      "PinocchioModel: the URDF has " + std::to_string(model_.nv) + " DoF but " +
      std::to_string(joint_names_.size()) + " joints were configured");
  }

  const std::size_t n = joint_names_.size();
  idx_.resize(n);
  effort_limit_.resize(static_cast<Eigen::Index>(n));
  velocity_limit_.resize(static_cast<Eigen::Index>(n));
  lower_limit_.resize(static_cast<Eigen::Index>(n));
  upper_limit_.resize(static_cast<Eigen::Index>(n));
  for (std::size_t j = 0; j < n; ++j) {
    if (!model_.existJointName(joint_names_[j])) {
      throw std::runtime_error("PinocchioModel: joint '" + joint_names_[j] + "' not in the URDF");
    }
    const auto id = model_.getJointId(joint_names_[j]);
    if (model_.joints[id].nv() != 1) {
      throw std::runtime_error("PinocchioModel: joint '" + joint_names_[j] + "' is not 1-DoF");
    }
    const int k = model_.joints[id].idx_v();
    idx_[j] = k;
    const auto e = static_cast<Eigen::Index>(j);
    effort_limit_[e] = model_.upperEffortLimit[k];
    velocity_limit_[e] = model_.upperVelocityLimit[k];
    lower_limit_[e] = model_.lowerPositionLimit[k];
    upper_limit_[e] = model_.upperPositionLimit[k];
  }

  if (!model_.existFrame(tip_frame)) {
    throw std::runtime_error("PinocchioModel: frame '" + tip_frame + "' not in the URDF");
  }
  tip_frame_id_ = model_.getFrameId(tip_frame);
  tip_joint_id_ = model_.frames[tip_frame_id_].parentJoint;
  tip_body_inertia_without_payload_ = model_.inertias[tip_joint_id_];
  model_.gravity.linear(Eigen::Vector3d(0.0, 0.0, gravity_z_));

  data_ = pinocchio::Data(model_);
  q_ = Eigen::VectorXd::Zero(model_.nq);
  v_ = Eigen::VectorXd::Zero(model_.nv);
  a_ = Eigen::VectorXd::Zero(model_.nv);
  tau_ = Vector::Zero(static_cast<Eigen::Index>(n));
  mass_ = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(n), static_cast<Eigen::Index>(n));
  jacobian_pin_ = Matrix6x::Zero(6, model_.nv);
  jacobian_ = Matrix6x::Zero(6, static_cast<Eigen::Index>(n));
}

std::unique_ptr<DynamicsModel> PinocchioModel::clone() const
{
  return std::unique_ptr<DynamicsModel>(new PinocchioModel(*this));
}

void PinocchioModel::set_payload(double mass, const Eigen::Vector3d & com_in_tip)
{
  if (mass < 0.0) {
    throw std::invalid_argument("PinocchioModel: payload mass must be >= 0");
  }
  const double i = mass * kPayloadCubeSide * kPayloadCubeSide / 6.0;
  const pinocchio::Inertia payload_in_tip(mass, com_in_tip, Eigen::Matrix3d::Identity() * i);
  // The tip frame's placement is relative to its parent joint; move the payload there.
  const pinocchio::SE3 & tip_in_joint = model_.frames[tip_frame_id_].placement;
  model_.inertias[tip_joint_id_] =
    tip_body_inertia_without_payload_ + tip_in_joint.act(payload_in_tip);
  payload_mass_ = mass;
}

void PinocchioModel::to_pinocchio(const Vector & in, Eigen::VectorXd & out) const
{
  for (std::size_t j = 0; j < idx_.size(); ++j) {
    out[idx_[j]] = in[static_cast<Eigen::Index>(j)];
  }
}

void PinocchioModel::from_pinocchio(const Eigen::VectorXd & in, Vector & out) const
{
  for (std::size_t j = 0; j < idx_.size(); ++j) {
    out[static_cast<Eigen::Index>(j)] = in[idx_[j]];
  }
}

const PinocchioModel::Vector & PinocchioModel::gravity(const Vector & q)
{
  to_pinocchio(q, q_);
  from_pinocchio(pinocchio::computeGeneralizedGravity(model_, data_, q_), tau_);
  return tau_;
}

const PinocchioModel::Vector & PinocchioModel::rnea(
  const Vector & q, const Vector & v, const Vector & a)
{
  to_pinocchio(q, q_);
  to_pinocchio(v, v_);
  to_pinocchio(a, a_);
  from_pinocchio(pinocchio::rnea(model_, data_, q_, v_, a_), tau_);
  return tau_;
}

const PinocchioModel::Vector & PinocchioModel::nonlinear_effects(const Vector & q, const Vector & v)
{
  to_pinocchio(q, q_);
  to_pinocchio(v, v_);
  from_pinocchio(pinocchio::nonLinearEffects(model_, data_, q_, v_), tau_);
  return tau_;
}

const Eigen::MatrixXd & PinocchioModel::mass_matrix(const Vector & q)
{
  to_pinocchio(q, q_);
  pinocchio::crba(model_, data_, q_);
  // crba fills the upper triangle only.
  data_.M.triangularView<Eigen::StrictlyLower>() =
    data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  for (std::size_t r = 0; r < idx_.size(); ++r) {
    for (std::size_t c = 0; c < idx_.size(); ++c) {
      mass_(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = data_.M(idx_[r], idx_[c]);
    }
  }
  return mass_;
}

const PinocchioModel::Matrix6x & PinocchioModel::tip_jacobian(const Vector & q)
{
  to_pinocchio(q, q_);
  jacobian_pin_.setZero();
  pinocchio::computeFrameJacobian(
    model_, data_, q_, tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_pin_);
  for (std::size_t j = 0; j < idx_.size(); ++j) {
    jacobian_.col(static_cast<Eigen::Index>(j)) = jacobian_pin_.col(idx_[j]);
  }
  return jacobian_;
}

Eigen::Vector3d PinocchioModel::tip_position(const Vector & q)
{
  to_pinocchio(q, q_);
  pinocchio::framesForwardKinematics(model_, data_, q_);
  return data_.oMf[tip_frame_id_].translation();
}

Eigen::Matrix3d PinocchioModel::tip_rotation(const Vector & q)
{
  to_pinocchio(q, q_);
  pinocchio::framesForwardKinematics(model_, data_, q_);
  return data_.oMf[tip_frame_id_].rotation();
}

}  // namespace ur5e_teaching_controllers::dynamics
