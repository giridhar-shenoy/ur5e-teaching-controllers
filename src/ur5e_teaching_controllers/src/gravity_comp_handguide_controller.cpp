// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "ur5e_teaching_controllers/gravity_comp_handguide_controller.hpp"

#include <cmath>

#include "control/control_laws.hpp"
#include "dynamics/pinocchio_model.hpp"
#include "ur5e_teaching_controllers/gravity_comp_handguide_controller_parameters.hpp"

namespace ur5e_teaching_controllers
{

using controller_interface::CallbackReturn;
using controller_interface::return_type;

namespace
{
// Velocity scale of the smooth sign() used by the Coulomb friction hook [rad/s].
constexpr double kCoulombSmoothing = 0.01;
}  // namespace

GravityCompHandGuidingController::GravityCompHandGuidingController() = default;
GravityCompHandGuidingController::~GravityCompHandGuidingController() = default;

CallbackReturn GravityCompHandGuidingController::on_init()
{
  try {
    param_listener_ =
      std::make_shared<gravity_comp_handguide_controller::ParamListener>(get_node());
    params_ =
      std::make_unique<gravity_comp_handguide_controller::Params>(param_listener_->get_params());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "parameter init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  joints_ = params_->joints;
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityCompHandGuidingController::on_configure(const rclcpp_lifecycle::State &)
{
  *params_ = param_listener_->get_params();
  const std::size_t n = params_->joints.size();
  for (const auto * v :
       {&params_->damping, &params_->friction_viscous, &params_->friction_coulomb,
        &params_->max_joint_velocity}) {
    if (v->size() != n) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "every per-joint parameter needs %zu entries (one per joint)", n);
      return CallbackReturn::ERROR;
    }
  }
  if (!configure_common(params_->joints, params_->tip_frame, params_->state_publish_decimation)) {
    return CallbackReturn::ERROR;
  }
  qd_vec_.assign(n, 0.0);
  tau_model_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(n));
  apply_params();
  RCLCPP_INFO(
    get_node()->get_logger(), "configured: %zu joints, model payload %.2f kg, coriolis %s", n,
    params_->payload_mass, params_->coriolis_compensation ? "on" : "off");
  return CallbackReturn::SUCCESS;
}

void GravityCompHandGuidingController::apply_params()
{
  model_->set_payload(
    params_->payload_mass,
    Eigen::Vector3d(params_->payload_com[0], params_->payload_com[1], params_->payload_com[2]));
}

CallbackReturn GravityCompHandGuidingController::on_activate(const rclcpp_lifecycle::State &)
{
  if (!read_state()) {
    RCLCPP_ERROR(get_node()->get_logger(), "cannot read joint state on activation");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityCompHandGuidingController::on_deactivate(const rclcpp_lifecycle::State &)
{
  write_zero_effort();
  return CallbackReturn::SUCCESS;
}

return_type GravityCompHandGuidingController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (param_listener_->is_old(*params_)) {
    *params_ = param_listener_->get_params();
    apply_params();  // runtime payload change -> payload body inertia in the Pinocchio model
  }
  read_state();

  tau_model_ =
    params_->coriolis_compensation ? model_->nonlinear_effects(q_, qd_) : model_->gravity(q_);

  const auto & lower = model_->lower_limits();
  const auto & upper = model_->upper_limits();
  for (std::size_t j = 0; j < joints_.size(); ++j) {
    const auto e = static_cast<Eigen::Index>(j);
    const double v = qd_[e];
    qd_vec_[j] = v;
    const double friction = params_->friction_viscous[j] * v +
                            params_->friction_coulomb[j] * std::tanh(v / kCoulombSmoothing);
    const double repulsion = control::limit_repulsion(
      q_[e], lower[e], upper[e], params_->limit_margin, params_->limit_stiffness);
    const double tau = tau_model_[e] - params_->damping[j] * v + friction + repulsion;
    tau_[e] = control::Pid::saturate(tau, effort_limit_[e]);

    msg_.reference.effort[j] = tau_model_[e];
    msg_.feedback.positions[j] = q_[e];
    msg_.feedback.velocities[j] = v;
    msg_.output.effort[j] = tau_[e];
  }

  const int violating = control::first_speed_violation(qd_vec_, params_->max_joint_velocity);
  if (violating >= 0) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "speed cap: |qd| of %s is %.3f rad/s > %.3f rad/s; deactivating",
      joints_[static_cast<std::size_t>(violating)].c_str(),
      std::abs(qd_vec_[static_cast<std::size_t>(violating)]),
      params_->max_joint_velocity[static_cast<std::size_t>(violating)]);
    write_zero_effort();
    return return_type::ERROR;
  }

  write_effort();
  publish_state(time);
  return return_type::OK;
}

}  // namespace ur5e_teaching_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ur5e_teaching_controllers::GravityCompHandGuidingController,
  controller_interface::ControllerInterface)
