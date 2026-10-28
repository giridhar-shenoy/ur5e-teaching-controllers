// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "ur5e_teaching_controllers/tcp_jog_controller.hpp"

#include <algorithm>
#include <cmath>

#include "control/control_laws.hpp"
#include "control/dls.hpp"
#include "dynamics/dynamics_model.hpp"
#include "ur5e_teaching_controllers/tcp_jog_controller_parameters.hpp"

namespace ur5e_teaching_controllers
{

using control::Matrix6;
using control::Vector6;
using controller_interface::CallbackReturn;
using controller_interface::return_type;

TcpJogController::TcpJogController() = default;
TcpJogController::~TcpJogController() = default;

CallbackReturn TcpJogController::on_init()
{
  try {
    param_listener_ = std::make_shared<tcp_jog_controller::ParamListener>(get_node());
    params_ = std::make_unique<tcp_jog_controller::Params>(param_listener_->get_params());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "parameter init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  joints_ = params_->joints;
  return CallbackReturn::SUCCESS;
}

namespace
{
// One entry per joint; checked on configure and again for every runtime parameter change.
bool per_joint_sizes_ok(const tcp_jog_controller::Params & p)
{
  const std::size_t n = p.joints.size();
  for (const auto * v :
       {&p.kp, &p.ki, &p.kd, &p.i_clamp, &p.max_joint_velocity, &p.max_joint_acceleration}) {
    if (v->size() != n) {
      return false;
    }
  }
  return true;
}

Vector6 to6(const Eigen::VectorXd & v) { return Vector6(v.head<6>()); }
}  // namespace

CallbackReturn TcpJogController::on_configure(const rclcpp_lifecycle::State &)
{
  *params_ = param_listener_->get_params();
  const std::size_t n = params_->joints.size();
  if (n != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "TcpJogController needs exactly 6 joints, got %zu", n);
    return CallbackReturn::ERROR;
  }
  if (!per_joint_sizes_ok(*params_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "every per-joint parameter needs %zu entries (one per joint)", n);
    return CallbackReturn::ERROR;
  }
  staged_ = std::make_unique<tcp_jog_controller::Params>(*params_);
  if (!configure_common(
        params_->joints, params_->tip_frame, params_->dynamics_backend, params_->gravity_z,
        params_->payload_mass, params_->payload_com, params_->state_publish_decimation)) {
    return CallbackReturn::ERROR;
  }
  param_listener_->setUserCallback([this](const tcp_jog_controller::Params & p) {
    request_payload(p.payload_mass, p.payload_com);
  });
  pid_.assign(n, control::Pid{});
  apply_params();

  const auto en = static_cast<Eigen::Index>(n);
  q_des_ = Eigen::VectorXd::Zero(en);
  qd_des_ = Eigen::VectorXd::Zero(en);
  qdd_des_ = Eigen::VectorXd::Zero(en);
  tau_ff_ = Eigen::VectorXd::Zero(en);

  command_frame_ = params_->command_frame;
  jog_msg_.data.assign(kJogStateSize, 0.0);
  jog_pub_ = std::make_unique<realtime_tools::RealtimePublisher<JogState>>(
    get_node(), "~/jog_state", rclcpp::SystemDefaultsQoS());

  twist_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/twist", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::TwistStamped & msg) { on_twist(msg); });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "configured: jogging %s in the URDF root frame, w0 %.4f, lambda_max %.3f, %s dynamics",
    params_->tip_frame.c_str(), params_->manipulability_threshold, params_->max_damping,
    model_->backend());
  return CallbackReturn::SUCCESS;
}

void TcpJogController::apply_params()
{
  for (std::size_t j = 0; j < pid_.size(); ++j) {
    pid_[j].kp = params_->kp[j];
    pid_[j].ki = params_->ki[j];
    pid_[j].kd = params_->kd[j];
    pid_[j].i_clamp = params_->i_clamp[j];
  }
  effort_limit_ = urdf_effort_limit_ * params_->effort_limit_scale;
}

void TcpJogController::on_twist(const geometry_msgs::msg::TwistStamped & msg)
{
  // Non-realtime. The twist is the tip's velocity in the URDF root frame; anything else is refused
  // rather than silently reinterpreted.
  if (!msg.header.frame_id.empty() && msg.header.frame_id != command_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "twist ignored: frame_id '%s', expected '%s' or empty", msg.header.frame_id.c_str(),
      command_frame_.c_str());
    return;
  }
  const auto & l = msg.twist.linear;
  const auto & a = msg.twist.angular;
  Command c;
  c.twist = {l.x, l.y, l.z, a.x, a.y, a.z};
  for (const double x : c.twist) {
    if (!std::isfinite(x)) {
      RCLCPP_WARN(get_node()->get_logger(), "twist ignored: non-finite component");
      return;
    }
  }
  c.received = get_node()->now();
  const double stamp = rclcpp::Time(msg.header.stamp).seconds();
  c.stamp = stamp > 0.0 ? stamp : c.received.seconds();
  c.seq = next_seq_++;
  pending_.set(c);
}

CallbackReturn TcpJogController::on_activate(const rclcpp_lifecycle::State &)
{
  adopt_pending_model();
  if (!read_state()) {
    RCLCPP_ERROR(get_node()->get_logger(), "cannot read joint state on activation");
    return CallbackReturn::ERROR;
  }
  q_des_ = q_;  // start from where the arm is, at rest
  qd_des_.setZero();
  qdd_des_.setZero();
  for (auto & pid : pid_) {
    pid.reset();
  }
  // Discard any command that arrived while inactive.
  pending_.get([this](const Command & c) { command_ = c; });
  command_.twist.fill(0.0);
  command_.received = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
  return CallbackReturn::SUCCESS;
}

CallbackReturn TcpJogController::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Leave the last torque on the interface (see PidFfTrajectoryController::on_deactivate).
  return CallbackReturn::SUCCESS;
}

return_type TcpJogController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (param_listener_->try_update_params(*staged_)) {
    if (per_joint_sizes_ok(*staged_)) {
      *params_ = *staged_;  // same-size vectors: copy-assignment does not allocate
      apply_params();
    } else {
      RCLCPP_ERROR(
        get_node()->get_logger(), "parameter change rejected: per-joint array size != %zu",
        joints_.size());
    }
  }
  adopt_pending_model();
  read_state();
  (void)pending_.try_get([this](const Command & c) {
    if (c.seq != command_.seq) {
      command_ = c;
    }
  });

  const double dt = std::max(period.seconds(), 1e-6);
  Vector6 v = Vector6::Zero();
  const bool fresh = command_.seq != 0 &&
                     command_.received.get_clock_type() == time.get_clock_type() &&
                     (time - command_.received).seconds() <= params_->command_timeout;
  if (fresh) {
    v = Eigen::Map<const Vector6>(command_.twist.data());
  }

  // Inverse differential kinematics on the reference configuration.
  const Matrix6 jacobian = model_->tip_jacobian(q_des_);
  const double w = control::manipulability(jacobian);
  const double lambda_sq =
    control::damping_squared(w, params_->manipulability_threshold, params_->max_damping);
  Vector6 qd_new = control::damped_least_squares(jacobian, v, lambda_sq);

  const Vector6 qd_max = Eigen::Map<const Vector6>(params_->max_joint_velocity.data());
  const Vector6 qdd_max = Eigen::Map<const Vector6>(params_->max_joint_acceleration.data());
  const double s_vel = control::uniform_scale(qd_new, qd_max);
  qd_new *= s_vel;
  const Vector6 qd_prev = to6(qd_des_);
  const Vector6 dqd = qd_new - qd_prev;
  const double s_acc = control::uniform_scale(dqd, qdd_max * dt);
  qd_new = qd_prev + s_acc * dqd;
  const double s_lim = control::limit_scale(
    to6(q_des_), qd_new * dt, to6(lower_limit_), to6(upper_limit_), params_->limit_margin);
  qd_new *= s_lim;

  for (Eigen::Index j = 0; j < 6; ++j) {
    qdd_des_[j] = (qd_new[j] - qd_prev[j]) / dt;
    qd_des_[j] = qd_new[j];
    q_des_[j] += qd_new[j] * dt;
  }

  tau_ff_ = model_->rnea(q_des_, qd_des_, qdd_des_);
  for (std::size_t j = 0; j < pid_.size(); ++j) {
    const auto e = static_cast<Eigen::Index>(j);
    const double err = q_des_[e] - q_[e];
    const double err_dot = qd_des_[e] - qd_[e];
    tau_[e] = pid_[j].update(err, err_dot, dt, tau_ff_[e], effort_limit_[e]);

    msg_.reference.positions[j] = q_des_[e];
    msg_.reference.velocities[j] = qd_des_[e];
    msg_.reference.accelerations[j] = qdd_des_[e];
    msg_.reference.effort[j] = tau_ff_[e];
    msg_.feedback.positions[j] = q_[e];
    msg_.feedback.velocities[j] = qd_[e];
    msg_.error.positions[j] = err;
    msg_.error.velocities[j] = err_dot;
    msg_.output.effort[j] = tau_[e];
  }
  write_effort();
  publish_state(time);

  if (++jog_counter_ >= params_->state_publish_decimation) {
    jog_counter_ = 0;
    const Vector6 v_meas = Matrix6(model_->tip_jacobian(q_)) * to6(qd_);
    jog_msg_.data[0] = time.seconds();
    jog_msg_.data[1] = fresh ? command_.stamp : 0.0;
    jog_msg_.data[2] = w;
    jog_msg_.data[3] = std::sqrt(lambda_sq);
    jog_msg_.data[4] = s_vel * s_acc * s_lim;
    for (std::size_t k = 0; k < 6; ++k) {
      jog_msg_.data[5 + k] = v[static_cast<Eigen::Index>(k)];
      jog_msg_.data[11 + k] = v_meas[static_cast<Eigen::Index>(k)];
    }
    (void)jog_pub_->try_publish(jog_msg_);
  }
  return return_type::OK;
}

}  // namespace ur5e_teaching_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ur5e_teaching_controllers::TcpJogController, controller_interface::ControllerInterface)
