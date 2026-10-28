// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "ur5e_teaching_controllers/pid_ff_trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "control/computed_torque.hpp"
#include "control/control_laws.hpp"
#include "dynamics/dynamics_model.hpp"
#include "profiles/trapezoidal.hpp"
#include "ur5e_teaching_controllers/pid_ff_trajectory_controller_parameters.hpp"

namespace ur5e_teaching_controllers
{

using controller_interface::CallbackReturn;
using controller_interface::return_type;

PidFfTrajectoryController::PidFfTrajectoryController() = default;
PidFfTrajectoryController::~PidFfTrajectoryController() = default;

CallbackReturn PidFfTrajectoryController::on_init()
{
  try {
    param_listener_ = std::make_shared<pid_ff_trajectory_controller::ParamListener>(get_node());
    params_ = std::make_unique<pid_ff_trajectory_controller::Params>(param_listener_->get_params());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "parameter init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  joints_ = params_->joints;  // needed by the interface configuration before on_configure
  return CallbackReturn::SUCCESS;
}

namespace
{
// Per-joint arrays must have one entry per joint; generate_parameter_library cannot express
// "same size as joints", so this is checked on configure and again for every runtime change.
bool per_joint_sizes_ok(const pid_ff_trajectory_controller::Params & p)
{
  const std::size_t n = p.joints.size();
  for (const auto * v :
       {&p.kp, &p.ki, &p.kd, &p.i_clamp, &p.max_velocity, &p.max_acceleration, &p.ct_omega,
        &p.ct_zeta}) {
    if (v->size() != n) {
      return false;
    }
  }
  return true;
}
}  // namespace

CallbackReturn PidFfTrajectoryController::on_configure(const rclcpp_lifecycle::State &)
{
  *params_ = param_listener_->get_params();
  const std::size_t n = params_->joints.size();
  if (!per_joint_sizes_ok(*params_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "every per-joint parameter needs %zu entries (one per joint)", n);
    return CallbackReturn::ERROR;
  }
  staged_ = std::make_unique<pid_ff_trajectory_controller::Params>(*params_);
  if (!configure_common(
        params_->joints, params_->tip_frame, params_->dynamics_backend, params_->gravity_z,
        params_->payload_mass, params_->payload_com, params_->state_publish_decimation)) {
    return CallbackReturn::ERROR;
  }
  // Payload changes: the model is rebuilt here, in the parameter service's thread, and update()
  // only swaps it in (see EffortControllerBase::request_payload).
  param_listener_->setUserCallback([this](const pid_ff_trajectory_controller::Params & p) {
    request_payload(p.payload_mass, p.payload_com);
  });
  pid_.assign(n, control::Pid{});
  const auto en = static_cast<Eigen::Index>(n);
  ct_kp_ = Eigen::VectorXd::Zero(en);
  ct_kd_ = Eigen::VectorXd::Zero(en);
  ct_omega_ = Eigen::VectorXd::Zero(en);
  ct_zeta_ = Eigen::VectorXd::Zero(en);
  a_cmd_ = Eigen::VectorXd::Zero(en);
  law_ = Law::kPidFf;
  apply_params();

  q_des_ = Eigen::VectorXd::Zero(en);
  qd_des_ = Eigen::VectorXd::Zero(en);
  qdd_des_ = Eigen::VectorXd::Zero(en);
  tau_ff_ = Eigen::VectorXd::Zero(en);
  samples_.resize(n);
  desired_scratch_.assign(n, 0.0);
  latest_desired_.set(desired_scratch_);

  trajectory_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory", rclcpp::SystemDefaultsQoS(),
    [this](const trajectory_msgs::msg::JointTrajectory & msg) { on_trajectory(msg); });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "configured: %zu joints, control law %s, feedforward %s, model payload %.2f kg, %s dynamics", n,
    params_->control_law.c_str(), params_->feedforward ? "on" : "off", params_->payload_mass,
    model_->backend());
  return CallbackReturn::SUCCESS;
}

void PidFfTrajectoryController::apply_params()
{
  for (std::size_t j = 0; j < pid_.size(); ++j) {
    pid_[j].kp = params_->kp[j];
    pid_[j].ki = params_->ki[j];
    pid_[j].kd = params_->kd[j];
    pid_[j].i_clamp = params_->i_clamp[j];
    ct_omega_[static_cast<Eigen::Index>(j)] = params_->ct_omega[j];
    ct_zeta_[static_cast<Eigen::Index>(j)] = params_->ct_zeta[j];
  }
  control::computed_torque_gains(ct_omega_, ct_zeta_, ct_kp_, ct_kd_);
  effort_limit_ = urdf_effort_limit_ * params_->effort_limit_scale;
  const Law law = params_->control_law == "computed_torque" ? Law::kComputedTorque : Law::kPidFf;
  if (law != law_) {
    // The integral belongs to the PID law only; coming back to it starts from zero instead of
    // from whatever it held when the other law took over.
    for (auto & pid : pid_) {
      pid.reset();
    }
    law_ = law;
  }
}

void PidFfTrajectoryController::on_trajectory(const trajectory_msgs::msg::JointTrajectory & msg)
{
  // Non-realtime: validate, reorder to our joint order and build the profile here.
  const std::size_t n = joints_.size();
  std::unordered_map<std::string, std::size_t> column;
  for (std::size_t i = 0; i < msg.joint_names.size(); ++i) {
    column[msg.joint_names[i]] = i;
  }
  std::vector<std::size_t> order(n);
  for (std::size_t j = 0; j < n; ++j) {
    const auto it = column.find(joints_[j]);
    if (it == column.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "trajectory rejected: missing joint %s", joints_[j].c_str());
      return;
    }
    order[j] = it->second;
  }
  if (msg.points.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "trajectory rejected: no points");
    return;
  }
  std::vector<std::vector<double>> waypoints;
  std::vector<double> times;
  const auto & lower = lower_limit_;
  const auto & upper = upper_limit_;
  for (const auto & point : msg.points) {
    if (point.positions.size() != msg.joint_names.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "trajectory rejected: point size mismatch");
      return;
    }
    std::vector<double> wp(n);
    for (std::size_t j = 0; j < n; ++j) {
      wp[j] = point.positions[order[j]];
      const auto e = static_cast<Eigen::Index>(j);
      // NaN fails both comparisons, so test for "inside" rather than "outside".
      if (!(wp[j] >= lower[e] && wp[j] <= upper[e])) {
        RCLCPP_ERROR(
          get_node()->get_logger(), "trajectory rejected: %s target %f outside [%f, %f]",
          joints_[j].c_str(), wp[j], lower[e], upper[e]);
        return;
      }
    }
    waypoints.push_back(std::move(wp));
    times.push_back(rclcpp::Duration(point.time_from_start).seconds());
  }
  const auto params = param_listener_->get_params();
  std::vector<double> start = latest_desired_.get();
  try {
    auto trajectory = std::make_shared<const profiles::TrapezoidalTrajectory>(
      start, waypoints, times, params.max_velocity, params.max_acceleration);
    RCLCPP_INFO(
      get_node()->get_logger(),
      "trajectory accepted: %zu points, %.3f s requested, %.3f s profiled", waypoints.size(),
      times.back(), trajectory->duration());
    // Keep a reference here so the realtime loop never drops the last one (no free in update()).
    keep_alive_.erase(
      std::remove_if(
        keep_alive_.begin(), keep_alive_.end(), [](const auto & t) { return t.use_count() == 1; }),
      keep_alive_.end());
    keep_alive_.push_back(trajectory);
    pending_.set(Pending{next_id_++, std::move(trajectory)});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "trajectory rejected: %s", e.what());
  }
}

CallbackReturn PidFfTrajectoryController::on_activate(const rclcpp_lifecycle::State &)
{
  adopt_pending_model();
  if (!read_state()) {
    RCLCPP_ERROR(get_node()->get_logger(), "cannot read joint state on activation");
    return CallbackReturn::ERROR;
  }
  q_des_ = q_;  // hold where we are
  qd_des_.setZero();
  qdd_des_.setZero();
  for (auto & pid : pid_) {
    pid.reset();
  }
  active_.reset();
  start_pending_ = false;
  // Discard anything that arrived while inactive.
  pending_.get([this](const Pending & p) { active_id_ = p.id; });
  for (std::size_t j = 0; j < desired_scratch_.size(); ++j) {
    desired_scratch_[j] = q_des_[static_cast<Eigen::Index>(j)];
  }
  latest_desired_.set(desired_scratch_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn PidFfTrajectoryController::on_deactivate(const rclcpp_lifecycle::State &)
{
  // The last torque stays on the interface until the next controller writes: zeroing it here
  // dropped the arm under gravity for one cycle at every hand-over between effort controllers.
  active_.reset();
  return CallbackReturn::SUCCESS;
}

return_type PidFfTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Non-blocking: skips this cycle if a parameter change is being written right now.
  if (param_listener_->try_update_params(*staged_)) {
    if (per_joint_sizes_ok(*staged_)) {
      *params_ = *staged_;  // same-size vectors: copy-assignment does not allocate
      apply_params();       // gains and torque limits; the payload arrives as a new model
    } else {
      RCLCPP_ERROR(
        get_node()->get_logger(), "parameter change rejected: per-joint array size != %zu",
        joints_.size());
    }
  }
  adopt_pending_model();
  read_state();

  // Adopt a newly received trajectory (never blocks: try_get skips if the callback holds it).
  // The profile was built from the desired state the callback last saw; if that is no longer
  // where we are (e.g. it was read before this activation), starting it would step q_des.
  pending_.try_get([this](const Pending & p) {
    if (p.id == active_id_ || !p.trajectory) {
      return;
    }
    active_id_ = p.id;
    const auto & start = p.trajectory->start();
    double deviation = 0.0;
    for (std::size_t j = 0; j < start.size(); ++j) {
      deviation = std::max(deviation, std::abs(start[j] - q_des_[static_cast<Eigen::Index>(j)]));
    }
    if (deviation > params_->max_start_deviation) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "trajectory dropped: it starts %.4f rad from the current desired state", deviation);
      return;
    }
    active_ = p.trajectory;
    start_pending_ = true;
  });
  if (start_pending_) {
    t_start_ = time;
    start_pending_ = false;
  }

  if (active_) {
    active_->sample((time - t_start_).seconds(), samples_);
    for (std::size_t j = 0; j < samples_.size(); ++j) {
      const auto e = static_cast<Eigen::Index>(j);
      q_des_[e] = samples_[j].position;
      qd_des_[e] = samples_[j].velocity;
      qdd_des_[e] = samples_[j].acceleration;
    }
  }

  if (law_ == Law::kComputedTorque) {
    // tau = RNEA(q, qd, qdd_des + Kp e + Kd e_dot) on the measured state; tau_ff_ receives the
    // unsaturated model torque (published as reference.effort).
    control::computed_torque(
      *model_, q_, qd_, q_des_, qd_des_, qdd_des_, ct_kp_, ct_kd_, effort_limit_, a_cmd_, tau_ff_,
      tau_);
  } else {
    if (params_->feedforward) {
      tau_ff_ = model_->rnea(q_des_, qd_des_, qdd_des_);
    } else {
      tau_ff_.setZero();
    }
    const double dt = std::max(period.seconds(), 1e-6);
    for (std::size_t j = 0; j < pid_.size(); ++j) {
      const auto e = static_cast<Eigen::Index>(j);
      tau_[e] =
        pid_[j].update(q_des_[e] - q_[e], qd_des_[e] - qd_[e], dt, tau_ff_[e], effort_limit_[e]);
    }
  }

  for (std::size_t j = 0; j < pid_.size(); ++j) {
    const auto e = static_cast<Eigen::Index>(j);
    const double err = q_des_[e] - q_[e];
    const double err_dot = qd_des_[e] - qd_[e];
    msg_.reference.positions[j] = q_des_[e];
    msg_.reference.velocities[j] = qd_des_[e];
    msg_.reference.accelerations[j] = qdd_des_[e];
    msg_.reference.effort[j] = tau_ff_[e];
    msg_.feedback.positions[j] = q_[e];
    msg_.feedback.velocities[j] = qd_[e];
    msg_.error.positions[j] = err;
    msg_.error.velocities[j] = err_dot;
    msg_.output.effort[j] = tau_[e];
    desired_scratch_[j] = q_des_[e];
  }
  write_effort();
  (void)latest_desired_.try_set(desired_scratch_);
  publish_state(time);
  return return_type::OK;
}

}  // namespace ur5e_teaching_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ur5e_teaching_controllers::PidFfTrajectoryController, controller_interface::ControllerInterface)
