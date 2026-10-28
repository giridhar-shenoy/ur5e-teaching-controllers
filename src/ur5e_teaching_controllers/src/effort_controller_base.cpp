// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "ur5e_teaching_controllers/effort_controller_base.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "dynamics/dynamics_model.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ur5e_teaching_controllers
{

EffortControllerBase::EffortControllerBase() = default;
EffortControllerBase::~EffortControllerBase() = default;

controller_interface::InterfaceConfiguration EffortControllerBase::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration EffortControllerBase::state_interface_configuration()
  const
{
  // Order matters: read_state() expects [pos_0, vel_0, pos_1, vel_1, ...].
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

bool EffortControllerBase::configure_common(
  const std::vector<std::string> & joints, const std::string & tip_frame,
  const std::string & dynamics_backend, double gravity_z, double payload_mass,
  const std::vector<double> & payload_com, int publish_decimation)
{
  joints_ = joints;
  const std::string & urdf = get_robot_description();
  if (urdf.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty");
    return false;
  }
  try {
    prototype_ =
      dynamics::make_dynamics_model(dynamics_backend, urdf, joints_, tip_frame, gravity_z);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(models_mutex_);
    models_.clear();
    requested_com_.clear();  // forces the build below
  }
  pending_model_.set(PendingModel{});
  request_payload(payload_mass, payload_com);
  pending_model_.get([this](const PendingModel & pending) { model_ = pending.model; });
  if (!model_) {
    RCLCPP_ERROR(get_node()->get_logger(), "could not build the dynamics model");
    return false;
  }
  RCLCPP_INFO(
    get_node()->get_logger(), "dynamics backend: %s, gravity_z %.4f m/s^2", model_->backend(),
    model_->gravity_z());

  const auto n = static_cast<Eigen::Index>(joints_.size());
  q_ = Eigen::VectorXd::Zero(n);
  qd_ = Eigen::VectorXd::Zero(n);
  tau_ = Eigen::VectorXd::Zero(n);
  lower_limit_ = prototype_->lower_limits();
  upper_limit_ = prototype_->upper_limits();
  urdf_effort_limit_ = prototype_->effort_limits();
  effort_limit_ = urdf_effort_limit_;
  for (Eigen::Index j = 0; j < effort_limit_.size(); ++j) {
    // Saturation treats a limit <= 0 as "no limit"; a joint without a URDF effort limit must not
    // silently end up unlimited.
    if (!(std::isfinite(effort_limit_[j]) && effort_limit_[j] > 0.0)) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "joint %s has no positive finite effort limit in the URDF",
        joints_[static_cast<std::size_t>(j)].c_str());
      return false;
    }
  }

  publish_decimation_ = std::max(1, publish_decimation);
  state_pub_ = std::make_unique<realtime_tools::RealtimePublisher<StateMsg>>(
    get_node(), "~/controller_state", rclcpp::SystemDefaultsQoS());

  // Pre-size every field once so the realtime copy in try_publish() never allocates.
  const auto size = joints_.size();
  msg_.joint_names = joints_;
  for (auto * point : {&msg_.reference, &msg_.feedback, &msg_.error, &msg_.output}) {
    point->positions.assign(size, std::numeric_limits<double>::quiet_NaN());
    point->velocities.assign(size, std::numeric_limits<double>::quiet_NaN());
    point->accelerations.assign(size, std::numeric_limits<double>::quiet_NaN());
    point->effort.assign(size, std::numeric_limits<double>::quiet_NaN());
  }
  return true;
}

void EffortControllerBase::request_payload(
  double payload_mass, const std::vector<double> & payload_com)
{
  std::lock_guard<std::mutex> lock(models_mutex_);
  if (!prototype_ || (payload_mass == requested_mass_ && payload_com == requested_com_)) {
    return;
  }
  std::shared_ptr<dynamics::DynamicsModel> next;
  try {
    next = prototype_->clone();
    next->set_payload(
      payload_mass, Eigen::Vector3d(payload_com.at(0), payload_com.at(1), payload_com.at(2)));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "payload change rejected: %s", e.what());
    return;
  }
  requested_mass_ = payload_mass;
  requested_com_ = payload_com;
  // Release models that neither the realtime loop nor the box still holds (use_count 1 = only
  // this list). The loop only ever takes the model in the box, which is always the newest.
  models_.erase(
    std::remove_if(
      models_.begin(), models_.end(), [](const auto & m) { return m.use_count() == 1; }),
    models_.end());
  models_.push_back(next);
  pending_model_.set(PendingModel{std::move(next)});
}

void EffortControllerBase::adopt_pending_model()
{
  (void)pending_model_.try_get([this](const PendingModel & pending) {
    if (pending.model && pending.model != model_) {
      model_ = pending.model;  // reference counts only; models_ still owns the old one
    }
  });
}

bool EffortControllerBase::read_state()
{
  bool ok = true;
  for (std::size_t j = 0; j < joints_.size(); ++j) {
    const auto p = state_interfaces_[2 * j].get_optional<double>();
    const auto v = state_interfaces_[2 * j + 1].get_optional<double>();
    const auto e = static_cast<Eigen::Index>(j);
    if (p && v) {
      q_[e] = *p;
      qd_[e] = *v;
    } else {
      ok = false;  // keep the previous sample
    }
  }
  return ok;
}

bool EffortControllerBase::write_effort()
{
  bool ok = true;
  for (std::size_t j = 0; j < joints_.size(); ++j) {
    ok &= command_interfaces_[j].set_value(tau_[static_cast<Eigen::Index>(j)]);
  }
  return ok;
}

void EffortControllerBase::publish_state(const rclcpp::Time & time)
{
  if (++publish_counter_ < publish_decimation_) {
    return;
  }
  publish_counter_ = 0;
  msg_.header.stamp = time;
  (void)state_pub_->try_publish(msg_);
}

}  // namespace ur5e_teaching_controllers
