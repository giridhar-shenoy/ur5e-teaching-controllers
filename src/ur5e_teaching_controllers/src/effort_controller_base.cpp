// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "ur5e_teaching_controllers/effort_controller_base.hpp"

#include <cmath>
#include <limits>

#include "dynamics/pinocchio_model.hpp"
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
  const std::vector<std::string> & joints, const std::string & tip_frame, int publish_decimation)
{
  joints_ = joints;
  const std::string & urdf = get_robot_description();
  if (urdf.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty");
    return false;
  }
  try {
    model_ = std::make_unique<dynamics::PinocchioModel>(urdf, joints_, tip_frame);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return false;
  }
  const auto n = static_cast<Eigen::Index>(joints_.size());
  q_ = Eigen::VectorXd::Zero(n);
  qd_ = Eigen::VectorXd::Zero(n);
  tau_ = Eigen::VectorXd::Zero(n);
  effort_limit_ = model_->effort_limits();
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

void EffortControllerBase::write_zero_effort()
{
  for (auto & ci : command_interfaces_) {
    (void)ci.set_value(0.0);
  }
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
