// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Plumbing shared by the effort-interface controllers: claim <joint>/effort, read
// <joint>/position and <joint>/velocity, build the Pinocchio model from robot_description, and
// publish a control_msgs/JointTrajectoryControllerState on ~/controller_state from the realtime
// loop without blocking it.
#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace ur5e_teaching_controllers
{
namespace dynamics
{
class PinocchioModel;
}

class EffortControllerBase : public controller_interface::ControllerInterface
{
public:
  EffortControllerBase();
  ~EffortControllerBase() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  using StateMsg = control_msgs::msg::JointTrajectoryControllerState;

  /// Build the model and the publisher; call from on_configure of the derived controller.
  bool configure_common(
    const std::vector<std::string> & joints, const std::string & tip_frame, int publish_decimation);
  /// Read positions and velocities into q_ / qd_. Returns false if any read failed.
  bool read_state();
  /// Write tau_ to the effort interfaces. Returns false if any write failed.
  bool write_effort();
  /// Write zero effort (used on deactivate).
  void write_zero_effort();
  /// Publish msg_ every publish_decimation_ calls; fill msg_ before calling.
  void publish_state(const rclcpp::Time & time);

  std::vector<std::string> joints_;
  std::unique_ptr<dynamics::PinocchioModel> model_;
  Eigen::VectorXd q_, qd_, tau_, effort_limit_;
  StateMsg msg_;

private:
  std::unique_ptr<realtime_tools::RealtimePublisher<StateMsg>> state_pub_;
  int publish_decimation_{1};
  int publish_counter_{0};
};

}  // namespace ur5e_teaching_controllers
