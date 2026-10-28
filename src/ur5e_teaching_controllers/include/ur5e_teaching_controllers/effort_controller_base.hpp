// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Plumbing shared by the effort-interface controllers: claim <joint>/effort, read
// <joint>/position and <joint>/velocity, build the dynamics model (Pinocchio or KDL backend)
// from robot_description, swap in payload changes without touching the model inside update(),
// and publish a control_msgs/JointTrajectoryControllerState on ~/controller_state from the
// realtime loop without blocking it.
#pragma once

#include <Eigen/Core>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

namespace ur5e_teaching_controllers
{
namespace dynamics
{
class DynamicsModel;
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
    const std::vector<std::string> & joints, const std::string & tip_frame,
    const std::string & dynamics_backend, double gravity_z, double payload_mass,
    const std::vector<double> & payload_com, int publish_decimation);
  /// Non-realtime (parameter callback): if the payload differs from the last one requested,
  /// build a model with it (KdlModel rebuilds its solvers here) and queue it for update().
  void request_payload(double payload_mass, const std::vector<double> & payload_com);
  /// Realtime: adopt the most recently queued model, if any. Never blocks, allocates or frees:
  /// replaced models stay referenced in models_ until request_payload() releases them.
  void adopt_pending_model();
  /// Read positions and velocities into q_ / qd_. Returns false if any read failed.
  bool read_state();
  /// Write tau_ to the effort interfaces. Returns false if any write failed.
  bool write_effort();
  /// Publish msg_ every publish_decimation_ calls; fill msg_ before calling.
  void publish_state(const rclcpp::Time & time);

  std::vector<std::string> joints_;
  /// The model update() uses; only update() and on_activate() replace it.
  std::shared_ptr<dynamics::DynamicsModel> model_;
  /// URDF limits, copied at configure so non-realtime code never reads model_.
  Eigen::VectorXd lower_limit_, upper_limit_, urdf_effort_limit_;
  Eigen::VectorXd q_, qd_, tau_, effort_limit_;
  StateMsg msg_;

private:
  struct PendingModel
  {
    std::shared_ptr<dynamics::DynamicsModel> model;
  };
  realtime_tools::RealtimeThreadSafeBox<PendingModel> pending_model_;
  std::mutex models_mutex_;                                       // non-realtime side only
  std::shared_ptr<const dynamics::DynamicsModel> prototype_;      // payload-free; cloned per change
  std::vector<std::shared_ptr<dynamics::DynamicsModel>> models_;  // keeps swapped models alive
  double requested_mass_{0.0};
  std::vector<double> requested_com_;

  std::unique_ptr<realtime_tools::RealtimePublisher<StateMsg>> state_pub_;
  int publish_decimation_{1};
  int publish_counter_{0};
};

}  // namespace ur5e_teaching_controllers
