// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Joint-space trajectory tracking on the effort interface, with one of two control laws
// (parameter control_law, switchable at runtime):
//
//   pid_ff:           tau = Kp e + Kd e_dot + Ki integral(e) + tau_ff,
//                     tau_ff = RNEA(q_des, qd_des, qdd_des)   (feedforward: false -> tau_ff = 0)
//   computed_torque:  tau = RNEA(q, qd, qdd_des + Kp' e + Kd' e_dot),
//                     Kp' = w^2, Kd' = 2 zeta w per joint (ct_omega, ct_zeta)
//
// Input: trajectory_msgs/JointTrajectory on ~/joint_trajectory (topic only; no action server in
// v0.1). Each segment is re-profiled as a velocity/acceleration-limited trapezoid; a new message
// pre-empts the running trajectory and starts from the current desired state (rest-to-rest
// profiles: pre-empting while moving steps the desired velocity to zero, see docs). Without a
// trajectory the controller holds the position it had when it was activated.
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "ur5e_teaching_controllers/effort_controller_base.hpp"

namespace pid_ff_trajectory_controller
{
class ParamListener;
struct Params;
}  // namespace pid_ff_trajectory_controller

namespace ur5e_teaching_controllers
{
namespace control
{
struct Pid;
}
namespace profiles
{
class TrapezoidalTrajectory;
struct JointSample;
}  // namespace profiles

class PidFfTrajectoryController : public EffortControllerBase
{
public:
  PidFfTrajectoryController();
  ~PidFfTrajectoryController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Pending
  {
    std::uint64_t id{0};
    std::shared_ptr<const profiles::TrapezoidalTrajectory> trajectory;
  };

  enum class Law
  {
    kPidFf,
    kComputedTorque
  };

  void on_trajectory(const trajectory_msgs::msg::JointTrajectory & msg);
  void apply_params();

  std::shared_ptr<pid_ff_trajectory_controller::ParamListener> param_listener_;
  std::unique_ptr<pid_ff_trajectory_controller::Params> params_;
  std::unique_ptr<pid_ff_trajectory_controller::Params> staged_;  // runtime changes land here
  std::vector<control::Pid> pid_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  realtime_tools::RealtimeThreadSafeBox<Pending> pending_;
  std::uint64_t next_id_{1};
  std::uint64_t active_id_{0};
  std::shared_ptr<const profiles::TrapezoidalTrajectory> active_;
  // Owned by the subscription callback only; see on_trajectory().
  std::vector<std::shared_ptr<const profiles::TrapezoidalTrajectory>> keep_alive_;
  bool start_pending_{false};
  rclcpp::Time t_start_;

  // Desired state handed from the realtime loop to the subscription callback (trajectory start).
  realtime_tools::RealtimeThreadSafeBox<std::vector<double>> latest_desired_;
  std::vector<double> desired_scratch_;

  std::vector<profiles::JointSample> samples_;
  Eigen::VectorXd q_des_, qd_des_, qdd_des_, tau_ff_;

  Law law_{Law::kPidFf};
  Eigen::VectorXd ct_omega_, ct_zeta_, ct_kp_, ct_kd_, a_cmd_;  // computed-torque outer loop
};

}  // namespace ur5e_teaching_controllers
