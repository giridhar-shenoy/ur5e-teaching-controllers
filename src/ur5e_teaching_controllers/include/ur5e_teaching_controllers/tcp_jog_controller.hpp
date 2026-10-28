// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Cartesian (TCP) jogging on the effort interface. A geometry_msgs/TwistStamped on ~/twist is the
// tip frame's linear and angular velocity in the URDF root frame. Every cycle:
//
//   qd_new  = J(q_des)^T (J J^T + lambda^2 I)^-1 v     damped least squares, lambda scheduled on
//                                                      the manipulability w (control/dls.hpp)
//   qd_new  scaled uniformly to the joint speed limits, then its change to the acceleration limits,
//           then so that no joint passes limit_margin before a URDF position limit
//   q_des  += qd_new dt                                the joint reference is integrated
//   tau     = PID(q_des - q, qd_des - qd) + RNEA(q_des, qd_des, qdd_des)
//
// Uniform scaling keeps the TCP moving in the commanded direction, only slower. Without a new
// twist for command_timeout the command is zero and the arm decelerates at the acceleration
// limit (deadman). ~/jog_state (std_msgs/Float64MultiArray, every published cycle):
//   [t, t_cmd, w, lambda, scale, v_cmd(6), v_meas(6)]
// t: this update's time [s]; t_cmd: header stamp of the command in use (its receipt time if the
// stamp is zero; 0 when there is none); scale: product of the three scalings; v_meas = J(q) qd
// from the measured state. Times are the controller manager's clock (sim time in Gazebo), so a
// benchmark can measure command-to-motion latency without a wall-clock in the loop.
// Only for 6-DoF arms: the inverse kinematics uses fixed 6x6 matrices so update() never allocates.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ur5e_teaching_controllers/effort_controller_base.hpp"

namespace tcp_jog_controller
{
class ParamListener;
struct Params;
}  // namespace tcp_jog_controller

namespace ur5e_teaching_controllers
{
namespace control
{
struct Pid;
}

class TcpJogController : public EffortControllerBase
{
public:
  TcpJogController();
  ~TcpJogController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Layout of ~/jog_state.
  static constexpr std::size_t kJogStateSize = 17;

private:
  struct Command
  {
    std::array<double, 6> twist{};
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
    double stamp{0.0};  // header stamp [s], or the receipt time if the header has none
    std::uint64_t seq{0};
  };

  void on_twist(const geometry_msgs::msg::TwistStamped & msg);
  void apply_params();

  std::shared_ptr<tcp_jog_controller::ParamListener> param_listener_;
  std::unique_ptr<tcp_jog_controller::Params> params_;
  std::unique_ptr<tcp_jog_controller::Params> staged_;
  std::vector<control::Pid> pid_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  realtime_tools::RealtimeThreadSafeBox<Command> pending_;
  std::uint64_t next_seq_{1};
  Command command_;            // realtime copy of the latest command
  std::string command_frame_;  // copied at configure; read by the subscription callback

  using JogState = std_msgs::msg::Float64MultiArray;
  std::unique_ptr<realtime_tools::RealtimePublisher<JogState>> jog_pub_;
  JogState jog_msg_;
  int jog_counter_{0};

  Eigen::VectorXd q_des_, qd_des_, qdd_des_, tau_ff_;
};

}  // namespace ur5e_teaching_controllers
