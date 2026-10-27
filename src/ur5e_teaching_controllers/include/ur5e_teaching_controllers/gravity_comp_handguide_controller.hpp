// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Gravity-compensated hand guiding on the effort interface:
//
//   tau = g(q) [+ C(q, qd) qd] - D qd + tau_friction(qd) + tau_limits(q)
//
// g and C come from Pinocchio with the configured model payload (payload_mass, payload_com),
// which can be changed at runtime and is pushed into the payload body's inertia. The arm floats
// and a person (in simulation: an ApplyLinkWrench push on the wrist) moves it. Safety: a spring
// pushes joints back near their URDF position limits, and the controller deactivates itself with
// an error if any |qd| exceeds max_joint_velocity.
#pragma once

#include <memory>
#include <vector>

#include "ur5e_teaching_controllers/effort_controller_base.hpp"

namespace gravity_comp_handguide_controller
{
class ParamListener;
struct Params;
}  // namespace gravity_comp_handguide_controller

namespace ur5e_teaching_controllers
{

class GravityCompHandGuidingController : public EffortControllerBase
{
public:
  GravityCompHandGuidingController();
  ~GravityCompHandGuidingController() override;

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
  void apply_params();

  std::shared_ptr<gravity_comp_handguide_controller::ParamListener> param_listener_;
  std::unique_ptr<gravity_comp_handguide_controller::Params> params_;
  std::vector<double> qd_vec_;
  Eigen::VectorXd tau_model_;
};

}  // namespace ur5e_teaching_controllers
