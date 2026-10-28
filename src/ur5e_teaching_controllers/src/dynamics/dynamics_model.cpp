// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "dynamics/dynamics_model.hpp"

#include <stdexcept>

#include "dynamics/kdl_model.hpp"
#include "dynamics/pinocchio_model.hpp"

namespace ur5e_teaching_controllers::dynamics
{

std::unique_ptr<DynamicsModel> make_dynamics_model(
  const std::string & backend, const std::string & urdf,
  const std::vector<std::string> & joint_names, const std::string & tip_frame, double gravity_z)
{
  if (backend == "pinocchio") {
    return std::make_unique<PinocchioModel>(urdf, joint_names, tip_frame, gravity_z);
  }
  if (backend == "kdl") {
    return std::make_unique<KdlModel>(urdf, joint_names, tip_frame, gravity_z);
  }
  throw std::runtime_error(
    "unknown dynamics backend '" + backend + "' (expected 'pinocchio' or 'kdl')");
}

}  // namespace ur5e_teaching_controllers::dynamics
