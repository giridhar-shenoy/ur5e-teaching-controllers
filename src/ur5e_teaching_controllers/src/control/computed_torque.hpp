// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Computed-torque (inverse dynamics) control, as in Spong, Hutchinson and Vidyasagar, "Robot
// Modeling and Control", inverse dynamics control:
//
//   a   = qdd_des + Kp (q_des - q) + Kd (qd_des - qd)       outer loop, per joint
//   tau = RNEA(q, qd, a) = M(q) a + C(q, qd) qd + g(q)      inner loop, measured state
//
// With an exact model, M(q) qdd + C qd + g = tau gives qdd = a, so every joint's error obeys
// e'' + Kd e' + Kp e = 0, decoupled from the others. Kp = w^2 and Kd = 2 zeta w place both poles
// of joint j at natural frequency w_j and damping ratio zeta_j. No integral term (see
// docs/controllers.md). Allocation-free: every vector is pre-sized by the caller.
#pragma once

#include <Eigen/Core>

#include "control/control_laws.hpp"
#include "dynamics/dynamics_model.hpp"

namespace ur5e_teaching_controllers::control
{

/// Outer-loop gains from natural frequency w [rad/s] and damping ratio zeta, per joint.
inline void computed_torque_gains(
  const Eigen::VectorXd & omega, const Eigen::VectorXd & zeta, Eigen::VectorXd & kp,
  Eigen::VectorXd & kd)
{
  kp = omega.cwiseProduct(omega);
  kd = 2.0 * zeta.cwiseProduct(omega);
}

/// tau = sat(RNEA(q, qd, qdd_des + kp e + kd e_dot)); `a` is scratch for the commanded
/// acceleration and `tau_model` receives the unsaturated model torque.
inline void computed_torque(
  dynamics::DynamicsModel & model, const Eigen::VectorXd & q, const Eigen::VectorXd & qd,
  const Eigen::VectorXd & q_des, const Eigen::VectorXd & qd_des, const Eigen::VectorXd & qdd_des,
  const Eigen::VectorXd & kp, const Eigen::VectorXd & kd, const Eigen::VectorXd & effort_limit,
  Eigen::VectorXd & a, Eigen::VectorXd & tau_model, Eigen::VectorXd & tau)
{
  a = qdd_des + kp.cwiseProduct(q_des - q) + kd.cwiseProduct(qd_des - qd);
  tau_model = model.rnea(q, qd, a);
  for (Eigen::Index j = 0; j < tau.size(); ++j) {
    tau[j] = Pid::saturate(tau_model[j], effort_limit[j]);
  }
}

}  // namespace ur5e_teaching_controllers::control
