// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Small, allocation-free building blocks shared by the controllers and unit-tested on their own:
// a PID with conditional-integration anti-windup, torque saturation, joint-limit repulsion and
// the speed-cap check.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace ur5e_teaching_controllers::control
{

/// One joint's PID state.
///
/// u = kp e + kd e_dot + ki * integral. The integral is clamped to |ki * integral| <= i_clamp and
/// is frozen whenever the total command is saturated in the direction that integrating would
/// make worse (conditional integration), so it cannot wind up against a torque limit.
struct Pid
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double i_clamp{0.0};
  double integral{0.0};

  void reset() { integral = 0.0; }

  /// @param feedforward added before saturation (so the integrator sees the real headroom)
  /// @param u_limit     symmetric saturation of the final command (<= 0 disables it)
  /// @return saturated command
  double update(double error, double error_dot, double dt, double feedforward, double u_limit)
  {
    const double candidate = integral + error * dt;
    const double i_term_candidate = ki * candidate;
    const double i_term_clamped =
      (i_clamp > 0.0) ? std::clamp(i_term_candidate, -i_clamp, i_clamp) : i_term_candidate;
    const double u_unsat = kp * error + kd * error_dot + i_term_clamped + feedforward;
    const bool saturating_up = u_limit > 0.0 && u_unsat > u_limit && error > 0.0;
    const bool saturating_down = u_limit > 0.0 && u_unsat < -u_limit && error < 0.0;
    if (!saturating_up && !saturating_down) {
      integral = (ki != 0.0) ? i_term_clamped / ki : 0.0;
    }
    const double u = kp * error + kd * error_dot + ki * integral + feedforward;
    return saturate(u, u_limit);
  }

  static double saturate(double u, double limit)
  {
    return (limit > 0.0) ? std::clamp(u, -limit, limit) : u;
  }
};

/// Spring torque pushing a joint back once it is within `margin` of a position limit.
/// Zero in the safe interior; -stiffness * penetration near the upper limit and
/// +stiffness * penetration near the lower limit. Infinite limits disable it.
inline double limit_repulsion(double q, double lower, double upper, double margin, double stiffness)
{
  if (margin <= 0.0 || stiffness <= 0.0) {
    return 0.0;
  }
  if (std::isfinite(upper) && q > upper - margin) {
    return -stiffness * (q - (upper - margin));
  }
  if (std::isfinite(lower) && q < lower + margin) {
    return stiffness * ((lower + margin) - q);
  }
  return 0.0;
}

/// Index of the first joint whose |velocity| exceeds its cap, or -1 if none does.
inline int first_speed_violation(
  const std::vector<double> & velocity, const std::vector<double> & cap)
{
  const std::size_t n = std::min(velocity.size(), cap.size());
  for (std::size_t j = 0; j < n; ++j) {
    if (cap[j] > 0.0 && std::abs(velocity[j]) > cap[j]) {
      return static_cast<int>(j);
    }
  }
  return -1;
}

}  // namespace ur5e_teaching_controllers::control
