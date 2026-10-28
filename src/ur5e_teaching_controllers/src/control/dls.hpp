// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Damped least squares (DLS) inverse differential kinematics for a 6-DoF arm, with the damping
// scheduled on the manipulability measure (Nakamura and Hanafusa 1986; Chiaverini 1997):
//
//   qd = J^T (J J^T + lambda^2 I)^-1 v,   lambda^2 = lambda_max^2 (1 - (w / w0)^2) if w < w0,
//                                          lambda^2 = 0 otherwise,     w = sqrt(det(J J^T))
//
// Far from singularities this is the exact inverse; close to one the damping trades tracking
// accuracy for bounded joint speed (|qd| <= |v| / (2 lambda)). Fixed 6x6 sizes, so nothing here
// allocates.
#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>

namespace ur5e_teaching_controllers::control
{

using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Vector6 = Eigen::Matrix<double, 6, 1>;

/// Yoshikawa's manipulability w = sqrt(det(J J^T)); |det J| for a square Jacobian.
inline double manipulability(const Matrix6 & jacobian) { return std::abs(jacobian.determinant()); }

/// lambda^2 from the manipulability schedule; zero at or above w0 (or when w0 <= 0).
inline double damping_squared(double w, double w0, double lambda_max)
{
  if (w0 <= 0.0 || w >= w0) {
    return 0.0;
  }
  const double r = w / w0;
  return lambda_max * lambda_max * (1.0 - r * r);
}

/// qd = J^T (J J^T + lambda^2 I)^-1 v. With lambda^2 = 0 and a singular J the solve is
/// meaningless; callers schedule lambda so that it is not zero there.
inline Vector6 damped_least_squares(
  const Matrix6 & jacobian, const Vector6 & twist, double lambda_sq)
{
  Matrix6 a = jacobian * jacobian.transpose();
  a.diagonal().array() += lambda_sq;
  return jacobian.transpose() * a.ldlt().solve(twist);
}

/// Largest s in [0, 1] with |s x_j| <= limit_j for every j (limits <= 0 are ignored). Scaling the
/// whole vector keeps its direction, i.e. the TCP keeps moving the way it was told, only slower.
inline double uniform_scale(const Vector6 & x, const Vector6 & limit)
{
  double s = 1.0;
  for (int j = 0; j < 6; ++j) {
    if (limit[j] > 0.0 && std::abs(x[j]) > limit[j]) {
      s = std::min(s, limit[j] / std::abs(x[j]));
    }
  }
  return s;
}

/// Largest s in [0, 1] such that q + s * step stays inside [lower + margin, upper - margin] for
/// every joint that moves towards a limit; 0 if a joint is already past its margin and moving
/// further out. Joints moving away from a limit never restrict the step.
inline double limit_scale(
  const Vector6 & q, const Vector6 & step, const Vector6 & lower, const Vector6 & upper,
  double margin)
{
  double s = 1.0;
  for (int j = 0; j < 6; ++j) {
    if (step[j] > 0.0 && std::isfinite(upper[j])) {
      s = std::min(s, std::max(0.0, (upper[j] - margin - q[j]) / step[j]));
    } else if (step[j] < 0.0 && std::isfinite(lower[j])) {
      s = std::min(s, std::max(0.0, (lower[j] + margin - q[j]) / step[j]));
    }
  }
  return s;
}

}  // namespace ur5e_teaching_controllers::control
