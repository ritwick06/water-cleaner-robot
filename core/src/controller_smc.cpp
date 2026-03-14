#include "awscr/controller_smc.hpp"

#include <algorithm>
#include <cmath>

namespace awscr {

namespace {

inline double wrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

inline double sat(double s, double phi) {
  if (s > phi) return 1.0;
  if (s < -phi) return -1.0;
  return s / phi;
}

}  // namespace

SmcController::SmcController(const VesselParams& p) : p_(p) {}

MotorThrustCmd SmcController::compute(const StateEstimate& est,
                                      const TrajectoryPoint& ref,
                                      double dt_s) {
  MotorThrustCmd cmd{};
  cmd.t = ref.t;

  // Position errors in world, then rotate to body frame.
  const double ex_world = est.x_m - ref.x;
  const double ey_world = est.y_m - ref.y;
  const double c = std::cos(est.psi_rad);
  const double s = std::sin(est.psi_rad);
  const double ex_b = c * ex_world + s * ey_world;

  const double epsi = wrapAngle(est.psi_rad - ref.psi);
  const double eu = est.u_mps - ref.u;
  const double er = est.r_rps - ref.r;

  integral_ex_ += ex_b * dt_s;
  integral_epsi_ += epsi * dt_s;

  const double kpx = 1.5;
  const double kix = 0.1;
  const double kdx = 0.5;

  const double kp_psi = 2.0;
  const double ki_psi = 0.05;
  const double kd_psi = 0.3;

  const double s1 = kpx * ex_b + kix * integral_ex_ + eu;
  const double s2 = kp_psi * epsi + ki_psi * integral_epsi_ + er;

  const double phi1 = 0.2;
  const double phi2 = 0.2;

  const double us = -kdx * sat(s1, phi1);
  const double urs = -kd_psi * sat(s2, phi2);

  const double u_des = ref.u + us;
  const double r_des = ref.r + urs;

  const double Xu = (p_.X_u != 0.0) ? p_.X_u : -5.0;
  const double N_r = (p_.N_r != 0.0) ? p_.N_r : -1.0;

  const double T_sum = (p_.m - p_.X_du) * (u_des - est.u_mps) / dt_s -
                       Xu * est.u_mps;
  const double delta_T = (p_.I_z - p_.N_dr) * (r_des - est.r_rps) / dt_s -
                         N_r * est.r_rps;

  const double T_left = 0.5 * (T_sum - delta_T / p_.thruster_y);
  const double T_right = 0.5 * (T_sum + delta_T / p_.thruster_y);

  const double T_max = 100.0;
  cmd.thrust_left_N = std::clamp(T_left, -T_max, T_max);
  cmd.thrust_right_N = std::clamp(T_right, -T_max, T_max);

  return cmd;
}

}  // namespace awscr

