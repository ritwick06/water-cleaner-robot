#include "awscr/dynamics_3dof.hpp"

#include <cmath>

namespace awscr {

namespace {

// Rotation from body to ENU.
inline void bodyToEnu(double psi, double u, double v, double r,
                      double& xdot, double& ydot, double& psidot) {
  const double c = std::cos(psi);
  const double s = std::sin(psi);
  xdot = c * u - s * v;
  ydot = s * u + c * v;
  psidot = r;
}

}  // namespace

Dynamics3Dof::Dynamics3Dof(const VesselParams& p) : p_(p) {}

State3Dof Dynamics3Dof::derivative(const State3Dof& x,
                                   const Input3Dof& in) const {
  State3Dof xd{};

  // 1) Kinematics: eta_dot = R(psi) * nu.
  bodyToEnu(x.psi, x.u, x.v, x.r, xd.x, xd.y, xd.psi);

  // 2) Hydrodynamic forces.
  // Relative velocity wrt water (subtract current expressed in ENU).
  double xdot_rel{}, ydot_rel{}, psidot_rel{};
  bodyToEnu(x.psi, x.u, x.v, x.r, xdot_rel, ydot_rel, psidot_rel);
  xdot_rel -= in.u_c;
  ydot_rel -= in.v_c;
  // Transform back to body for damping. For small currents this
  // approximation keeps complexity low.
  const double c = std::cos(-x.psi);
  const double s = std::sin(-x.psi);
  const double u_r = c * xdot_rel - s * ydot_rel;
  const double v_r = s * xdot_rel + c * ydot_rel;
  const double r_r = x.r;  // yaw rate unaffected by uniform current

  // Linear + quadratic damping in surge, sway, yaw.
  const double X_damp =
      p_.X_u * u_r + p_.X_uu * std::fabs(u_r) * u_r;
  const double Y_damp =
      p_.Y_v * v_r + p_.Y_vv * std::fabs(v_r) * v_r;
  const double N_damp =
      p_.N_r * r_r + p_.N_rr * std::fabs(r_r) * r_r;

  // Thruster forces and moment.
  const double X_thr = in.T_left + in.T_right;
  const double Y_thr = 0.0;
  const double N_thr = (in.T_right - in.T_left) * p_.thruster_y;

  // Added mass contributions.
  const double m11 = p_.m - p_.X_du;
  const double m22 = p_.m - p_.Y_dv;
  const double m33 = p_.I_z - p_.N_dr;

  // Coriolis + centripetal (rigid-body + added mass, simplified for symmetry).
  const double m = p_.m;
  const double xG = p_.x_G;
  const double C_RB_13 = -m * (xG * x.r + x.v);
  const double C_RB_23 = m * x.u;
  const double C_A_13 = p_.Y_dv * x.v + p_.N_dr * x.r;
  const double C_A_23 = -p_.X_du * x.u;

  const double X_coriolis = (C_RB_13 + C_A_13) * x.r;
  const double Y_coriolis = (C_RB_23 + C_A_23) * x.r;
  const double N_coriolis = -(C_RB_13 + C_A_13) * x.u -
                            (C_RB_23 + C_A_23) * x.v;

  const double tau_X = X_thr + in.tau_wave_X - X_damp - X_coriolis;
  const double tau_Y = Y_thr + in.tau_wave_Y - Y_damp - Y_coriolis;
  const double tau_N = N_thr + in.tau_wave_N - N_damp - N_coriolis;

  // Solve M * nu_dot = tau → diagonal approximation.
  xd.u = tau_X / m11;
  xd.v = tau_Y / m22;
  xd.r = tau_N / m33;

  return xd;
}

}  // namespace awscr

