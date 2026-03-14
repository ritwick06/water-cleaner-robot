#pragma once

#include "types.hpp"

namespace awscr {

// Physical parameters for the 3-DOF catamaran model.
struct VesselParams {
  double m;    // [kg] total mass
  double x_G;  // [m] CoG x-offset in body frame
  double I_z;  // [kg m^2] yaw inertia

  // Linear damping coefficients (surge, sway, yaw).
  double X_u;
  double Y_v;
  double N_r;

  // Quadratic damping coefficients.
  double X_uu;
  double Y_vv;
  double N_rr;

  // Added mass derivatives (negative values).
  double X_du;
  double Y_dv;
  double N_dr;

  double thruster_y;  // [m] lateral offset of each thruster from centerline
};

// State for the continuous-time 3-DOF model.
struct State3Dof {
  double x;    // [m] ENU East
  double y;    // [m] ENU North
  double psi;  // [rad] heading

  double u;  // [m/s] surge
  double v;  // [m/s] sway
  double r;  // [rad/s] yaw rate
};

// Input to the 3-DOF model.
struct Input3Dof {
  double T_left;   // [N] left thruster force
  double T_right;  // [N] right thruster force
  double u_c;      // [m/s] current in ENU x (east)
  double v_c;      // [m/s] current in ENU y (north)

  // External wave-induced generalized forces.
  double tau_wave_X;  // [N]
  double tau_wave_Y;  // [N]
  double tau_wave_N;  // [Nm]
};

// Continuous-time 3-DOF marine dynamics (Fossen-style).
class Dynamics3Dof {
 public:
  explicit Dynamics3Dof(const VesselParams& p);

  // Compute state derivative xdot = f(x, u).
  State3Dof derivative(const State3Dof& x, const Input3Dof& in) const;

 private:
  VesselParams p_;
};

}  // namespace awscr

