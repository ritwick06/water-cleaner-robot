#pragma once

#include <cmath>

#include "types.hpp"

namespace awscr {

// Simple error-state EKF skeleton for 2D pose + velocities and IMU/GPS/Mag.
// This implementation is intentionally compact and platform-agnostic. It is
// sufficient for SIL and can be refined with more detailed Jacobians later.
class EsEkfEstimator {
 public:
  EsEkfEstimator();

  void set_reference_origin(double lat0_deg, double lon0_deg, double alt0_m);

  // Prediction with IMU at IMU rate.
  void predict(const ImuSample& imu, double dt_s);

  // Asynchronous updates.
  void update_gps(const GpsFix& gps);
  void update_mag(const MagSample& mag);

  StateEstimate get_state() const { return state_; }

 private:
  // Nominal state (ENU + body velocities + biases).
  StateEstimate state_{};
  double bg_z_{0.0};   // gyro bias around z

  // Simple covariance (9x9) stored as flat array.
  static constexpr int N = 7;  // [x,y,psi,u,v,r,bg_z]
  double P_[N * N]{};

  // Reference origin WGS84 for ENU conversion.
  double lat0_rad_{0.0};
  double lon0_rad_{0.0};
  double alt0_m_{0.0};

  // Process/measurement noise (tunable).
  double q_pos_{0.01};
  double q_vel_{0.1};
  double q_psi_{0.01};
  double q_bg_{1e-6};

  double r_gps_pos_{1.0};
  double r_mag_psi_{5.0 * 3.14159265358979323846 / 180.0};

  static void mat_add(double* A, const double* B, int n);
  static void mat_eye(double* A, double v, int n);
};

}  // namespace awscr

