#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace awscr {

// Generic IMU sample (gyroscope + accelerometer) in body frame.
struct ImuSample {
  double t;  // [s] monotonic time stamp
  std::array<double, 3> ang_vel_b;   // [rad/s] (gx, gy, gz)
  std::array<double, 3> lin_acc_b;   // [m/s^2] (ax, ay, az)
};

// Generic GPS fix in WGS84.
struct GpsFix {
  double t;       // [s]
  double lat_deg; // [deg]
  double lon_deg; // [deg]
  double alt_m;   // [m]
  bool valid{false};
};

// Generic 3D magnetometer sample in body frame.
struct MagSample {
  double t;  // [s]
  std::array<double, 3> mag_b;  // [uT]
};

// Simple 2D LiDAR scan (planar, 360 deg or subset).
struct LidarScan {
  double t;  // [s]
  double angle_min_rad;
  double angle_max_rad;
  double angle_step_rad;
  std::vector<double> ranges_m;  // [m]
};

// Filtered state estimate in local ENU and body frame.
struct StateEstimate {
  double t;      // [s]
  double x_m;    // [m] ENU East
  double y_m;    // [m] ENU North
  double psi_rad;  // [rad] yaw in ENU frame
  double u_mps;    // [m/s] surge
  double v_mps;    // [m/s] sway
  double r_rps;    // [rad/s] yaw rate
};

// Motor thrust command for twin-thruster catamaran.
struct MotorThrustCmd {
  double t;             // [s]
  double thrust_left_N;   // [N]
  double thrust_right_N;  // [N]
};

enum class FsmMode : std::uint8_t {
  IDLE = 0,
  PLANNING = 1,
  EXECUTING_COVERAGE = 2,
  EXECUTING_INTERCEPT = 3,
  DOCKING = 4,
  ERROR = 5
};

// Minimal telemetry packet for UI / logging.
struct Telemetry {
  double t;          // [s]
  double lat_deg;    // [deg]
  double lon_deg;    // [deg]
  double heading_deg;  // [deg]
  double speed_mps;    // [m/s]
  FsmMode mode;
};

}  // namespace awscr

