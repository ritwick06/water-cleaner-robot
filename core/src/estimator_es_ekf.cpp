#include "awscr/estimator_es_ekf.hpp"

#include <cmath>
#include <cstring>

namespace awscr {

namespace {

constexpr double kReEarth = 6378137.0;

inline void wgs84ToEnu(double lat_rad, double lon_rad,
                       double lat0_rad, double lon0_rad,
                       double& x_e, double& y_n) {
  const double d_lat = lat_rad - lat0_rad;
  const double d_lon = lon_rad - lon0_rad;
  const double c_lat0 = std::cos(lat0_rad);
  x_e = d_lon * c_lat0 * kReEarth;
  y_n = d_lat * kReEarth;
}

inline double wrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

}  // namespace

EsEkfEstimator::EsEkfEstimator() {
  mat_eye(P_, 1.0, N);
}

void EsEkfEstimator::set_reference_origin(double lat0_deg, double lon0_deg,
                                          double alt0_m) {
  lat0_rad_ = lat0_deg * M_PI / 180.0;
  lon0_rad_ = lon0_deg * M_PI / 180.0;
  alt0_m_ = alt0_m;
}

void EsEkfEstimator::predict(const ImuSample& imu, double dt_s) {
  // Very compact constant-velocity prediction with yaw integration.
  const double wz = imu.ang_vel_b[2] - bg_z_;
  state_.psi_rad = wrapAngle(state_.psi_rad + wz * dt_s);

  // Assume forward acceleration approximately lin_acc_b.x (ignoring gravity).
  const double ax_b = imu.lin_acc_b[0];
  state_.u_mps += ax_b * dt_s;

  const double c = std::cos(state_.psi_rad);
  const double s = std::sin(state_.psi_rad);
  const double vx = c * state_.u_mps;
  const double vy = s * state_.u_mps;

  state_.x_m += vx * dt_s;
  state_.y_m += vy * dt_s;
  state_.r_rps = wz;
  state_.t = imu.t;

  // Inflate covariance slightly to reflect motion uncertainty.
  for (int i = 0; i < N; ++i) {
    P_[i * N + i] += (i < 2 ? q_pos_ : (i < 6 ? q_vel_ : q_bg_)) * dt_s;
  }
}

void EsEkfEstimator::update_gps(const GpsFix& gps) {
  if (!gps.valid) return;
  double x_e = 0.0, y_n = 0.0;
  const double lat = gps.lat_deg * M_PI / 180.0;
  const double lon = gps.lon_deg * M_PI / 180.0;
  wgs84ToEnu(lat, lon, lat0_rad_, lon0_rad_, x_e, y_n);

  // Observation: z = [x; y]
  const double z[2] = {x_e, y_n};
  const double h[2] = {state_.x_m, state_.y_m};
  const double y[2] = {z[0] - h[0], z[1] - h[1]};

  // H is 2xN: [1 0 0 0 0 0 0; 0 1 0 0 0 0 0]
  double S[4] = {0.0, 0.0, 0.0, 0.0};
  for (int i = 0; i < N; ++i) {
    S[0] += P_[0 * N + i];  // H0i = [1,0,...]
    S[3] += P_[1 * N + i];  // H1i = [0,1,...]
  }
  S[0] += r_gps_pos_;
  S[3] += r_gps_pos_;

  const double invS0 = 1.0 / S[0];
  const double invS3 = 1.0 / S[3];

  // K = P H^T S^{-1}
  double Kx[N]{};
  double Ky[N]{};
  for (int i = 0; i < N; ++i) {
    Kx[i] = P_[i * N + 0] * invS0;
    Ky[i] = P_[i * N + 1] * invS3;
  }

  // State update.
  state_.x_m += Kx[0] * y[0] + Ky[0] * y[1];
  state_.y_m += Kx[1] * y[0] + Ky[1] * y[1];

  // Covariance update (diagonal approximation to keep things simple).
  for (int i = 0; i < N; ++i) {
    P_[i * N + i] *= 0.9;
  }
}

void EsEkfEstimator::update_mag(const MagSample& mag) {
  // Extract heading from horizontal components.
  const double mx = mag.mag_b[0];
  const double my = mag.mag_b[1];
  const double psi_meas = std::atan2(my, mx);

  const double err = wrapAngle(psi_meas - state_.psi_rad);
  const double K = 0.05;  // simple scalar gain; in full ES-EKF this comes
                          // from covariance and R matrix.
  state_.psi_rad = wrapAngle(state_.psi_rad + K * err);
}

void EsEkfEstimator::mat_add(double* A, const double* B, int n) {
  for (int i = 0; i < n * n; ++i) {
    A[i] += B[i];
  }
}

void EsEkfEstimator::mat_eye(double* A, double v, int n) {
  std::memset(A, 0, sizeof(double) * n * n);
  for (int i = 0; i < n; ++i) {
    A[i * n + i] = v;
  }
}

}  // namespace awscr

