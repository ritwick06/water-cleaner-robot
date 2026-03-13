function [x_est, P] = ekf_estimator(x_est, P, imu, gps, compass, dt)
% EKF_ESTIMATOR  5-state Extended Kalman Filter for USV localization
%
%   State vector: x = [lat_deg, lon_deg, psi_rad, v_x_ms, v_y_ms]'
%
%   Predict: dead-reckoning using IMU (accel + gyro) at 50Hz
%   Update1: GPS position measurements at 1Hz (when valid)
%   Update2: Compass heading at 10Hz
%
%   Inputs:
%     x_est   : [5×1] previous state estimate
%     P       : [5×5] previous covariance
%     imu     : struct from sim_MPU6050 (.accel_x,y,z, .gyro_z)
%     gps     : struct from sim_NEO6M  (.lat_deg,.lon_deg,.valid)
%     compass : struct from sim_QMC5883L (.heading_deg)
%     dt      : timestep [s]
%
%   Outputs:
%     x_est   : [5×1] updated state estimate
%     P       : [5×5] updated covariance

% ── Constants ────────────────────────────────────────────────────────────
R_earth = 6378137.0;       % [m]
g       = 9.81;            % [m/s²]

% ── Extract State ────────────────────────────────────────────────────────
lat = x_est(1);            % [deg]
lon = x_est(2);            % [deg]
psi = x_est(3);            % [rad]
vx  = x_est(4);            % ENU East velocity [m/s]
vy  = x_est(5);            % ENU North velocity [m/s]

lat_rad = deg2rad(lat);
lon_rad = deg2rad(lon);

% ── Process Noise Q ──────────────────────────────────────────────────────
% Tuned for a slow USV (max 1.5 m/s)
q_pos = 1e-10;     % position std² [deg²]
q_psi = 0.005^2;   % heading std² [rad²] per step
q_vel = 0.05^2;    % velocity std² [(m/s)²] per step

Q = diag([q_pos, q_pos, q_psi, q_vel, q_vel]);

% ─────────────────────────────────────────────────────────────────────────
% STEP 1: PREDICT (dead-reckoning)
% ─────────────────────────────────────────────────────────────────────────

% Yaw rate from gyro Z (corrected for IMU axis orientation)
omega_z = imu.gyro_z;  % [rad/s]
psi_new = psi + omega_z * dt;
psi_new = atan2(sin(psi_new), cos(psi_new));   % wrap

% Body accelerations → ENU (subtract gravity component, simplified)
ax_body = imu.accel_x;
ay_body = imu.accel_y;

% Rotate body → ENU
% psi=0 → North, body X=surge, Y=sway
ax_enu = ax_body * cos(psi) - ay_body * sin(psi);
ay_enu = ax_body * sin(psi) + ay_body * cos(psi);

% Velocity update
vx_new = vx + ax_enu * dt;
vy_new = vy + ay_enu * dt;

% Simple drag: limit max speed to 1.5 m/s
speed = sqrt(vx_new^2 + vy_new^2);
if speed > 1.5
    vx_new = vx_new * 1.5 / speed;
    vy_new = vy_new * 1.5 / speed;
end

% Position update (metre → degree conversion)
m_per_deg_lat = 111320;
m_per_deg_lon = 111320 * cos(lat_rad);

lat_new = lat + (vy_new * dt) / m_per_deg_lat;
lon_new = lon + (vx_new * dt) / m_per_deg_lon;

x_pred = [lat_new; lon_new; psi_new; vx_new; vy_new];

% ── Jacobian F (state transition) ────────────────────────────────────────
F = eye(5);
F(1,4) = 0;                               % dlat/dvx small
F(1,5) = dt / m_per_deg_lat;              % dlat/dvy
F(2,4) = dt / m_per_deg_lon;              % dlon/dvx
F(2,5) = 0;
F(3,3) = 1;                              % dpsi/dpsi
% vel rows: F(4:5,3) — dv/dpsi coupling (small, neglect)

% Predict covariance
P_pred = F * P * F' + Q;

% ─────────────────────────────────────────────────────────────────────────
% STEP 2a: GPS UPDATE (1Hz)
% ─────────────────────────────────────────────────────────────────────────
x_upd = x_pred;
P_upd = P_pred;

if gps.valid
    % Measurement: [lat, lon]
    z_gps   = [gps.lat_deg; gps.lon_deg];
    H_gps   = [1 0 0 0 0; 0 1 0 0 0];

    sigma_gps_deg_lat = 2.5 / m_per_deg_lat;
    sigma_gps_deg_lon = 2.5 / m_per_deg_lon;
    R_gps = diag([sigma_gps_deg_lat^2, sigma_gps_deg_lon^2]);

    y_gps  = z_gps - H_gps * x_pred;
    S_gps  = H_gps * P_pred * H_gps' + R_gps;
    K_gps  = P_pred * H_gps' / S_gps;

    x_upd = x_pred + K_gps * y_gps;
    P_upd = (eye(5) - K_gps * H_gps) * P_pred;
end

% ─────────────────────────────────────────────────────────────────────────
% STEP 2b: COMPASS UPDATE (10Hz approximation — always available)
% ─────────────────────────────────────────────────────────────────────────
z_psi = deg2rad(compass.heading_deg);

% Wrap innovation to [-pi, pi]
H_psi  = [0 0 1 0 0];
y_psi  = z_psi - x_upd(3);
y_psi  = atan2(sin(y_psi), cos(y_psi));   % wrap

R_psi  = deg2rad(5)^2;    % 5° compass noise
S_psi  = H_psi * P_upd * H_psi' + R_psi;
K_psi  = P_upd * H_psi' / S_psi;

x_upd  = x_upd + K_psi * y_psi;
x_upd(3) = atan2(sin(x_upd(3)), cos(x_upd(3)));   % wrap heading
P_upd  = (eye(5) - K_psi * H_psi) * P_upd;

% ── Output ────────────────────────────────────────────────────────────────
x_est = x_upd;
P     = (P_upd + P_upd') / 2;   % enforce symmetry
end
