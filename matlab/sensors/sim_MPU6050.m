function meas = sim_MPU6050(true_state, dt, bias_state)
% SIM_MPU6050  Simulates MPU-6050 IMU (accelerometer + gyroscope)
%
%   true_state  = [x,y,psi,u,v,r] USV state from usv_3dof
%   dt          = timestep (s)
%   bias_state  = struct with fields .accel_bias [3x1], .gyro_bias [3x1]
%                 (persistent across calls, modelled as random walk)
%
%   meas = struct:
%       .accel_x, .accel_y, .accel_z  [m/s²]  (body frame, includes gravity)
%       .gyro_x,  .gyro_y,  .gyro_z   [rad/s] (body frame)
%       .temp                          [°C]
%
% MPU-6050 Datasheet specs:
%   Accelerometer: ±2g, 16-bit, noise density 400μg/√Hz
%   Gyroscope:     ±250°/s, 16-bit, noise density 0.005°/s/√Hz
%   ODR:           up to 1kHz (we use 50Hz)

% ── Noise Parameters ────────────────────────────────────────────────────
accel_noise_density = 400e-6 * 9.81;  % [m/s²/√Hz]
gyro_noise_density  = deg2rad(0.005); % [rad/s/√Hz]
sigma_accel = accel_noise_density * sqrt(1/dt);   % at sample rate 1/dt
sigma_gyro  = gyro_noise_density  * sqrt(1/dt);

% Bias random walk
sigma_accel_rw = 1e-4 * sqrt(dt);   % [m/s²/s^0.5]
sigma_gyro_rw  = deg2rad(1e-3) * sqrt(dt);

% ── Extract True Derivatives ─────────────────────────────────────────────
u   = true_state(4);  % surge velocity
v   = true_state(5);  % sway velocity
r   = true_state(6);  % yaw rate
psi = true_state(3);  % heading

% True accelerations (body frame) — centripetal + Coriolis
% Simplified: a_x = du/dt (approximated as zero for steady state demo)
% Include centripetal: a_y = u*r (from yaw rate)
a_x_true = -v * r;           % surge acceleration + centripetal coupling
a_y_true =  u * r;           % sway acceleration
g = 9.81;
a_z_true = g;                % gravity component (robot assumed near-flat)

% True angular rates (body frame)
omega_x_true = 0;    % roll rate (USV on water, minimal)
omega_y_true = 0;    % pitch rate
omega_z_true = r;    % yaw rate

% ── Bias Update (Random Walk) ─────────────────────────────────────────────
bias_state.accel_bias = bias_state.accel_bias + sigma_accel_rw * randn(3,1);
bias_state.gyro_bias  = bias_state.gyro_bias  + sigma_gyro_rw  * randn(3,1);

% ── Add Noise + Bias ─────────────────────────────────────────────────────
meas.accel_x = a_x_true + bias_state.accel_bias(1) + sigma_accel * randn();
meas.accel_y = a_y_true + bias_state.accel_bias(2) + sigma_accel * randn();
meas.accel_z = a_z_true + bias_state.accel_bias(3) + sigma_accel * randn();

meas.gyro_x  = omega_x_true + bias_state.gyro_bias(1) + sigma_gyro * randn();
meas.gyro_y  = omega_y_true + bias_state.gyro_bias(2) + sigma_gyro * randn();
meas.gyro_z  = omega_z_true + bias_state.gyro_bias(3) + sigma_gyro * randn();

% Temperature (constant ± small noise)
meas.temp    = 28.0 + 0.5*randn();

% Return updated bias state
meas.bias_state = bias_state;

% Pack raw for serial transmission (scaled to int16)
ACCEL_SCALE = 9.81 / 16384.0;  % MPU6050 ±2g default
GYRO_SCALE  = (250.0/32768.0) * pi/180;
meas.raw_accel_x = int16(meas.accel_x / ACCEL_SCALE);
meas.raw_accel_y = int16(meas.accel_y / ACCEL_SCALE);
meas.raw_accel_z = int16(meas.accel_z / ACCEL_SCALE);
meas.raw_gyro_x  = int16(meas.gyro_x  / GYRO_SCALE);
meas.raw_gyro_y  = int16(meas.gyro_y  / GYRO_SCALE);
meas.raw_gyro_z  = int16(meas.gyro_z  / GYRO_SCALE);
end
