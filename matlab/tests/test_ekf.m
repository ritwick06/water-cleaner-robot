%% TEST_EKF.M  Unit tests for EKF estimator
%
%  Run from matlab/ directory: >> run('tests/test_ekf.m')

addpath('..');
addpath('../navigation');
addpath('../sensors');

fprintf('\n=== test_ekf ===\n');

REF = [13.0827, 77.5946];

%% Test 1: GPS update converges
x_est = [REF(1)+0.001; REF(2)+0.001; pi/4; 0; 0];  % 100m off
P = diag([1e-4, 1e-4, 0.5, 1, 1]);

% Simulate GPS-only correction (100 updates at 1Hz)
for k=1:100
    imu = struct('accel_x',0,'accel_y',0,'accel_z',9.81, ...
                 'gyro_z',0,'bias_state',struct('accel_bias',[0;0;0],'gyro_bias',[0;0;0]));
    gps = struct('lat_deg',REF(1),'lon_deg',REF(2),'valid',true);
    cmp = struct('heading_deg',45);
    [x_est, P] = ekf_estimator(x_est, P, imu, gps, cmp, 1.0);
end

m_lat = 111320;
err_lat_m = abs(x_est(1) - REF(1)) * m_lat;
assert(err_lat_m < 3.0, sprintf('FAIL T1: EKF lat not converged, err=%.2f m', err_lat_m));
fprintf('  [PASS] T1 — EKF converges to GPS, lat error %.2f m < 3m\n', err_lat_m);

%% Test 2: Compass update converges to true heading
x_est2 = [REF(1); REF(2); 0; 0; 0];
P2 = diag([1e-8, 1e-8, 1.0, 0.1, 0.1]);

for k = 1:50
    imu2 = struct('accel_x',0,'accel_y',0,'accel_z',9.81, ...
                  'gyro_z',0,'bias_state',struct('accel_bias',[0;0;0],'gyro_bias',[0;0;0]));
    gps2 = struct('lat_deg',REF(1),'lon_deg',REF(2),'valid',false);
    cmp2 = struct('heading_deg',90);   % true heading = East = 90°
    [x_est2, P2] = ekf_estimator(x_est2, P2, imu2, gps2, cmp2, 0.02);
end

err_hdg = abs(x_est2(3) - pi/2);   % 90° in radians
assert(err_hdg < 0.2, sprintf('FAIL T2: heading not converged, err=%.3f rad', err_hdg));
fprintf('  [PASS] T2 — EKF heading converges, error %.3f rad\n', err_hdg);

%% Test 3: Velocity estimate from IMU
x_est3 = [REF(1); REF(2); 0; 0; 0];
P3 = diag([1e-8, 1e-8, 0.01, 1, 1]);

ax_true = 0.5;   % 0.5 m/s² surge (body = ENU forward because psi=0)
for k = 1:50     % 50 × 20ms = 1 second
    imu3 = struct('accel_x',ax_true,'accel_y',0,'accel_z',9.81, ...
                  'gyro_z',0,'bias_state',struct('accel_bias',[0;0;0],'gyro_bias',[0;0;0]));
    gps3 = struct('lat_deg',REF(1),'lon_deg',REF(2),'valid',false);
    cmp3 = struct('heading_deg',0);
    [x_est3, P3] = ekf_estimator(x_est3, P3, imu3, gps3, cmp3, 0.02);
end

% Expected vy = ax_true × 1s = 0.5 m/s (North, since psi=0)
v_expected = ax_true * 1.0;
assert(abs(x_est3(5) - v_expected) < 0.1, ...
    sprintf('FAIL T3: velocity estimate %.3f ≠ %.3f', x_est3(5), v_expected));
fprintf('  [PASS] T3 — velocity estimate %.3f m/s (expected %.3f)\n', x_est3(5), v_expected);

%% Test 4: Covariance is symmetric and positive definite
assert(all(abs(P(:) - P'(:)) < 1e-10), 'FAIL T4: P not symmetric');
assert(all(eig(P) > 0), 'FAIL T4: P not positive definite');
fprintf('  [PASS] T4 — covariance matrix symmetric and positive definite\n');

fprintf('\n=== test_ekf PASSED ===\n\n');
