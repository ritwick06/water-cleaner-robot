function meas = sim_QMC5883L(true_psi_rad, declination_deg)
% SIM_QMC5883L  Simulates QMC5883L 3-axis digital compass
%
%   true_psi_rad    : true heading (radians, 0=North, CCW positive)
%   declination_deg : magnetic declination for location (deg)
%                     Chennai/Bangalore area: ~0.5° W → use -0.5
%
%   meas = struct:
%       .mag_x, .mag_y, .mag_z   [Gauss] (body frame)
%       .heading_deg              Compensated heading [0,360) deg
%       .raw_x, .raw_y, .raw_z   int16 ADC values (for HIL packet)
%
% QMC5883L Specs:
%   Range: ±8 Gauss, Resolution 1.5 mGauss/LSB, ODR up to 200Hz
%   Noise: ~2 mGauss RMS per axis

% ── Parameters ──────────────────────────────────────────────────────────
if nargin < 2, declination_deg = -0.5; end

EARTH_FIELD_H = 0.40;   % Horizontal component [Gauss] (South India ~0.39G)
EARTH_FIELD_Z = 0.30;   % Vertical component [Gauss]

sigma_mag = 0.002;      % Noise per axis [Gauss] (2 mGauss)

% ── Hard-iron Offsets (typical for mounted electronics) ──────────────────
hard_iron = [0.05; 0.03; 0.01];   % [Gauss] offset from PCB currents

% ── Soft-iron Distortion (scale + cross-axis coupling) ───────────────────
soft_iron = [1.02, 0.01, 0.00;
             0.01, 0.98, 0.00;
             0.00, 0.00, 1.00];

% ── True Magnetic Field in Body Frame ────────────────────────────────────
% Heading angle = true heading + magnetic declination
psi_mag = true_psi_rad - deg2rad(declination_deg);

% Body-frame horizontal components (NE projected)
Bx_body =  EARTH_FIELD_H * cos(psi_mag);   % forward
By_body = -EARTH_FIELD_H * sin(psi_mag);   % starboard
Bz_body =  EARTH_FIELD_Z;                  % down

B_true = [Bx_body; By_body; Bz_body];

% ── Apply Soft-iron + Hard-iron Distortion ───────────────────────────────
B_dist = soft_iron * B_true + hard_iron;

% ── Add Noise ────────────────────────────────────────────────────────────
B_meas = B_dist + sigma_mag * randn(3,1);

meas.mag_x = B_meas(1);
meas.mag_y = B_meas(2);
meas.mag_z = B_meas(3);

% ── Compute Heading (tilt-compensated — flat robot, simplified) ──────────
heading_rad = atan2(-B_meas(2), B_meas(1));   % standard atan2 formula
heading_rad = heading_rad + deg2rad(declination_deg);  % add declination
heading_deg = mod(rad2deg(heading_rad), 360);

meas.heading_deg = heading_deg;

% ── Raw ADC values (QMC5883L at ±8G range: 3000 LSB/G) ──────────────────
LSB_PER_G  = 3000;
meas.raw_x = int16(B_meas(1) * LSB_PER_G);
meas.raw_y = int16(B_meas(2) * LSB_PER_G);
meas.raw_z = int16(B_meas(3) * LSB_PER_G);
end
