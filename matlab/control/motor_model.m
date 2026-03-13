function [RPM_L, RPM_R, thrust_L_N, thrust_R_N] = motor_model(v_base_ms, pid_out, battery_V)
% MOTOR_MODEL  A2212 1000KV BLDC motor + 30A ESC model for differential drive
%
%   A2212 1000KV @ 3S LiPo (11.1V nominal):
%     Max RPM: KV × V = 1000 × 11.1 = 11,100 RPM
%     Max static thrust (5-inch prop): ~700g ≈ 6.87 N per motor
%     Propeller diameter: 5 inch = 0.127 m (assuming)
%     No-load current: ~1A | Max current: ~22A per motor
%
%   ESC Input: PWM 50Hz, 1000μs–2000μs → 0–100% throttle
%
% Inputs:
%   v_base_ms  : desired forward speed [m/s] (maps to base RPM)
%   pid_out    : PID heading output [-1, 1] (differential steering)
%   battery_V  : actual battery voltage (LiPo 3S: 12.6V full, 9.9V cutoff)
%
% Outputs:
%   RPM_L, RPM_R         : left/right motor RPM
%   thrust_L_N, thrust_R_N: left/right thrust [N]

if nargin < 3, battery_V = 11.1; end

% ── Motor Parameters ─────────────────────────────────────────────────────
KV         = 1000;          % RPM/V
V_nominal  = 11.1;          % Nominal 3S LiPo [V]
MAX_RPM    = KV * V_nominal;% 11,100 RPM at nominal
PROP_D_m   = 0.127;         % 5-inch prop diameter [m]
MAX_THRUST_N = 6.87;        % at max RPM (700g = 6.87N)

% Efficiency factor (battery derating)
eta_batt   = battery_V / V_nominal;
eta_batt   = max(0, min(1, eta_batt));

% ── Speed → Base RPM Mapping ─────────────────────────────────────────────
% From kinematics: USV max speed ≈ 1.5 m/s, maps to ~70% throttle
% RPM_base = v_base / v_max × MAX_RPM × eta
v_max      = 1.5;           % [m/s] max forward speed
throttle   = min(v_base_ms / v_max, 1.0);
RPM_base   = throttle * MAX_RPM * eta_batt;

% ── Differential Steering ─────────────────────────────────────────────────
% pid_out > 0 → turn left → increase right RPM, decrease left
gain       = 0.4 * MAX_RPM;   % max differential contribution
RPM_L_raw  = RPM_base - pid_out * gain;
RPM_R_raw  = RPM_base + pid_out * gain;

% Clamp to [0, MAX_RPM]
RPM_L = max(0, min(MAX_RPM, RPM_L_raw));
RPM_R = max(0, min(MAX_RPM, RPM_R_raw));

% ── Thrust Model: T ∝ RPM² (actuator disk theory) ────────────────────────
% T = C_T × ρ × n² × D⁴
% Simplified: T = MAX_THRUST × (RPM/MAX_RPM)²
thrust_L_N = MAX_THRUST_N * (RPM_L / MAX_RPM)^2 * eta_batt;
thrust_R_N = MAX_THRUST_N * (RPM_R / MAX_RPM)^2 * eta_batt;

end
