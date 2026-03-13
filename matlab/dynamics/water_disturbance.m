function [F_dist, current_state] = water_disturbance(t, current_state, params)
% WATER_DISTURBANCE  Simulate wave forces and water current on USV
%
%   Outputs combined ENU-frame disturbance force [Fx; Fy] in Newtons.
%   Current follows a first-order Gauss-Markov process.
%   Waves modelled as superposition of sinusoids (JONSWAP-inspired).
%
% Inputs:
%   t             : current simulation time (s)
%   current_state : [v_cx; v_cy] current velocity (m/s) — state memory
%   params        : struct with fields:
%       .H_s     : significant wave height (m), default 0.05 (calm pond)
%       .T_p     : peak wave period (s),        default 3.0
%       .theta_w : dominant wave direction (rad), default 0 (from North)
%       .V_c     : mean current speed (m/s),    default 0.15
%       .theta_c : current direction (rad),     default pi/6
%       .mu_c    : current correlation time (s),default 30
%       .rho     : water density (kg/m³),      default 1000
%       .A_wl    : waterplane area (m²),       default 0.4×0.5=0.2
%
% Outputs:
%   F_dist        : [Fx_E; Fy_N] total disturbance force (ENU, N)
%   current_state : updated [v_cx; v_cy]

% ── Defaults ────────────────────────────────────────────────────────────
if nargin < 3 || isempty(params)
    params = struct();
end
H_s     = get_param(params, 'H_s',     0.05);
T_p     = get_param(params, 'T_p',     3.0);
theta_w = get_param(params, 'theta_w', 0.0);
V_c     = get_param(params, 'V_c',     0.15);
theta_c = get_param(params, 'theta_c', pi/6);
mu_c    = get_param(params, 'mu_c',    30.0);
rho     = get_param(params, 'rho',     1000);
A_wl    = get_param(params, 'A_wl',    0.20);

dt = 0.02;  % nominal stepsize (matches 50Hz sim)

% ── 1. Water Current (Gauss-Markov) ─────────────────────────────────────
% dv_c/dt = -v_c/mu_c + w   (w = white noise driving)
sigma_c = V_c / sqrt(2 * mu_c);   % steady-state std

w_c     = sigma_c * randn(2,1);
v_cx    = current_state(1) * (1 - dt/mu_c) + w_c(1)*sqrt(dt);
v_cy    = current_state(2) * (1 - dt/mu_c) + w_c(2)*sqrt(dt);

% Clamp to physically reasonable speed
speed   = sqrt(v_cx^2 + v_cy^2);
speed_max = 0.5;
if speed > speed_max
    v_cx = v_cx * speed_max / speed;
    v_cy = v_cy * speed_max / speed;
end

current_state = [v_cx; v_cy];

% Current drag force ∝ v² (skin friction simplified)
C_dc   = 0.8;            % drag coefficient
F_curr = 0.5 * rho * C_dc * A_wl * current_state .* abs(current_state);

% ── 2. Wave Force (JONSWAP multi-component) ──────────────────────────────
% Using 5 sinusoidal components around peak frequency
omega_p = 2*pi / T_p;
n_comp  = 5;
omega_k = omega_p * (0.7 + 0.15*(0:n_comp-1));  % frequencies near peak
A_k     = H_s/4 * ones(1,n_comp) / n_comp;       % amplitudes (simplified)
phi_k   = 2*pi * rand(1,n_comp);                  % random phases (persistent per run)

% Total wave elevation
eta_w = sum(A_k .* cos(omega_k * t + phi_k));

% Wave force (Froude-Krylov approximation)
% F_wave ≈ rho * g * A_wl * eta_w  (simplified vertical coupling → surge)
g     = 9.81;
F_wave_mag = rho * g * A_wl * eta_w * 0.05;  % scaled to body interaction

F_wave  = F_wave_mag * [cos(theta_w); sin(theta_w)];

% ── 3. Total Disturbance ────────────────────────────────────────────────
F_dist = F_curr + F_wave;

end

% ── Helper ──────────────────────────────────────────────────────────────
function val = get_param(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end
