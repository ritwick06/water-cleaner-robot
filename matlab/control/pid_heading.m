function [u_out, state] = pid_heading(e, state, params, dt)
% PID_HEADING  Discrete PID heading controller with anti-windup
%
%   Controls USV heading by computing differential motor speed command.
%
%   u_out = Kp*e + Ki*integral(e)*dt + Kd*(de/dt)
%
%   Anti-windup: if output saturates, stop integrating
%
% Inputs:
%   e      : heading error (rad, normalized to [-pi, pi])
%   state  : struct with .integral, .e_prev (persistent state)
%   params : struct with .Kp, .Ki, .Kd, .u_max (optional, defaults below)
%   dt     : timestep [s]
%
% Outputs:
%   u_out  : control output (normalized [-1, 1], maps to differential RPM)
%   state  : updated controller state

% ── Defaults (Ziegler-Nichols tuned for USV at 0.5 m/s) ─────────────────
Kp    = getf(params,'Kp', 1.5);
Ki    = getf(params,'Ki', 0.3);
Kd    = getf(params,'Kd', 0.45);
u_max = getf(params,'u_max', 1.0);   % normalized output saturation

% State initialization
if ~isfield(state,'integral'), state.integral = 0; end
if ~isfield(state,'e_prev'),   state.e_prev   = e; end

% Wrap error to [-pi, pi]
e = atan2(sin(e), cos(e));

% ── PID Terms ─────────────────────────────────────────────────────────────
P_term = Kp * e;
I_term = Ki * state.integral;
D_term = Kd * (e - state.e_prev) / dt;

u_raw  = P_term + I_term + D_term;

% ── Anti-windup: conditional integration ────────────────────────────────
if abs(u_raw) < u_max
    state.integral = state.integral + e * dt;
else
    % Clamping anti-windup: only integrate if it would reduce saturation
    if sign(e) ~= sign(u_raw)
        state.integral = state.integral + e * dt;
    end
end

% ── Output saturation ────────────────────────────────────────────────────
u_out = max(-u_max, min(u_max, u_raw));

state.e_prev = e;
end

function v = getf(s, f, d)
    if isfield(s,f), v = s.(f); else, v = d; end
end
