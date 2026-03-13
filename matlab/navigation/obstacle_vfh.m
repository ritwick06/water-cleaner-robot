function [desired_heading_rad, avoidance_active] = obstacle_vfh(lidar_meas, target_heading_rad, params)
% OBSTACLE_VFH  Vector Field Histogram Plus (VFH+) obstacle avoidance
%
%   Uses LIDAR scan to build a polar obstacle density histogram,
%   then selects the best heading that is (a) obstacle-free and
%   (b) closest to the desired target heading.
%
% VFH+ Key improvements over VFH:
%   - Safety margin threshold (accounting for robot width)
%   - Weighted candidate sector selection (cosine to target)
%   - Hysteresis for heading stability
%
% Inputs:
%   lidar_meas        : struct from sim_lidar (.ranges_m, .angles_deg)
%   target_heading_rad: desired heading from path planner (body frame: 0=fwd)
%   params            : optional struct
%       .d_safe_m     : minimum safe distance [m] (default 1.5m)
%       .robot_r_m    : robot radius for inflation [m] (default 0.35m)
%       .smoothing    : histogram smoothing window (default 5)
%
% Outputs:
%   desired_heading_rad : adjusted heading recommendation (body frame)
%   avoidance_active    : true if avoiding an obstacle

if nargin < 3, params = struct(); end
d_safe  = getf(params,'d_safe_m',  1.5);
r_robot = getf(params,'robot_r_m', 0.35);
smooth_w = getf(params,'smoothing', 5);

% ── Build Polar Obstacle Density (POD) histogram ─────────────────────────
N_SECTORS = 72;   % 5° per sector
pod = zeros(1, N_SECTORS);

ranges  = lidar_meas.ranges_m;
ang_deg = lidar_meas.angles_deg;

for i = 1:length(ranges)
    if isnan(ranges(i)), continue; end

    % Enlarge obstacle by robot radius (safety inflation)
    r = ranges(i) - r_robot;
    if r <= 0, r = 0.01; end

    % Obstacle density: inverse square of distance
    density = max(0, (d_safe - r))^2 / d_safe^2;

    % Which sector
    sec = mod(floor(ang_deg(i)/5), N_SECTORS) + 1;
    pod(sec) = max(pod(sec), density);
end

% ── Smooth histogram ─────────────────────────────────────────────────────
pod_smooth = movmean(pod, smooth_w, 'Endpoints','circular');

% ── Binary obstacle map: blocked sectors ─────────────────────────────────
THRESHOLD  = 0.1;
blocked    = pod_smooth > THRESHOLD;

% ── Find free sectors and score them ─────────────────────────────────────
target_sec = mod(round(rad2deg(target_heading_rad)/5), N_SECTORS) + 1;

best_score = -Inf;
best_sec   = target_sec;
avoidance_active = false;

for k = 1:N_SECTORS
    if blocked(k), continue; end

    % Angular distance from candidate to target sector
    delta = min(abs(k - target_sec), N_SECTORS - abs(k - target_sec));
    score = cos(delta * pi / (N_SECTORS/2));   % 1 at target, -1 opposite

    if score > best_score
        best_score = score;
        best_sec   = k;
    end
end

% If we didn't pick the target sector, avoidance is active
if best_sec ~= target_sec
    avoidance_active = true;
end

% Convert sector back to heading (body frame, radians)
desired_heading_rad = deg2rad((best_sec - 1) * 5);
end

function v = getf(s, f, d)
    if isfield(s,f), v = s.(f); else, v = d; end
end
