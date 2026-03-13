function meas = sim_lidar(true_state, polygon_ENU, obstacles_ENU)
% SIM_LIDAR  Simulates RPLiDAR A1 360° 2D laser scanner
%
%   RPLiDAR A1 Specs:
%     Range: 0.15m – 12m (we cap at 8m for clarity)
%     Angular resolution: 1° (360 samples/scan)
%     Scan rate: 5.5 Hz (typical)
%     Range precision: ±1%  → ±5cm at 5m
%
%   true_state    = [x,y,psi,u,v,r] (ENU metres)
%   polygon_ENU   = Nx2 [x,y] polygon boundary vertices (closed)
%   obstacles_ENU = Mx2 [x,y] additional obstacle points (can be empty)
%
%   meas = struct:
%       .angles_deg   [1×360] angles in body frame (0°=forward)
%       .ranges_m     [1×360] range per angle in metres (NaN = no return)
%       .points_body  [360×2] XY hit points in body frame
%       .points_ENU   [360×2] XY hit points in ENU world frame

% ── Parameters ──────────────────────────────────────────────────────────
N_RAYS      = 360;
MAX_RANGE_M = 8.0;
MIN_RANGE_M = 0.15;
sigma_range = 0.03;     % [m] range noise (30mm)

x   = true_state(1);
y   = true_state(2);
psi = true_state(3);    % heading in ENU

angles_body_deg = linspace(0, 359, N_RAYS);   % body frame, 0=forward

% ── Build obstacle set (polygon edges as line segments) ──────────────────
% Convert polygon boundary to line segments
n_poly = size(polygon_ENU, 1);
segs   = zeros(0, 4);   % [x1, y1, x2, y2]
for i = 1:n_poly
    next = mod(i, n_poly) + 1;
    segs = [segs; polygon_ENU(i,:), polygon_ENU(next,:)]; %#ok<AGROW>
end

ranges      = nan(1, N_RAYS);
pts_body    = nan(N_RAYS, 2);
pts_enu     = nan(N_RAYS, 2);

for i = 1:N_RAYS
    % Body angle → ENU bearing
    alpha_body = deg2rad(angles_body_deg(i));
    % psi: 0=North in ENU.  body forward = psi direction
    bearing_enu = psi + alpha_body;   % ENU angle
    % ENU: x=East, y=North
    ray_dx = sin(bearing_enu);   % East component
    ray_dy = cos(bearing_enu);   % North component

    t_min = MAX_RANGE_M;

    % ── Intersect with polygon wall segments ──────────────────────────
    for k = 1:size(segs,1)
        x1 = segs(k,1) - x;   y1 = segs(k,2) - y;
        x2 = segs(k,3) - x;   y2 = segs(k,4) - y;
        t = ray_segment_intersect(ray_dx, ray_dy, x1, y1, x2, y2);
        if ~isnan(t) && t > MIN_RANGE_M && t < t_min
            t_min = t;
        end
    end

    % ── Intersect with point obstacles (circle r=0.15m) ───────────────
    for k = 1:size(obstacles_ENU,1)
        ox = obstacles_ENU(k,1) - x;
        oy = obstacles_ENU(k,2) - y;
        % Circle-ray: (ray_dir×oc)² = r²(1+ ...) solve quadratic
        a  = 1;
        b  = -2*(ray_dx*ox + ray_dy*oy);
        c  = ox^2 + oy^2 - 0.15^2;
        disc = b^2 - 4*a*c;
        if disc >= 0
            t1 = (-b - sqrt(disc)) / (2*a);
            if t1 > MIN_RANGE_M && t1 < t_min
                t_min = t1;
            end
        end
    end

    % ── Record result ─────────────────────────────────────────────────
    if t_min < MAX_RANGE_M
        r_noisy = t_min + sigma_range * randn();
        r_noisy = max(MIN_RANGE_M, r_noisy);
        ranges(i) = r_noisy;
        pts_body(i,:) = [r_noisy * sin(alpha_body), r_noisy * cos(alpha_body)];
        pts_enu(i,:)  = [x + r_noisy*ray_dx, y + r_noisy*ray_dy];
    end
end

meas.angles_deg  = angles_body_deg;
meas.ranges_m    = ranges;
meas.points_body = pts_body;
meas.points_ENU  = pts_enu;
end

% ── Ray-segment intersection (returns t along ray, NaN if no hit) ────────
function t = ray_segment_intersect(dx, dy, x1, y1, x2, y2)
    t  = NaN;
    sx = x2 - x1;
    sy = y2 - y1;
    denom = dx*sy - dy*sx;
    if abs(denom) < 1e-10, return; end
    t_ray = (x1*sy - y1*sx) / denom;
    t_seg = (x1*dy - y1*dx) / denom;
    if t_ray > 0 && t_seg >= 0 && t_seg <= 1
        t = t_ray;
    end
end
