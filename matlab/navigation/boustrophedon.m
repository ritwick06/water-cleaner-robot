function [waypoints_ENU, waypoints_GPS, sweep_width_m, n_rows] = boustrophedon(polygon_GPS, ref_origin, sweep_width_m)
% BOUSTROPHEDON  Coverage path planning using Boustrophedon Cell Decomposition
%
%   Generates a complete back-and-forth (lawnmower) coverage path for an
%   arbitrary convex/non-convex polygon in GPS coordinates.
%
% Algorithm:
%   1. Project polygon GPS → local ENU (metres)
%   2. Rotate to find minimum-area bounding rectangle (optimal sweep axis)
%   3. Generate horizontal scan lines spaced sweep_width_m apart
%   4. Intersect scan lines with polygon edges → entry/exit pairs
%   5. Alternate left-to-right / right-to-left for Boustrophedon pattern
%   6. Project waypoints back to GPS
%
% Inputs:
%   polygon_GPS   : Nx2 [lat_deg, lon_deg] vertices (do NOT repeat last=first)
%   ref_origin    : [lat0, lon0] reference for ENU projection
%   sweep_width_m : (optional) row spacing in metres, default 0.6
%
% Outputs:
%   waypoints_ENU : Mx2 [East, North] metres, local ENU frame
%   waypoints_GPS : Mx2 [lat_deg, lon_deg]
%   sweep_width_m : actual sweep width used
%   n_rows        : number of sweep rows

if nargin < 3 || isempty(sweep_width_m)
    sweep_width_m = 0.6;   % 0.5m robot + 10% overlap
end

lat0 = ref_origin(1);
lon0 = ref_origin(2);

% ── 1. GPS → ENU projection ───────────────────────────────────────────────
m_per_deg_lat = 111320;
m_per_deg_lon = 111320 * cos(deg2rad(lat0));

E = (polygon_GPS(:,2) - lon0) * m_per_deg_lon;   % East [m]
N = (polygon_GPS(:,1) - lat0) * m_per_deg_lat;   % North [m]
poly_ENU = [E, N];

% ── 2. Find optimal sweep angle (min bounding rectangle) ─────────────────
angle_opt = find_optimal_sweep_angle(poly_ENU);

% Rotate polygon to align sweep axis with Y (North-aligned after rotation)
R = [cos(angle_opt), -sin(angle_opt);
     sin(angle_opt),  cos(angle_opt)];
poly_rot = (R * poly_ENU')';         % rotate polygon

% ── 3. Generate sweep rows ────────────────────────────────────────────────
y_min = min(poly_rot(:,2));
y_max = max(poly_rot(:,2));
n_rows = ceil((y_max - y_min) / sweep_width_m);

waypoints_rot = [];
dir = 1;   % +1 = left-to-right, -1 = right-to-left

for row = 0 : n_rows-1
    y_sweep = y_min + (row + 0.5) * sweep_width_m;

    % Intersect horizontal line y=y_sweep with polygon edges
    x_intersections = polygon_scan_intersect(poly_rot, y_sweep);

    if length(x_intersections) < 2, continue; end
    x_intersections = sort(x_intersections);

    % Boustrophedon: take first and last intersection as entry/exit
    x_enter = x_intersections(1);
    x_exit  = x_intersections(end);

    if dir > 0
        wp_a = [x_enter, y_sweep];
        wp_b = [x_exit,  y_sweep];
    else
        wp_a = [x_exit,  y_sweep];
        wp_b = [x_enter, y_sweep];
    end

    waypoints_rot = [waypoints_rot; wp_a; wp_b]; %#ok<AGROW>
    dir = -dir;   % flip direction for next row
end

% ── 4. Rotate back to ENU ─────────────────────────────────────────────────
R_inv = R';
waypoints_ENU = (R_inv * waypoints_rot')';

% ── 5. ENU → GPS projection ───────────────────────────────────────────────
waypoints_GPS = zeros(size(waypoints_ENU,1), 2);
waypoints_GPS(:,1) = lat0 + waypoints_ENU(:,2) / m_per_deg_lat;   % lat
waypoints_GPS(:,2) = lon0 + waypoints_ENU(:,1) / m_per_deg_lon;   % lon

fprintf('[Boustrophedon] Area: %.1f m² | Rows: %d | Waypoints: %d | Sweep: %.2f m\n', ...
    polygon_area(poly_ENU), n_rows, size(waypoints_ENU,1), sweep_width_m);
end

% ── Helpers ───────────────────────────────────────────────────────────────

function angle = find_optimal_sweep_angle(poly)
    % Try sweep angles 0–179° and pick one minimising bounding-box area
    % (equivalent to minimum-area bounding rectangle orientation)
    angles = deg2rad(0:1:179);
    min_area = Inf;
    angle    = 0;
    for i = 1:length(angles)
        a    = angles(i);
        rot  = poly * [cos(a), sin(a); -sin(a), cos(a)];
        area = (max(rot(:,1))-min(rot(:,1))) * (max(rot(:,2))-min(rot(:,2)));
        if area < min_area
            min_area = area;
            angle    = a;
        end
    end
end

function xs = polygon_scan_intersect(poly, y)
    % Find x-coordinates where horizontal line y=y intersects polygon edges
    n  = size(poly,1);
    xs = [];
    for i = 1:n
        j  = mod(i, n) + 1;
        y1 = poly(i,2);  x1 = poly(i,1);
        y2 = poly(j,2);  x2 = poly(j,1);
        if (y1 > y) ~= (y2 > y)    % edge straddles y
            x_int = x1 + (y - y1) * (x2 - x1) / (y2 - y1);
            xs    = [xs, x_int]; %#ok<AGROW>
        end
    end
end

function A = polygon_area(poly)
    % Shoelace formula
    n = size(poly,1);
    A = 0;
    for i = 1:n
        j = mod(i, n) + 1;
        A = A + poly(i,1)*poly(j,2) - poly(j,1)*poly(i,2);
    end
    A = abs(A) / 2;
end
