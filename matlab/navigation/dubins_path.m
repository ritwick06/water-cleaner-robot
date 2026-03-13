function path = dubins_path(q_start, q_goal, R_min)
% DUBINS_PATH  Compute minimum-length Dubins path between two configurations
%
%   A Dubins path is the shortest path for a vehicle with a minimum turning
%   radius R_min, moving from q_start to q_goal.
%
%   The six path types are: LSL, LSR, RSL, RSR, RLR, LRL
%   (L=left turn, R=right turn, S=straight)
%
% Inputs:
%   q_start : [x, y, theta] start configuration (ENU metres, heading rad)
%   q_goal  : [x, y, theta] goal configuration
%   R_min   : minimum turning radius [m]. For ESP32 USV: v_max/omega_max
%             = 1.0/0.8 = 1.25m
%
% Outputs:
%   path = struct:
%       .type       : string e.g. 'LSL', 'RSR' etc.
%       .length     : total path length [m]
%       .segments   : [t, p, q] arc/segment lengths (in R_min units)
%       .waypoints  : Nx3 [x,y,theta] sampled path for plotting

if nargin < 3, R_min = 1.25; end

% Normalize to unit turning radius
dx = (q_goal(1) - q_start(1)) / R_min;
dy = (q_goal(2) - q_start(2)) / R_min;
D  = sqrt(dx^2 + dy^2);

alpha = mod(q_start(3) - atan2(dy, dx), 2*pi);
beta  = mod(q_goal(3)  - atan2(dy, dx), 2*pi);

% ── Compute all 4 standard path types ────────────────────────────────────
best_len = Inf;
best_type = '';
best_segs = [0 0 0];

types = {'LSL','RSR','LSR','RSL'};
for k = 1:4
    segs = dubins_compute(alpha, beta, D, types{k});
    if ~isempty(segs) && all(segs >= 0)
        L = sum(segs);
        if L < best_len
            best_len  = L;
            best_type = types{k};
            best_segs = segs;
        end
    end
end

% ── Sample path waypoints ─────────────────────────────────────────────────
path.type     = best_type;
path.length   = best_len * R_min;
path.segments = best_segs;
path.waypoints = sample_dubins(q_start, best_type, best_segs, R_min, 50);
end

% ── Dubins Segment Computation ────────────────────────────────────────────
function segs = dubins_compute(alpha, beta, d, type)
    sa = sin(alpha); ca = cos(alpha);
    sb = sin(beta);  cb = cos(beta);
    c_ab = cos(alpha - beta);
    segs = [];
    switch type
        case 'LSL'
            tmp0 = d + sa - sb;
            pq   = atan2(cb - ca, tmp0);
            p    = mod(-alpha + pq, 2*pi);
            q    = mod(beta  - pq, 2*pi);
            t    = mod(pq, 2*pi);
            if tmp0 >= 0
                segs = [t, sqrt(tmp0^2 + (cb-ca)^2), q];
            end
        case 'RSR'
            tmp0 = d - sa + sb;
            pq   = atan2(ca - cb, tmp0);
            p    = mod(alpha - pq, 2*pi);
            q    = mod(-beta + pq, 2*pi);
            if tmp0 >= 0
                segs = [p, sqrt(tmp0^2 + (ca-cb)^2), q];
            end
        case 'LSR'
            tmp  = -2 + d^2 + 2*c_ab + 2*d*(sa+sb);
            if tmp >= 0
                p2   = sqrt(tmp);
                pq   = atan2(-ca-cb, d+sa+sb) - atan2(-2, p2);
                t    = mod(-alpha + pq, 2*pi);
                q    = mod(-mod(beta,2*pi) + pq, 2*pi);
                segs = [t, p2, q];
            end
        case 'RSL'
            tmp  = d^2 - 2 + 2*c_ab - 2*d*(sa+sb);
            if tmp >= 0
                p2   = sqrt(tmp);
                pq   = atan2(ca+cb, d-sa-sb) - atan2(2, p2);
                t    = mod(alpha - pq, 2*pi);
                q    = mod(beta  - pq, 2*pi);
                segs = [t, p2, q];
            end
    end
end

% ── Sample along path ─────────────────────────────────────────────────────
function pts = sample_dubins(q0, type, segs, R, n_samples)
    pts = zeros(n_samples+1, 3);
    pts(1,:) = q0;
    q = q0;
    total_len = sum(segs) * R;

    seg_types = num2cell(type);   % e.g. {'L','S','R'}
    seg_lens  = segs * R;

    samples_per_seg = max(1, floor(n_samples .* seg_lens / total_len));
    samples_per_seg(3) = n_samples - sum(samples_per_seg(1:2));

    idx = 2;
    for s = 1:3
        ns  = samples_per_seg(s);
        L_s = seg_lens(s) / max(ns, 1);
        for k = 1:ns
            q = step_dubins(q, seg_types{s}, L_s, R);
            if idx <= n_samples+1
                pts(idx,:) = q;
                idx = idx + 1;
            end
        end
    end
    pts = pts(1:idx-1,:);
end

function q_new = step_dubins(q, seg_type, ds, R)
    x = q(1); y = q(2); th = q(3);
    switch seg_type
        case 'L'  % left turn
            th_new = th + ds/R;
            x_new  = x + R*(sin(th_new) - sin(th));
            y_new  = y + R*(-cos(th_new) + cos(th));
        case 'R'  % right turn
            th_new = th - ds/R;
            x_new  = x + R*(-sin(th_new) + sin(th));
            y_new  = y + R*(cos(th_new) - cos(th));
        case 'S'  % straight
            x_new  = x + ds * sin(th);
            y_new  = y + ds * cos(th);
            th_new = th;
    end
    q_new = [x_new, y_new, atan2(sin(th_new), cos(th_new))];
end
