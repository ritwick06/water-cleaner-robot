function [detections, bot_frame_pts] = sim_trash_detection(true_state, trash_locations_ENU, params)
% SIM_TRASH_DETECTION  Simulates ESP32-CAM trash detection + pixel-to-world
%
%   Simulates a camera mounted on the robot looking at the water surface.
%   For each trash object in range and FOV, generates:
%     - Bounding box in image pixels
%     - Confidence score
%     - Estimated world position (pixel-to-world projection)
%
% Camera mounting:
%   - Resolution: 320 × 240 pixels (OV2640)
%   - HFOV: 66°, VFOV: 50°
%   - Mounting height: h = 0.30 m above water
%   - Depression angle: α = 25° from horizontal (looking slightly down-forward)
%
% Pixel-to-World Math:
%   θ_v = (v - H/2)/H × VFOV  (pixel row → vertical angle from optical axis)
%   θ_h = (u - W/2)/W × HFOV  (pixel col → horizontal angle)
%   D   = h / tan(α + θ_v)    (ground range)
%   X   = D × tan(θ_h)        (lateral offset)
%
% Inputs:
%   true_state         : [x,y,psi,u,v,r] USV state (ENU)
%   trash_locations_ENU: Nx2 [E,N] trash positions in ENU metres
%   params             : optional config struct
%
% Outputs:
%   detections         : struct array, each with fields:
%       .bbox_xywh     : [x_centre, y_centre, width, height] in pixels
%       .confidence    : float [0,1]
%       .class         : string ('plastic_bottle'|'bag'|'debris')
%       .world_ENU     : [E, N] estimated world position [m]
%       .dist_m        : estimated distance from bot [m]
%   bot_frame_pts      : Nx2 [E,N] ground truth trash in bot frame (debug)

% ── Camera Parameters ────────────────────────────────────────────────────
IMG_W  = 320;   IMG_H = 240;
HFOV   = deg2rad(66);    VFOV = deg2rad(50);
h_cam  = 0.30;           % camera height [m]
alpha  = deg2rad(25);    % depression angle
MAX_DET_RANGE  = 3.0;    % max detection range [m]
MIN_DET_RANGE  = 0.20;   % min detection range [m]
SIGMA_CONF     = 0.05;   % confidence noise

CLASSES = {'plastic_bottle','plastic_bag','debris','foam_cup'};

bot_x = true_state(1);
bot_y = true_state(2);
psi   = true_state(3);

detections   = struct([]);
bot_frame_pts = [];

n_trash = size(trash_locations_ENU, 1);
if n_trash == 0, return; end

det_idx = 0;
for k = 1:n_trash
    % ── Transform trash to bot body frame ─────────────────────────────
    dx = trash_locations_ENU(k,1) - bot_x;   % East diff
    dy = trash_locations_ENU(k,2) - bot_y;   % North diff

    % Rotate to body frame (bot heading psi, 0=North)
    % Body: X=forward(surge), Y=left(port)
    x_body =  dx*sin(psi) + dy*cos(psi);     % forward
    y_body = -dx*cos(psi) + dy*sin(psi);     % left

    dist_m = sqrt(dx^2 + dy^2);
    if dist_m < MIN_DET_RANGE || dist_m > MAX_DET_RANGE, continue; end

    % ── Check if in camera FOV ─────────────────────────────────────────
    % Horizontal angle from camera axis (forward)
    theta_h = atan2(y_body, x_body);   % +ve = port
    % Vertical angle from camera axis
    theta_v = atan2(h_cam, x_body) - alpha;

    if abs(theta_h) > HFOV/2, continue; end   % outside HFOV
    if theta_v < -VFOV/2 || theta_v > VFOV/4, continue; end

    % ── Project to pixel coordinates ──────────────────────────────────
    u_px = IMG_W/2 + (theta_h / (HFOV/2)) * (IMG_W/2);
    v_px = IMG_H/2 - (theta_v / (VFOV/2)) * (IMG_H/2);   % image Y flipped

    % Bounding box size ∝ 1/distance (apparent size)
    obj_size_m = 0.15;    % typical trash item ~15cm
    bbox_w = (obj_size_m / dist_m) * (IMG_W / (2*tan(HFOV/2)));
    bbox_h = bbox_w * 0.7;

    bbox_w = max(5, round(bbox_w * (1 + 0.1*randn())));
    bbox_h = max(4, round(bbox_h * (1 + 0.1*randn())));
    u_px   = u_px + 2*randn();
    v_px   = v_px + 2*randn();

    % ── Pixel-to-World back-projection ────────────────────────────────
    theta_h_est = (u_px - IMG_W/2) / (IMG_W/2) * (HFOV/2);
    theta_v_est = -(v_px - IMG_H/2) / (IMG_H/2) * (VFOV/2);

    D_est = h_cam / tan(alpha + theta_v_est);
    D_est = max(0.1, D_est);
    X_est = D_est * tan(theta_h_est);

    % Body → ENU
    dE_est = X_est * cos(psi) + D_est * sin(psi);
    dN_est = -X_est * sin(psi) + D_est * cos(psi);

    trash_E_est = bot_x + dE_est;
    trash_N_est = bot_y + dN_est;

    % ── Confidence score (higher when closer + front-on) ──────────────
    conf_base = 0.92 * exp(-0.2 * dist_m);
    conf_base = min(0.99, max(0.5, conf_base));
    confidence = conf_base + SIGMA_CONF * randn();
    confidence = max(0.4, min(0.99, confidence));

    % ── Build detection ───────────────────────────────────────────────
    det_idx = det_idx + 1;
    detections(det_idx).bbox_xywh = [round(u_px), round(v_px), bbox_w, bbox_h];
    detections(det_idx).confidence = confidence;
    detections(det_idx).class      = CLASSES{mod(k-1,length(CLASSES))+1};
    detections(det_idx).world_ENU  = [trash_E_est, trash_N_est];
    detections(det_idx).dist_m     = dist_m;
    detections(det_idx).true_ENU   = trash_locations_ENU(k,:);

    bot_frame_pts = [bot_frame_pts; x_body, y_body]; %#ok<AGROW>
end
end
