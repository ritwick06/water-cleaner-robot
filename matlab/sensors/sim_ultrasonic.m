function meas = sim_ultrasonic(true_state, obstacles_ENU)
% SIM_ULTRASONIC  Simulates 3× HC-SR04 ultrasonic sensors
%
%   Sensors mounted on USV:
%     Sensor 1: Front  (0°   from heading)
%     Sensor 2: Left   (90°  from heading, port side)
%     Sensor 3: Right  (-90° from heading, starboard)
%
%   Beam width: ±15° cone
%   Range: 2 cm to 400 cm
%   Sample rate: 20 Hz
%
%   true_state      = [x,y,psi,u,v,r] (ENU metres)
%   obstacles_ENU   = Nx2 matrix of [x_m, y_m] obstacle points (or empty)
%
%   meas = struct:
%       .front_cm, .left_cm, .right_cm   [cm]  (9999 = no echo / max range)

% ── Sensor mounting angles (body frame, from heading) ────────────────────
sensor_angles_deg = [0, 90, -90];     % front, left, right
sensor_names      = {'front','left','right'};
MAX_RANGE_CM      = 400;
MIN_RANGE_CM      = 2;

sigma_range_cm    = 1.0;              % noise [cm]
sigma_beam_deg    = 15.0;            % half-cone angle

% ── USV position ─────────────────────────────────────────────────────────
x   = true_state(1);
y   = true_state(2);
psi = true_state(3);

results = struct('front_cm', 9999, 'left_cm', 9999, 'right_cm', 9999);

% ── Boundary walls (simulate pond edge) ──────────────────────────────────
% Add edges as obstacle line segments — simple: just use bounding polygon
% Here we inject boundary via obstacles_ENU
if isempty(obstacles_ENU)
    obstacles_ENU = zeros(0,2);
end

% Ray-cast for each sensor ────────────────────────────────────────────────
for s = 1:3
    % Sensor beam direction in ENU
    sensor_heading = psi + deg2rad(sensor_angles_deg(s));
    beam_dir = [cos(sensor_heading - pi/2); sin(sensor_heading - pi/2)];
    % Note: psi=0=North, ENU: E=x,N=y, so bearing to ENU:
    % heading 0→north: dir=[0,1]; 90→east: dir=[1,0]
    beam_dir = [sin(sensor_heading); cos(sensor_heading)];

    % Cast ray through all obstacles and polygon walls
    ray_origin = [x; y];
    min_dist_m = MAX_RANGE_CM / 100;

    for i = 1:size(obstacles_ENU,1)
        % Point obstacle: minimum distance from ray
        p  = obstacles_ENU(i,:)';
        op = p - ray_origin;
        t  = dot(op, beam_dir);
        if t > 0
            perp = norm(op - t*beam_dir);
            beam_half_width = t * tan(deg2rad(sigma_beam_deg));
            if perp < max(beam_half_width, 0.2)
                min_dist_m = min(min_dist_m, t);
            end
        end
    end

    range_cm = min_dist_m * 100;
    % Clamp to valid range
    if range_cm < MIN_RANGE_CM
        range_cm = MIN_RANGE_CM;
    elseif range_cm >= MAX_RANGE_CM
        range_cm = 9999;  % no echo
    end

    % Add noise only for valid readings
    if range_cm < 9999
        range_cm = range_cm + sigma_range_cm * randn();
        range_cm = max(MIN_RANGE_CM, range_cm);
    end

    % Store
    switch s
        case 1, results.front_cm = range_cm;
        case 2, results.left_cm  = range_cm;
        case 3, results.right_cm = range_cm;
    end
end

meas = results;

% Pack as uint16 for serial (0.1 cm resolution)
meas.raw_front = uint16(min(results.front_cm, 9999) * 10);
meas.raw_left  = uint16(min(results.left_cm,  9999) * 10);
meas.raw_right = uint16(min(results.right_cm, 9999) * 10);
end
