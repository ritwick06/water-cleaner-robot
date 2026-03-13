%% MAIN_SIMULATION.M  ─ AWSCR Hardware-in-the-Loop MATLAB Simulation
%
%  Autonomous Water Surface Cleaning Robot
%  Full mission simulation: IDLE → LAWNMOWER → ATTACK → RETURN → LAWNMOWER → IDLE
%
%  Usage:
%    >> main_simulation          % run with default test area
%    >> main_simulation('hil')   % run with ESP32 HIL serial link
%
%  Press Ctrl+C to stop at any time.
%  Results are saved to results/sim_YYYYMMDD_HHMMSS.mat
%
% -------------------------------------------------------------------------

function main_simulation(mode)
if nargin < 1, mode = 'standalone'; end
HIL_MODE = strcmpi(mode,'hil');

fprintf('\n╔══════════════════════════════════════════════════════╗\n');
fprintf('║   AWSCR HIL Simulation — Autonomous Surface Cleaner  ║\n');
fprintf('╚══════════════════════════════════════════════════════╝\n');
fprintf('Mode: %s\n\n', upper(mode));

%% ── CONFIGURATION ──────────────────────────────────────────────────────
SIM_DT      = 0.02;          % 50Hz simulation
SIM_T_MAX   = 600;           % max 10 minutes
SPEED_CRUISE = 0.6;          % m/s lawnmower speed
SPEED_ATTACK = 0.9;          % m/s attack approach speed
WP_RADIUS   = 0.4;           % waypoint acceptance radius [m]
TRASH_COLLECT_RADIUS = 0.3;  % how close to "collect" trash [m]
ATTACK_CONF_THRESH   = 0.75; % min detection confidence to trigger attack
SWEEP_WIDTH = 0.6;           % lawnmower row spacing [m]

% Operation area — a 15m × 10m rectangular pond (ENU metres)
% Users can modify this to match their demo scenario
POLYGON_ENU = [0,0; 15,0; 15,10; 0,10];

% Reference GPS origin (Bangalore lake, for realism)
REF_ORIGIN  = [13.0827, 77.5946];   % [lat_deg, lon_deg]

% Trash spawn — random positions inside polygon
rng(42);   % reproducible
N_TRASH = 5;
TRASH_ENU = rand(N_TRASH, 2) .* repmat([12, 8], N_TRASH, 1) + [1.5, 1];

%% ── FIGURE SETUP ─────────────────────────────────────────────────────────
fig = figure('Name','AWSCR Live Simulation','Color',[0.04,0.06,0.10], ...
             'Position',[50,50,1400,780],'NumberTitle','off');

tiledlayout(fig, 2, 2, 'TileSpacing','compact', 'Padding','compact');
ax_map     = nexttile([2,1]);   % large left panel: mission map
ax_cam     = nexttile;          % top-right: camera view
ax_sensors = nexttile;          % bottom-right: telemetry

colormap(ax_map,'cool');

%% ── BOUSTROPHEDON PATH PLANNING ─────────────────────────────────────────
% Convert polygon ENU to GPS for planner (it re-projects to ENU internally)
poly_GPS = zeros(size(POLYGON_ENU,1),2);
m_lat = 111320;
m_lon = 111320 * cos(deg2rad(REF_ORIGIN(1)));
poly_GPS(:,1) = REF_ORIGIN(1) + POLYGON_ENU(:,2) / m_lat;
poly_GPS(:,2) = REF_ORIGIN(2) + POLYGON_ENU(:,1) / m_lon;

[wps_ENU, wps_GPS, ~, n_rows] = boustrophedon(poly_GPS, REF_ORIGIN, SWEEP_WIDTH);
fprintf('[Mission] Polygon area: %.0f m² | Rows: %d | Waypoints: %d\n', ...
        polygon_area_local(POLYGON_ENU), n_rows, size(wps_ENU,1));

%% ── INITIAL STATE ────────────────────────────────────────────────────────
% Start at first waypoint
state = [wps_ENU(1,1), wps_ENU(1,2), pi/4, 0, 0, 0];   % [x,y,psi,u,v,r]

% EKF initial state [lat, lon, psi, vx, vy]
lat0 = REF_ORIGIN(1) + state(2)/m_lat;
lon0 = REF_ORIGIN(2) + state(1)/m_lon;
x_ekf = [lat0; lon0; state(3); 0; 0];
P_ekf = diag([1e-8, 1e-8, 0.1, 0.2, 0.2]);

% Sensor bias states
imu_bias.accel_bias = 0.01*randn(3,1);
imu_bias.gyro_bias  = deg2rad(0.5)*randn(3,1);

% Controller states
pid_state   = struct('integral',0,'e_prev',0);
curr_state  = [0.1; 0.05];   % initial water current [vx, vy] m/s

% HIL serial
if HIL_MODE
    hil_port = serial_hil_init('/dev/ttyUSB0', 115200);
end

%% ── FSM & MISSION STATE ─────────────────────────────────────────────────
FSM      = 'LAWNMOWER';
wp_idx   = 2;         % target next waypoint (1 = start)
attack_target_ENU = [];
rejoin_wp_idx     = wp_idx;
trash_ENU         = TRASH_ENU;
collected_ENU     = zeros(0,2);
path_hist         = state(1:2);

% Logging arrays
log_t       = zeros(ceil(SIM_T_MAX/SIM_DT),1);
log_state   = zeros(ceil(SIM_T_MAX/SIM_DT),6);
log_ekf     = zeros(ceil(SIM_T_MAX/SIM_DT),5);
log_fsm     = cell (ceil(SIM_T_MAX/SIM_DT),1);
log_battery = zeros(ceil(SIM_T_MAX/SIM_DT),1);

% Battery model: 3S LiPo 2200mAh, full=12.6V
battery_capacity_Wh = 11.1 * 2.2;   % 24.4Wh
battery_energy_Wh   = battery_capacity_Wh;
battery_V           = 12.6;

fprintf('[SIM] Starting mission... Press Ctrl+C to stop.\n\n');

%% ── MAIN SIMULATION LOOP ─────────────────────────────────────────────────
step = 0;
t    = 0;
tic;

while t < SIM_T_MAX && ishandle(fig)
    step = step + 1;
    t    = t + SIM_DT;

    %% 1. Sensor Simulation
    imu_meas  = sim_MPU6050(state, SIM_DT, imu_bias);
    imu_bias  = imu_meas.bias_state;

    gps_meas  = sim_NEO6M(state, t, REF_ORIGIN);
    cmp_meas  = sim_QMC5883L(state(3), -0.5);  % decl=-0.5° Bangalore
    ult_meas  = sim_ultrasonic(state, []);      % no extra obstacles
    lidar_meas= sim_lidar(state, POLYGON_ENU, []);

    %% 2. EKF Update
    [x_ekf, P_ekf] = ekf_estimator(x_ekf, P_ekf, imu_meas, gps_meas, cmp_meas, SIM_DT);

    % Convert EKF lat/lon → ENU for navigation
    ekf_x = (x_ekf(2) - REF_ORIGIN(2)) * m_lon;   % East
    ekf_y = (x_ekf(1) - REF_ORIGIN(1)) * m_lat;   % North
    ekf_psi = x_ekf(3);                            % heading

    %% 3. Trash Detection (at 5Hz)
    detections = struct([]);
    if mod(step, 10) == 0 && ~isempty(trash_ENU)
        detections = sim_trash_detection(state, trash_ENU, struct());
    end

    %% 4. FSM Logic
    switch FSM
        %-----------------------------------------------------------------
        case 'LAWNMOWER'
            % Check if reached current waypoint
            if wp_idx <= size(wps_ENU,1)
                dist_to_wp = norm(state(1:2) - wps_ENU(wp_idx,:));
                if dist_to_wp < WP_RADIUS
                    wp_idx = wp_idx + 1;
                    fprintf('  [WP] Reached waypoint %d/%d\n', wp_idx-1, size(wps_ENU,1));
                end
            end

            % Check if mission complete
            if wp_idx > size(wps_ENU,1)
                FSM = 'IDLE';
                fprintf('\n[MISSION COMPLETE] All waypoints visited!\n');
                break;
            end

            % Check for trash detection → ATTACK
            if ~isempty(detections)
                best_det = [];
                for d = 1:length(detections)
                    if detections(d).confidence >= ATTACK_CONF_THRESH
                        if isempty(best_det) || detections(d).confidence > best_det.confidence
                            best_det = detections(d);
                        end
                    end
                end
                if ~isempty(best_det)
                    attack_target_ENU = best_det.world_ENU;
                    rejoin_wp_idx = wp_idx;
                    FSM = 'ATTACK';
                    fprintf('\n  [ATTACK] Trash detected at [%.2f, %.2f] conf=%.0f%%\n', ...
                            attack_target_ENU(1), attack_target_ENU(2), best_det.confidence*100);
                end
            end

            target_ENU   = wps_ENU(wp_idx,:);
            target_speed = SPEED_CRUISE;

        %-----------------------------------------------------------------
        case 'ATTACK'
            if ~isempty(attack_target_ENU)
                dist_to_trash = norm(state(1:2) - attack_target_ENU);
                if dist_to_trash < TRASH_COLLECT_RADIUS
                    % Collect trash
                    for k = 1:size(trash_ENU,1)
                        if norm(trash_ENU(k,:) - attack_target_ENU) < 0.5
                            collected_ENU = [collected_ENU; trash_ENU(k,:)]; %#ok<AGROW>
                            trash_ENU(k,:) = [];
                            fprintf('  [COLLECT] Trash collected! Total: %d\n', size(collected_ENU,1));
                            break;
                        end
                    end
                    attack_target_ENU = [];
                    FSM = 'RETURN_TO_PATH';

                    % Find optimal re-join waypoint
                    [rejoin_wp_idx, cost] = rejoin_optimizer(state(1:2), wps_ENU, rejoin_wp_idx, 0.5);
                    wp_idx = rejoin_wp_idx;
                    fprintf('  [REJOIN] Re-joining at WP %d (cost=%.2fm)\n', rejoin_wp_idx, cost);
                end
                target_ENU   = attack_target_ENU;
                target_speed = SPEED_ATTACK;
            else
                FSM = 'RETURN_TO_PATH';
                target_ENU   = wps_ENU(min(wp_idx, size(wps_ENU,1)),:);
                target_speed = SPEED_CRUISE;
            end

        %-----------------------------------------------------------------
        case 'RETURN_TO_PATH'
            target_ENU = wps_ENU(min(wp_idx, size(wps_ENU,1)),:);
            target_speed = SPEED_CRUISE;
            if norm(state(1:2) - target_ENU) < WP_RADIUS
                FSM = 'LAWNMOWER';
                fprintf('  [REJOIN] Back on lawnmower path at WP %d\n', wp_idx);
            end

        %-----------------------------------------------------------------
        case 'IDLE'
            break;
    end

    %% 5. Heading Computation → PID → Motor Commands
    bot_pos = state(1:2);
    target_vec = target_ENU - bot_pos;
    bearing_enu = atan2(target_vec(1), target_vec(2));   % ENU: East=x,North=y
    e_heading   = bearing_enu - state(3);
    e_heading   = atan2(sin(e_heading), cos(e_heading));  % wrap

    % VFH+ obstacle avoidance
    [safe_heading, avoid_active] = obstacle_vfh(lidar_meas, e_heading, struct());

    if avoid_active
        e_heading = safe_heading;
    end

    [u_pid, pid_state] = pid_heading(e_heading, pid_state, struct('Kp',1.5,'Ki',0.3,'Kd',0.45), SIM_DT);

    [RPM_L, RPM_R, F_L, F_R] = motor_model(target_speed, u_pid, battery_V);

    %% 6. USV Dynamics
    tau = [F_L + F_R; 0; (F_R - F_L) * 0.25];   % wheelbase L/2 = 0.25m
    [F_dist, curr_state] = water_disturbance(t, curr_state, struct());
    state = usv_3dof(state, tau, SIM_DT, F_dist);

    % Clamp inside polygon (soft boundary)
    state(1) = max(0.1, min(14.9, state(1)));
    state(2) = max(0.1, min(9.9,  state(2)));

    %% 7. Battery Model
    P_motor = (F_L + F_R) * norm(state(4:5)) + 5;   % motor + electronics
    battery_energy_Wh = battery_energy_Wh - P_motor * SIM_DT / 3600;
    battery_V = 9.9 + (battery_energy_Wh / battery_capacity_Wh) * 2.7;
    battery_V = max(9.9, battery_V);

    %% 8. Path History
    path_hist = [path_hist; ekf_x, ekf_y]; %#ok<AGROW>

    %% 9. Logging
    log_t(step)       = t;
    log_state(step,:) = state;
    log_ekf(step,:)   = x_ekf';
    log_fsm{step}     = FSM;
    log_battery(step) = battery_V;

    %% 10. HIL Serial (if enabled)
    if HIL_MODE && exist('hil_port','var')
        serial_hil_send(hil_port, gps_meas, imu_meas, cmp_meas, ult_meas, lidar_meas, detections);
        motor_fb = serial_hil_recv(hil_port);
        if ~isempty(motor_fb)
            RPM_L = motor_fb.RPM_L;
            RPM_R = motor_fb.RPM_R;
        end
    end

    %% 11. Coverage computation
    cov_pct = 100 * min(1, (wp_idx-1) / size(wps_ENU,1));

    %% 12. Visualization (every 5 steps = 10Hz display)
    if mod(step, 5) == 0
        sim_data = struct( ...
            'state',       state, ...
            'x_ekf',       [ekf_x; ekf_y], ...
            'polygon_ENU', POLYGON_ENU, ...
            'waypoints',   wps_ENU, ...
            'wp_idx',      min(wp_idx, size(wps_ENU,1)), ...
            'path_hist',   path_hist, ...
            'trash_ENU',   trash_ENU, ...
            'collected',   collected_ENU, ...
            'detections',  detections, ...
            'fsm_state',   FSM, ...
            't',           t, ...
            'imu',         imu_meas, ...
            'ultrasonic',  ult_meas, ...
            'gps_lat',     gps_meas.lat_deg, ...
            'gps_lon',     gps_meas.lon_deg, ...
            'lidar',       lidar_meas, ...
            'RPM_L',       RPM_L, ...
            'RPM_R',       RPM_R, ...
            'battery_V',   battery_V, ...
            'coverage_pct',cov_pct ...
        );
        live_plot(ax_map, ax_cam, ax_sensors, sim_data);
    end
end

%% ── POST-MISSION SUMMARY ─────────────────────────────────────────────────
elapsed_real = toc;
fprintf('\n══════════════════════════════════════════\n');
fprintf('  MISSION SUMMARY\n');
fprintf('══════════════════════════════════════════\n');
fprintf('  Sim time     : %.1f s\n', t);
fprintf('  Real time    : %.1f s (%.1fx real-time)\n', elapsed_real, t/elapsed_real);
fprintf('  Coverage     : %.1f%%\n', cov_pct);
fprintf('  Trash found  : %d / %d\n', size(collected_ENU,1), N_TRASH);
fprintf('  Battery left : %.1f V (%.0f%%)\n', battery_V, (battery_V-9.9)/2.7*100);
fprintf('══════════════════════════════════════════\n\n');

% Save results
if ~exist('results','dir'), mkdir('results'); end
fname = sprintf('results/sim_%s.mat', datestr(now,'yyyymmdd_HHMMSS'));
log_t     = log_t(1:step);
log_state = log_state(1:step,:);
log_ekf   = log_ekf(1:step,:);
log_fsm   = log_fsm(1:step);
log_battery = log_battery(1:step);
save(fname,'log_t','log_state','log_ekf','log_fsm','log_battery',...
     'wps_ENU','collected_ENU','POLYGON_ENU','N_TRASH');
fprintf('[Saved] %s\n', fname);

if HIL_MODE && exist('hil_port','var')
    fclose(hil_port);
end
end

%% ── Local helpers ────────────────────────────────────────────────────────
function A = polygon_area_local(poly)
    n = size(poly,1);
    A = 0;
    for i=1:n
        j=mod(i,n)+1;
        A=A+poly(i,1)*poly(j,2)-poly(j,1)*poly(i,2);
    end
    A=abs(A)/2;
end
