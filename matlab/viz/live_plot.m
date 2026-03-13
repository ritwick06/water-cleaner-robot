function live_plot(ax_map, ax_cam, ax_sensors, sim_data)
% LIVE_PLOT  Real-time simulation visualization
%
%   Three-panel figure:
%     ax_map     : top-down mission map (polygon, path, bot, trash)
%     ax_cam     : simulated ESP32-CAM view with bounding boxes
%     ax_sensors : sensor telemetry strips (IMU, GPS, ultrasonic, battery)
%
% sim_data fields:
%   .state       [x,y,psi,u,v,r]               USV true state (ENU)
%   .x_ekf       [lat,lon,psi,vx,vy]            EKF estimate
%   .polygon_ENU [Nx2]                          operation polygon
%   .waypoints   [Mx2]                          lawnmower waypoints
%   .wp_idx      current waypoint index
%   .path_hist   [Kx2]                          ENU history of bot path
%   .trash_ENU   [Px2]                          uncollected trash
%   .collected   [Qx2]                          collected trash
%   .detections  struct array from sim_trash_detection
%   .fsm_state   string: IDLE|LAWNMOWER|ATTACK|RETURN_TO_PATH
%   .t           simulation time [s]
%   .imu         struct from sim_MPU6050
%   .ultrasonic  struct from sim_ultrasonic
%   .battery_V   float
%   .lidar       struct from sim_lidar
%   .coverage_pct float [0,100]

% ── Colors ───────────────────────────────────────────────────────────────
COL_PATH  = [0.2, 0.6, 1.0];
COL_BOT   = [0.1, 0.8, 0.3];
COL_TRASH = [1.0, 0.4, 0.1];
COL_DONE  = [0.6, 0.6, 0.6];
COL_EKF   = [1.0, 0.9, 0.0];
COL_ATCK  = [1.0, 0.2, 0.2];
COL_LIDAR = [0.8, 0.2, 0.8];

% ── Panel 1: Mission Map ─────────────────────────────────────────────────
cla(ax_map);
hold(ax_map,'on'); axis(ax_map,'equal'); grid(ax_map,'on');
ax_map.Color = [0.04, 0.08, 0.14];
ax_map.GridColor = [0.2, 0.3, 0.4];

% Fill polygon (water surface)
if ~isempty(sim_data.polygon_ENU)
    poly = [sim_data.polygon_ENU; sim_data.polygon_ENU(1,:)];
    fill(ax_map, poly(:,1), poly(:,2), [0.05,0.15,0.30], ...
         'EdgeColor',[0.3,0.7,1.0], 'LineWidth',1.5, 'FaceAlpha',0.6);
end

% LIDAR scan overlay
if isfield(sim_data,'lidar') && ~isempty(sim_data.lidar)
    pts = sim_data.lidar.points_ENU;
    valid = ~isnan(pts(:,1));
    if any(valid)
        scatter(ax_map, pts(valid,1), pts(valid,2), 3, COL_LIDAR, 'filled','MarkerFaceAlpha',0.4);
    end
end

% Boustrophedon path
if size(sim_data.waypoints,1) > 1
    wps = sim_data.waypoints;
    for i = 1:2:size(wps,1)-1
        plot(ax_map, wps(i:i+1,1), wps(i:i+1,2), '--', ...
             'Color',[COL_PATH,0.4], 'LineWidth',0.8);
    end
    % Completed waypoints
    if sim_data.wp_idx > 1
        visited = wps(1:min(sim_data.wp_idx,end),:);
        plot(ax_map, visited(:,1), visited(:,2), '-', ...
             'Color',[COL_PATH,0.9], 'LineWidth',2);
    end
    % Current target
    if sim_data.wp_idx <= size(wps,1)
        tar = wps(sim_data.wp_idx,:);
        plot(ax_map, tar(1), tar(2), 's', 'Color',COL_PATH, ...
             'MarkerSize',12, 'LineWidth',2);
    end
end

% EKF path history
if size(sim_data.path_hist,1) > 1
    plot(ax_map, sim_data.path_hist(:,1), sim_data.path_hist(:,2), ...
         '-', 'Color',COL_EKF, 'LineWidth',1.5);
end

% Trash objects
if ~isempty(sim_data.trash_ENU)
    scatter(ax_map, sim_data.trash_ENU(:,1), sim_data.trash_ENU(:,2), ...
            80, COL_TRASH, 'filled', 'Marker','d', 'LineWidth',1);
end
if ~isempty(sim_data.collected)
    scatter(ax_map, sim_data.collected(:,1), sim_data.collected(:,2), ...
            60, COL_DONE, 'x', 'LineWidth',2);
end

% Detections (camera estimate)
for d = 1:length(sim_data.detections)
    det = sim_data.detections(d);
    plot(ax_map, det.world_ENU(1), det.world_ENU(2), 'o', ...
         'Color',COL_ATCK, 'MarkerSize',14, 'LineWidth',2);
    text(ax_map, det.world_ENU(1)+0.1, det.world_ENU(2)+0.1, ...
         sprintf('%.0f%%',det.confidence*100),'Color',COL_ATCK,'FontSize',8);
end

% Robot body (triangle showing heading)
x   = sim_data.state(1);
y   = sim_data.state(2);
psi = sim_data.state(3);
r_sz = 0.25;
tri_x = x + r_sz*[sin(psi), sin(psi+2.6), sin(psi-2.6)];
tri_y = y + r_sz*[cos(psi), cos(psi+2.6), cos(psi-2.6)];
fill(ax_map, tri_x, tri_y, COL_BOT, 'EdgeColor','w', 'LineWidth',1.5);

% EKF estimated position
ref = sim_data.x_ekf;
% EKF in GPS — convert back to ENU for display if needed
% (assume we pass ENU directly for simplicity)
if length(ref) >= 2
    plot(ax_map, ref(1), ref(2), '+', 'Color',COL_EKF, 'MarkerSize',12, 'LineWidth',2);
end

% FSM state badge
fsm_colors = containers.Map({'IDLE','LAWNMOWER','ATTACK','RETURN_TO_PATH'}, ...
             {[0.5,0.5,0.5],[0.2,0.8,0.3],[1.0,0.2,0.2],[1.0,0.7,0.0]});
c_badge = [0.5,0.5,0.5];
if isKey(fsm_colors, sim_data.fsm_state), c_badge = fsm_colors(sim_data.fsm_state); end

title(ax_map, sprintf('t=%.1fs | Mode: %-14s | Cov: %.1f%% | Trash: %d/%d', ...
      sim_data.t, sim_data.fsm_state, sim_data.coverage_pct, ...
      size(sim_data.collected,1), size(sim_data.collected,1)+size(sim_data.trash_ENU,1)), ...
      'Color', c_badge, 'FontSize',10,'FontWeight','bold');
xlabel(ax_map,'East [m]'); ylabel(ax_map,'North [m]');

% Legend
legend(ax_map, {'Polygon','LIDAR','Path planned','Bot heading','EKF pos', ...
                'Trash','Collected','Camera detect'}, ...
       'TextColor','w', 'Color',[0.1,0.1,0.15], 'Location','northeast', 'FontSize',7);

hold(ax_map,'off');

% ── Panel 2: Simulated Camera View ──────────────────────────────────────
cla(ax_cam);
hold(ax_cam,'on');
ax_cam.Color = [0.03,0.07,0.12];
ax_cam.XLim  = [0 320]; ax_cam.YLim = [0 240];
axis(ax_cam,'ij');   % image coords

% Background grid (water texture effect)
for gx = 0:32:320
    plot(ax_cam,[gx gx],[0 240],'Color',[0.05,0.1,0.2],'LineWidth',0.5);
end
for gy = 0:24:240
    plot(ax_cam,[0 320],[gy gy],'Color',[0.05,0.1,0.2],'LineWidth',0.5);
end

% Draw detections
for d = 1:length(sim_data.detections)
    det = sim_data.detections(d);
    bb  = det.bbox_xywh;
    x1  = bb(1) - bb(3)/2;   y1 = bb(2) - bb(4)/2;

    % Bounding box
    rectangle(ax_cam,'Position',[x1,y1,bb(3),bb(4)],'EdgeColor',COL_ATCK,'LineWidth',2);

    % Label
    label = sprintf('%s %.0f%%', det.class, det.confidence*100);
    text(ax_cam, x1, max(0,y1-2), label, 'Color', COL_ATCK, ...
         'FontSize',8, 'Interpreter','none');
end

title(ax_cam,'ESP32-CAM View (320×240)','Color','w','FontSize',9);
xlabel(ax_cam,'pixels'); ylabel(ax_cam,'pixels');
hold(ax_cam,'off');

% ── Panel 3: Sensor Telemetry ─────────────────────────────────────────────
cla(ax_sensors); hold(ax_sensors,'on');
ax_sensors.Color = [0.04,0.08,0.14];
ax_sensors.YLim  = [0 5]; ax_sensors.XLim = [0 10];
ax_sensors.XAxis.Visible = 'off'; ax_sensors.YAxis.Visible = 'off';

imu  = sim_data.imu;
ult  = sim_data.ultrasonic;
batt = sim_data.battery_V;
batt_pct = max(0,min(100, (batt-9.9)/(12.6-9.9)*100));

lines = {
    sprintf('IMU   | Ax:%.2f  Ay:%.2f  Az:%.2f g | ωz: %+.3f rad/s', ...
            imu.accel_x/9.81, imu.accel_y/9.81, imu.accel_z/9.81, imu.gyro_z), ...
    sprintf('GPS   | Lat: %.6f°  Lon: %.6f° | Speed: %.2f m/s', ...
            sim_data.gps_lat, sim_data.gps_lon, norm(sim_data.state(4:5))), ...
    sprintf('CMPS  | Heading: %.1f°', rad2deg(sim_data.state(3))), ...
    sprintf('ULTRA | Front: %.0f cm  Left: %.0f cm  Right: %.0f cm', ...
            ult.front_cm, ult.left_cm, ult.right_cm), ...
    sprintf('MOTOR | RPM L: %.0f  RPM R: %.0f', sim_data.RPM_L, sim_data.RPM_R), ...
    sprintf('BATT  | %.2f V  (%.0f%%)', batt, batt_pct), ...
};

y_pos = 4.5;
for i = 1:length(lines)
    text(ax_sensors, 0.1, y_pos, lines{i}, 'Color',[0.85,0.9,1.0], ...
         'FontSize',8.5, 'FontName','Courier');
    y_pos = y_pos - 0.7;
end

title(ax_sensors,'Sensor Telemetry','Color','w','FontSize',9);
hold(ax_sensors,'off');

drawnow limitrate;
end
