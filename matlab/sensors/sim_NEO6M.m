function meas = sim_NEO6M(true_state, t, ref_origin)
% SIM_NEO6M  Simulates u-blox NEO-6M GPS receiver (NMEA GPGGA)
%
%   true_state  = [x_m, y_m, psi_rad, u_ms, v_ms, r_rads]' (ENU)
%   t           = simulation time (s)
%   ref_origin  = [lat0_deg, lon0_deg] reference WGS84 origin
%
%   meas = struct:
%       .lat_deg, .lon_deg     WGS84 position
%       .alt_m                 Altitude (fixed for water surface robot)
%       .speed_ms              Ground speed (m/s)
%       .course_deg            Course over ground (deg, 0=North)
%       .fix                   1=GPS fix, 0=no fix
%       .nmea_sentence         GPGGA string for HIL serial
%       .valid                 1 if new fix available (1Hz gate)
%
% NEO-6M Specs: CEP 2.5m, 18s cold start, 1Hz typical update rate

% ── Parameters ──────────────────────────────────────────────────────────
UPDATE_RATE_HZ = 1.0;           % GPS update rate
CEP = 2.5;                      % Circular Error Probable [m]
sigma_gps = CEP / 1.1774;       % sigma for Rayleigh → Gaussian approx
sigma_vel  = 0.05;              % speed noise [m/s]

% ── 1Hz gate ─────────────────────────────────────────────────────────────
sample_period = 1.0 / UPDATE_RATE_HZ;
meas.valid    = (mod(t, sample_period) < 0.02);   % true one tick per second

% ── True position in WGS84 ───────────────────────────────────────────────
lat0 = ref_origin(1) * pi/180;
lon0 = ref_origin(2) * pi/180;

x_m = true_state(1);   % East
y_m = true_state(2);   % North

% Back-project to WGS84
R_earth = 6378137.0;
dlat = y_m / R_earth;
dlon = x_m / (R_earth * cos(lat0));

lat_true_deg = ref_origin(1) + dlat * 180/pi;
lon_true_deg = ref_origin(2) + dlon * 180/pi;

% ── Add CEP-style noise (2D Gaussian) ────────────────────────────────────
noise_N = sigma_gps * randn() / 111320;          % deg
noise_E = sigma_gps * randn() / (111320*cos(lat0));

meas.lat_deg = lat_true_deg + noise_N;
meas.lon_deg = lon_true_deg + noise_E;
meas.alt_m   = 0.0 + 0.5*randn();      % water surface

% ── Speed and Course ─────────────────────────────────────────────────────
u   = true_state(4);
v   = true_state(5);
psi = true_state(3);

% Ground speed in ENU
v_E = u*cos(psi) - v*sin(psi);
v_N = u*sin(psi) + v*cos(psi);

speed = sqrt(v_E^2 + v_N^2);
meas.speed_ms   = speed + sigma_vel*randn();
meas.course_deg = mod(atan2d(v_E, v_N), 360);   % 0=North, CW

% ── NMEA GPGGA Sentence ──────────────────────────────────────────────────
meas.nmea_sentence = build_gpgga(meas.lat_deg, meas.lon_deg, meas.alt_m, t);

meas.fix = 1;   % always fix in simulation
end

% ── NMEA Builder ────────────────────────────────────────────────────────
function sentence = build_gpgga(lat_deg, lon_deg, alt_m, t)
    % Time field (fake UTC from sim time)
    hh = floor(mod(t, 86400)/3600);
    mm = floor(mod(t, 3600)/60);
    ss = mod(t, 60);
    time_str = sprintf('%02d%02d%05.2f', hh, mm, ss);

    % Latitude: DDMM.MMMM,N
    lat_abs = abs(lat_deg);
    lat_d   = floor(lat_abs);
    lat_m   = (lat_abs - lat_d) * 60;
    lat_hem = 'N';
    if lat_deg < 0, lat_hem = 'S'; end
    lat_str = sprintf('%02d%08.5f,%s', lat_d, lat_m, lat_hem);

    % Longitude: DDDMM.MMMM,E
    lon_abs = abs(lon_deg);
    lon_d   = floor(lon_abs);
    lon_m   = (lon_abs - lon_d) * 60;
    lon_hem = 'E';
    if lon_deg < 0, lon_hem = 'W'; end
    lon_str = sprintf('%03d%08.5f,%s', lon_d, lon_m, lon_hem);

    % Build GPGGA (no checksum for sim)
    sentence = sprintf('$GPGGA,%s,%s,%s,1,08,1.0,%.1f,M,0,M,,', ...
                       time_str, lat_str, lon_str, alt_m);
end
