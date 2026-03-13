function serial_hil_send(port, gps, imu, compass, ultrasonic, lidar, detections)
% SERIAL_HIL_SEND  Send sensor data packet to ESP32 over UART HIL link
%
% Packet format (binary, little-endian):
%   [0xAA, 0xBB]                Header (2 bytes)
%   [timestamp_ms uint32]       4 bytes
%   [lat_1e7 int32]            4 bytes  (lat_deg × 1e7)
%   [lon_1e7 int32]            4 bytes  (lon_deg × 1e7)
%   [ax int16][ay int16][az int16]   6 bytes  (raw MPU6050 accel)
%   [gx int16][gy int16][gz int16]   6 bytes  (raw MPU6050 gyro)
%   [compass_hdg uint16]        2 bytes  (heading_deg × 100)
%   [front uint16][left uint16][right uint16]  6 bytes (ultrasonic ×10 cm)
%   [n_det uint8]               1 byte   (number of detections)
%   [det₁: u16 u16 u8 int32 int32]   each detection: bbox_x,bbox_y,conf,lat_1e7,lon_1e7
%   [CRC16 uint16]              2 bytes
%
% Total (0 detections): 2+4+4+4+6+6+2+6+1+2 = 37 bytes
% Per detection: 2+2+1+4+4 = 13 bytes

HEADER = [0xAA, 0xBB];

timestamp_ms = uint32(round(gps.lat_deg * 0));   % placeholder; real: millis()

lat_1e7  = int32(round(gps.lat_deg * 1e7));
lon_1e7  = int32(round(gps.lon_deg * 1e7));

ax = imu.raw_accel_x;  ay = imu.raw_accel_y;  az = imu.raw_accel_z;
gx = imu.raw_gyro_x;   gy = imu.raw_gyro_y;   gz = imu.raw_gyro_z;

compass_hdg = uint16(round(compass.heading_deg * 100));

front_raw = uint16(min(ultrasonic.front_cm * 10, 9999));
left_raw  = uint16(min(ultrasonic.left_cm  * 10, 9999));
right_raw = uint16(min(ultrasonic.right_cm * 10, 9999));

n_det = uint8(numel(detections));

% Build byte array
data = [typecast(HEADER(1),'uint8'), typecast(HEADER(2),'uint8'), ...
        typecast(timestamp_ms,'uint8'), ...
        typecast(lat_1e7,'uint8'), typecast(lon_1e7,'uint8'), ...
        typecast(ax,'uint8'), typecast(ay,'uint8'), typecast(az,'uint8'), ...
        typecast(gx,'uint8'), typecast(gy,'uint8'), typecast(gz,'uint8'), ...
        typecast(compass_hdg,'uint8'), ...
        typecast(front_raw,'uint8'), typecast(left_raw,'uint8'), typecast(right_raw,'uint8'), ...
        n_det];

for k = 1:numel(detections)
    det = detections(k);
    bu  = uint16(det.bbox_xywh(1));
    bv  = uint16(det.bbox_xywh(2));
    cf  = uint8(round(det.confidence * 100));
    wlat= int32(round(det.world_ENU(2)*1e3));  % store as millimet ENU
    wlon= int32(round(det.world_ENU(1)*1e3));
    data = [data, typecast(bu,'uint8'), typecast(bv,'uint8'), cf, ...
                  typecast(wlat,'uint8'), typecast(wlon,'uint8')]; %#ok<AGROW>
end

% CRC16 (CCITT)
crc = crc16_ccitt(data);
data = [data, typecast(uint16(crc),'uint8')];

fwrite(port, data, 'uint8');
end

function pkt = serial_hil_recv(port)
% SERIAL_HIL_RECV  Read motor/state feedback packet from ESP32
%
% RX packet format:
%   [0xCC, 0xDD] Header
%   [fsm_state uint8]     0=IDLE,1=LAWNMOWER,2=ATTACK,3=RETURN
%   [wp_idx uint16]
%   [rpm_L uint16]   (RPM / 10)
%   [rpm_R uint16]
%   [crc uint16]

pkt = [];
if port.BytesAvailable < 11, return; end

raw = fread(port, 11, 'uint8')';
if raw(1) ~= 0xCC || raw(2) ~= 0xDD, return; end

pkt.fsm_state = raw(3);
pkt.wp_idx    = typecast(uint8(raw(4:5)),'uint16');
pkt.RPM_L     = double(typecast(uint8(raw(6:7)),'uint16')) * 10;
pkt.RPM_R     = double(typecast(uint8(raw(8:9)),'uint16')) * 10;
end

function port = serial_hil_init(port_name, baud)
% SERIAL_HIL_INIT  Open serial port for HIL
port = serial(port_name, 'BaudRate', baud, 'DataBits', 8, ...
              'Parity','none', 'StopBits',1, 'InputBufferSize', 4096);
fopen(port);
pause(2);   % allow ESP32 to reset after DTR
fprintf('[HIL] Serial port %s opened at %d baud\n', port_name, baud);
end

function crc = crc16_ccitt(data)
% CRC16-CCITT / XModem
crc = uint16(0);
poly = uint16(0x1021);
for b = data
    crc = bitxor(crc, uint16(bitshift(uint16(b), 8)));
    for i = 1:8
        if bitand(crc, uint16(0x8000))
            crc = bitxor(bitshift(crc,1), poly);
        else
            crc = bitshift(crc,1);
        end
    end
end
end
