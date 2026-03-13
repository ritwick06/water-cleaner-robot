/*
 * sensors.h — Hardware sensor driver manager for AWSCR ESP32
 *
 * Manages:
 *   - MPU6050 IMU via I2C
 *   - QMC5883L Compass via I2C
 *   - NEO-6M GPS via UART2
 *   - 3× HC-SR04 Ultrasonic via GPIO
 *
 * When in HIL mode, this class is bypassed;
 * data comes via hil_serial.h instead.
 */
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "fsm.h"   // for SensorData / Detection types

// Sensor data bundle
struct SensorData {
    // IMU
    float imu_ax, imu_ay, imu_az;   // [m/s²] body frame
    float imu_gx, imu_gy, imu_gz;   // [rad/s]
    // GPS
    float gps_lat, gps_lon, gps_alt;
    float gps_speed;
    bool  gps_valid;
    // Compass
    float compass_hdg_deg;
    // Ultrasonic
    float us_front_cm, us_left_cm, us_right_cm;
    // Camera detections
    Detection detections[4];
    int       n_detections;
};

// ── MPU6050 Register Addresses ────────────────────────────────────────────
#define MPU6050_ADDR    0x68
#define MPU_PWR_MGMT_1  0x6B
#define MPU_ACCEL_XOUTH 0x3B
#define MPU_GYRO_XOUTH  0x43

// ── QMC5883L Register Addresses ───────────────────────────────────────────
#define QMC_ADDR  0x0D
#define QMC_DATA  0x00
#define QMC_CR1   0x09
#define QMC_CR2   0x0A
#define QMC_PERI  0x0B
#define QMC_SR    0x06

class SensorManager {
public:
    void init(int tr_f, int ec_f, int tr_l, int ec_l, int tr_r, int ec_r) {
        // US pins
        _tr[0]=tr_f; _ec[0]=ec_f;
        _tr[1]=tr_l; _ec[1]=ec_l;
        _tr[2]=tr_r; _ec[2]=ec_r;
        for (int i=0;i<3;i++) {
            pinMode(_tr[i], OUTPUT);
            pinMode(_ec[i], INPUT);
        }

        // MPU6050 init
        _i2c_write(MPU6050_ADDR, MPU_PWR_MGMT_1, 0x00);  // wake up
        delay(100);
        Serial.println("[SENSOR] MPU6050 initialized");

        // QMC5883L init
        _i2c_write(QMC_ADDR, QMC_CR2,  0x01);  // soft reset
        delay(10);
        _i2c_write(QMC_ADDR, QMC_PERI, 0x01);  // period
        _i2c_write(QMC_ADDR, QMC_CR1,  0x0D);  // 200Hz, 8G, 512 OSR, continuous
        Serial.println("[SENSOR] QMC5883L initialized");

        // GPS UART2
        Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX=16, TX=17
        Serial.println("[SENSOR] NEO-6M GPS on UART2 RX:16 TX:17");
    }

    void read(SensorData& sd) {
        _readIMU(sd);
        _readCompass(sd);
        _readGPS(sd);
        _readUltrasonic(sd);
        sd.n_detections = 0;  // camera handled separately
    }

    float getFront() { return _last_front; }
    float getLeft()  { return _last_left; }
    float getRight() { return _last_right; }

private:
    int _tr[3], _ec[3];
    float _last_front=400, _last_left=400, _last_right=400;
    String _gps_buf;

    void _readIMU(SensorData& sd) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(MPU_ACCEL_XOUTH);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDR, 14, true);  // accel(6) + temp(2) + gyro(6)

        int16_t ax = Wire.read()<<8 | Wire.read();
        int16_t ay = Wire.read()<<8 | Wire.read();
        int16_t az = Wire.read()<<8 | Wire.read();
        Wire.read(); Wire.read();                   // skip temp
        int16_t gx = Wire.read()<<8 | Wire.read();
        int16_t gy = Wire.read()<<8 | Wire.read();
        int16_t gz = Wire.read()<<8 | Wire.read();

        const float ACC_SCALE = 9.81f / 16384.0f;   // ±2g default
        const float GYR_SCALE = (250.0f/32768.0f) * 3.14159265f/180.0f;

        sd.imu_ax = ax * ACC_SCALE;
        sd.imu_ay = ay * ACC_SCALE;
        sd.imu_az = az * ACC_SCALE;
        sd.imu_gx = gx * GYR_SCALE;
        sd.imu_gy = gy * GYR_SCALE;
        sd.imu_gz = gz * GYR_SCALE;
    }

    void _readCompass(SensorData& sd) {
        Wire.beginTransmission(QMC_ADDR);
        Wire.write(QMC_DATA);
        Wire.endTransmission(false);
        Wire.requestFrom(QMC_ADDR, 6, true);

        int16_t mx = Wire.read() | Wire.read()<<8;
        int16_t my = Wire.read() | Wire.read()<<8;
        /*int16_t mz =*/ Wire.read() | Wire.read()<<8;

        // Heading from X and Y (flat mount)
        float hdg = atan2f(-(float)my, (float)mx) * 180.0f / 3.14159265f;
        if (hdg < 0) hdg += 360.0f;
        // Add magnetic declination: -0.5° for Bangalore area
        hdg -= 0.5f;
        if (hdg < 0) hdg += 360.0f;
        sd.compass_hdg_deg = hdg;
    }

    void _readGPS(SensorData& sd) {
        sd.gps_valid = false;
        while (Serial2.available()) {
            char c = (char)Serial2.read();
            _gps_buf += c;
            if (c == '\n') {
                if (_gps_buf.startsWith("$GPGGA")) {
                    _parseGPGGA(_gps_buf, sd);
                }
                _gps_buf = "";
            }
        }
    }

    void _parseGPGGA(const String& s, SensorData& sd) {
        // $GPGGA,HHMMSS.ss,DDMM.MMMM,N,DDDMM.MMMM,E,Q,NN,D.D,ALT,M,...
        int fields[15]; int fi=0;
        fields[0]=0;
        for (int i=1; i<(int)s.length() && fi<14; i++) {
            if (s[i]==',') fields[++fi]=i+1;
        }
        if (fi < 6) return;

        float lat_raw = s.substring(fields[2], fields[3]-1).toFloat();
        char  lat_hem = s[fields[3]];
        float lon_raw = s.substring(fields[4], fields[5]-1).toFloat();
        char  lon_hem = s[fields[5]];
        int   fix_q   = s.substring(fields[6], fields[7]-1).toInt();

        if (fix_q == 0) return;

        // DDMM.MMMM → decimal degrees
        int lat_d = (int)(lat_raw / 100);
        float lat_m = lat_raw - lat_d*100;
        sd.gps_lat = lat_d + lat_m/60.0f;
        if (lat_hem == 'S') sd.gps_lat = -sd.gps_lat;

        int lon_d = (int)(lon_raw / 100);
        float lon_m = lon_raw - lon_d*100;
        sd.gps_lon = lon_d + lon_m/60.0f;
        if (lon_hem == 'W') sd.gps_lon = -sd.gps_lon;

        sd.gps_alt   = 0.0f;
        sd.gps_speed = 0.0f;
        sd.gps_valid = true;
    }

    void _readUltrasonic(SensorData& sd) {
        sd.us_front_cm = _pingCM(0);
        sd.us_left_cm  = _pingCM(1);
        sd.us_right_cm = _pingCM(2);
        _last_front = sd.us_front_cm;
        _last_left  = sd.us_left_cm;
        _last_right = sd.us_right_cm;
    }

    float _pingCM(int idx) {
        digitalWrite(_tr[idx], LOW);  delayMicroseconds(2);
        digitalWrite(_tr[idx], HIGH); delayMicroseconds(10);
        digitalWrite(_tr[idx], LOW);
        long dur = pulseIn(_ec[idx], HIGH, 25000);  // 25ms timeout=430cm
        if (dur == 0) return 400.0f;
        return dur * 0.01715f;   // speed_of_sound/2 = 17150 cm/s → cm
    }

    void _i2c_write(uint8_t addr, uint8_t reg, uint8_t val) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }
};
