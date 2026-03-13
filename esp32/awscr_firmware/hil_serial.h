/*
 * hil_serial.h — MATLAB ↔ ESP32 Hardware-in-the-Loop serial protocol
 *
 * MATLAB sends sensor data → ESP32 parses and feeds into EKF/FSM
 * ESP32 sends motor feedback → MATLAB updates dynamics simulation
 *
 * Packet format matches matlab/hil/serial_hil.m (see that file for full spec)
 */
#pragma once
#include <Arduino.h>
#include "fsm.h"   // for SensorData, Detection

#define HIL_HDR_TX1  0xAA
#define HIL_HDR_TX2  0xBB
#define HIL_HDR_RX1  0xCC
#define HIL_HDR_RX2  0xDD
#define HIL_MIN_PKT  37         // minimum packet bytes (0 detections)

class HILSerial {
public:
    HILSerial() : _port(nullptr), _pkt_ready(false) {}

    void init(HardwareSerial* port, uint32_t baud) {
        _port = port;
        // Already initialized in main, just store reference
        Serial.println("[HIL] Serial HIL initialized");
    }

    bool hasPacket() {
        if (!_port) return false;
        _fillBuffer();
        return _pkt_ready;
    }

    bool readPacket(SensorData& sd) {
        if (!_pkt_ready) return false;
        _pkt_ready = false;

        // Parse from _buf[]
        int i = 2;   // skip header

        uint32_t ts_ms = _read_u32(i); i+=4;   // timestamp (unused)
        (void)ts_ms;

        int32_t lat_1e7 = _read_i32(i); i+=4;
        int32_t lon_1e7 = _read_i32(i); i+=4;
        sd.gps_lat = lat_1e7 / 1e7f;
        sd.gps_lon = lon_1e7 / 1e7f;
        sd.gps_valid = true;

        int16_t ax = _read_i16(i); i+=2;
        int16_t ay = _read_i16(i); i+=2;
        int16_t az = _read_i16(i); i+=2;
        int16_t gx = _read_i16(i); i+=2;
        int16_t gy = _read_i16(i); i+=2;
        int16_t gz = _read_i16(i); i+=2;

        const float AS = 9.81f/16384.0f;
        const float GS = (250.0f/32768.0f)*3.14159265f/180.0f;
        sd.imu_ax = ax*AS; sd.imu_ay = ay*AS; sd.imu_az = az*AS;
        sd.imu_gx = gx*GS; sd.imu_gy = gy*GS; sd.imu_gz = gz*GS;

        uint16_t hdg_100 = _read_u16(i); i+=2;
        sd.compass_hdg_deg = hdg_100 / 100.0f;

        uint16_t f_raw = _read_u16(i); i+=2;
        uint16_t l_raw = _read_u16(i); i+=2;
        uint16_t r_raw = _read_u16(i); i+=2;
        sd.us_front_cm = f_raw / 10.0f;
        sd.us_left_cm  = l_raw / 10.0f;
        sd.us_right_cm = r_raw / 10.0f;

        uint8_t n_det = _buf[i++];
        sd.n_detections = min((int)n_det, 4);
        for (int d=0; d<sd.n_detections; d++) {
            uint16_t bu    = _read_u16(i); i+=2;
            uint16_t bv    = _read_u16(i); i+=2;
            uint8_t  cf    = _buf[i++];
            int32_t  we    = _read_i32(i); i+=4;
            int32_t  wn    = _read_i32(i); i+=4;
            sd.detections[d].world_e    = we / 1000.0f;
            sd.detections[d].world_n    = wn / 1000.0f;
            sd.detections[d].confidence = cf / 100.0f;
            snprintf(sd.detections[d].cls, 24, "trash");
            (void)bu; (void)bv;
        }
        return true;
    }

    void sendFeedback(int fsm_state, int wp_idx, int rpm_L, int rpm_R) {
        if (!_port) return;
        uint8_t pkt[11];
        pkt[0] = HIL_HDR_RX1;
        pkt[1] = HIL_HDR_RX2;
        pkt[2] = (uint8_t)fsm_state;
        pkt[3] = (uint8_t)(wp_idx & 0xFF);
        pkt[4] = (uint8_t)(wp_idx >> 8);
        uint16_t rL = (uint16_t)(rpm_L / 10);
        uint16_t rR = (uint16_t)(rpm_R / 10);
        pkt[5]=rL&0xFF; pkt[6]=rL>>8;
        pkt[7]=rR&0xFF; pkt[8]=rR>>8;
        uint16_t crc = _crc16(pkt, 9);
        pkt[9]=crc&0xFF; pkt[10]=crc>>8;
        _port->write(pkt, 11);
    }

private:
    HardwareSerial* _port;
    uint8_t _buf[256];
    int     _buf_len = 0;
    bool    _pkt_ready;

    void _fillBuffer() {
        while (_port->available()) {
            _buf[_buf_len++] = _port->read();
            if (_buf_len > 250) { _buf_len=0; }   // overflow reset

            // Check for header
            if (_buf_len >= 2 && _buf[0] == HIL_HDR_TX1 && _buf[1] == HIL_HDR_TX2) {
                // Have enough for minimum packet?
                if (_buf_len >= HIL_MIN_PKT) {
                    uint8_t n_det = _buf[HIL_MIN_PKT-3];   // n_det field
                    int expect = HIL_MIN_PKT + n_det*13;
                    if (_buf_len >= expect + 2) {   // +2 for CRC
                        _pkt_ready = true;
                        break;
                    }
                }
            } else if (_buf_len == 1 && _buf[0] != HIL_HDR_TX1) {
                _buf_len = 0;   // wrong first byte, discard
            }
        }
    }

    uint16_t _read_u16(int i) { return _buf[i] | ((uint16_t)_buf[i+1]<<8); }
    int16_t  _read_i16(int i) { return (int16_t)_read_u16(i); }
    uint32_t _read_u32(int i) { return _buf[i]|((uint32_t)_buf[i+1]<<8)|((uint32_t)_buf[i+2]<<16)|((uint32_t)_buf[i+3]<<24); }
    int32_t  _read_i32(int i) { return (int32_t)_read_u32(i); }

    uint16_t _crc16(uint8_t* d, int len) {
        uint16_t crc=0, poly=0x1021;
        for (int i=0;i<len;i++) {
            crc ^= (uint16_t)d[i]<<8;
            for (int j=0;j<8;j++)
                crc = (crc&0x8000) ? (crc<<1)^poly : crc<<1;
        }
        return crc;
    }
};
