/*
 * ekf.h — 5-state Extended Kalman Filter for AWSCR localization
 *
 * State: x = [lat_deg, lon_deg, psi_rad, vx_ms, vy_ms]
 *
 * Predict: dead-reckoning via IMU at 50Hz
 * Update:  GPS at 1Hz, compass at 10Hz
 */
#pragma once
#include <math.h>

struct EKFState {
    float lat_deg;
    float lon_deg;
    float psi_rad;
    float vx;       // East velocity [m/s]
    float vy;       // North velocity [m/s]
    float cov_pos;  // position uncertainty [m] (diagonal approx)
};

class EKF {
public:
    void init() {
        _x[0] = 13.0827f;  // lat
        _x[1] = 77.5946f;  // lon
        _x[2] = 0.0f;      // psi
        _x[3] = 0.0f;      // vx
        _x[4] = 0.0f;      // vy
        for (int i=0;i<25;i++) _P[i]=0;
        _P[0]=1e-4f; _P[6]=1e-4f; _P[12]=0.1f; _P[18]=0.1f; _P[24]=0.1f;
        _initialized = false;
    }

    void predict(float ax_body, float ay_body, float gyro_z, float dt) {
        // Rotate acceleration body→ENU
        float psi = _x[2];
        float ax_e = ax_body * cosf(psi) - ay_body * sinf(psi);
        float ay_e = ax_body * sinf(psi) + ay_body * cosf(psi);

        _x[4] += ay_e * dt;   // North vel
        _x[3] += ax_e * dt;   // East vel

        // Clamp speed
        float spd = sqrtf(_x[3]*_x[3]+_x[4]*_x[4]);
        if (spd > 2.0f) { _x[3]*=2.0f/spd; _x[4]*=2.0f/spd; }

        float m_lat = 111320.0f;
        float m_lon = 111320.0f * cosf(_x[0]*0.01745329f);

        _x[0] += (_x[4] * dt) / m_lat;
        _x[1] += (_x[3] * dt) / m_lon;
        _x[2] += gyro_z * dt;
        _x[2]  = atan2f(sinf(_x[2]), cosf(_x[2]));   // wrap

        // Increase covariance
        _P[0]  += 1e-9f;   // lat/lat
        _P[6]  += 1e-9f;   // lon/lon
        _P[12] += 1e-4f;   // psi/psi
        _P[18] += 1e-3f;   // vx/vx
        _P[24] += 1e-3f;   // vy/vy
    }

    void updateGPS(float lat_deg, float lon_deg) {
        // Scalar updates for lat and lon independently
        float R_pos = (2.5f/111320.0f)*(2.5f/111320.0f);  // 2.5m CEP
        float K0 = _P[0] / (_P[0] + R_pos);
        float K1 = _P[6] / (_P[6] + R_pos);

        _x[0] += K0 * (lat_deg - _x[0]);
        _x[1] += K1 * (lon_deg - _x[1]);
        _P[0]  *= (1.0f - K0);
        _P[6]  *= (1.0f - K1);
        _initialized = true;
    }

    void updateCompass(float heading_deg) {
        float psi_meas = heading_deg * 0.01745329f;
        float R_psi    = 0.00765f;   // (5°)² in rad²
        float K        = _P[12] / (_P[12] + R_psi);
        float innov    = psi_meas - _x[2];
        innov = atan2f(sinf(innov), cosf(innov));   // wrap
        _x[2] += K * innov;
        _x[2]  = atan2f(sinf(_x[2]), cosf(_x[2]));
        _P[12] *= (1.0f - K);
    }

    EKFState getState() {
        EKFState s;
        s.lat_deg = _x[0];
        s.lon_deg = _x[1];
        s.psi_rad = _x[2];
        s.vx      = _x[3];
        s.vy      = _x[4];
        s.cov_pos = sqrtf(_P[0]) * 111320.0f;  // approx uncertainty in metres
        return s;
    }

    bool isInitialized() { return _initialized; }

private:
    float _x[5];     // state
    float _P[25];    // diagonal-only covariance (5×5 simple)
    bool  _initialized;
};
