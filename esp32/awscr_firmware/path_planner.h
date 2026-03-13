/*
 * path_planner.h — Boustrophedon path planner for ESP32 (C++)
 *
 * Computes lawnmower coverage waypoints from a GPS polygon received via MQTT.
 * Stores waypoints in ENU (local flat-earth) metres for navigation.
 *
 * Algorithm: identical to matlab/navigation/boustrophedon.m
 * but runs on-chip with fixed-point arithmetic where needed.
 */
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>

#define MAX_POLY_VERTS  20
#define MAX_WAYPOINTS   200

struct GPSPoint {
    float lat;
    float lon;
};

struct ENUPoint {
    float e;   // East [m]
    float n;   // North [m]
};

class PathPlanner {
public:
    PathPlanner() : _n_poly(0), _n_waypoints(0), _wp_idx(0) {}

    // Called from MQTT callback with polygon JSON array
    void setPolygon(JsonArray polygon) {
        _n_poly = 0;
        for (JsonObject pt : polygon) {
            if (_n_poly >= MAX_POLY_VERTS) break;
            _poly_gps[_n_poly].lat = pt["lat"].as<float>();
            _poly_gps[_n_poly].lon = pt["lon"].as<float>();
            _n_poly++;
        }
        // Reference origin = centroid
        _lat0 = 0; _lon0 = 0;
        for (int i=0;i<_n_poly;i++) { _lat0+=_poly_gps[i].lat; _lon0+=_poly_gps[i].lon; }
        _lat0/=_n_poly; _lon0/=_n_poly;

        _m_lat = 111320.0f;
        _m_lon = 111320.0f * cosf(_lat0 * 0.01745329f);

        // Convert polygon to ENU
        for (int i=0;i<_n_poly;i++) {
            _poly_enu[i].e = (_poly_gps[i].lon - _lon0) * _m_lon;
            _poly_enu[i].n = (_poly_gps[i].lat - _lat0) * _m_lat;
        }

        Serial.printf("[Path] Polygon set with %d vertices\n", _n_poly);
    }

    // Compute boustrophedon waypoints
    void computeBoustrophedon(float sweep_m = 0.6f) {
        _n_waypoints = 0;
        _wp_idx      = 0;
        if (_n_poly < 3) return;

        float n_min = _poly_enu[0].n, n_max = _poly_enu[0].n;
        for (int i=1;i<_n_poly;i++) {
            if (_poly_enu[i].n < n_min) n_min = _poly_enu[i].n;
            if (_poly_enu[i].n > n_max) n_max = _poly_enu[i].n;
        }

        int dir = 1;
        for (float y = n_min + sweep_m/2; y < n_max && _n_waypoints+2 < MAX_WAYPOINTS; y += sweep_m) {
            float xs[20]; int nx = 0;
            for (int i=0; i<_n_poly && nx<20; i++) {
                int j = (i+1)%_n_poly;
                float y1=_poly_enu[i].n, y2=_poly_enu[j].n;
                float x1=_poly_enu[i].e, x2=_poly_enu[j].e;
                if ((y1>y) != (y2>y)) {
                    xs[nx++] = x1 + (y-y1)*(x2-x1)/(y2-y1);
                }
            }
            if (nx < 2) continue;
            // Sort xs (bubble)
            for (int a=0;a<nx-1;a++) for (int b=a+1;b<nx;b++) if (xs[a]>xs[b]) { float t=xs[a];xs[a]=xs[b];xs[b]=t; }

            float xa = dir>0 ? xs[0]   : xs[nx-1];
            float xb = dir>0 ? xs[nx-1]: xs[0];
            _wps_enu[_n_waypoints++] = {xa, y};
            _wps_enu[_n_waypoints++] = {xb, y};
            dir = -dir;
        }
        Serial.printf("[Path] %d waypoints computed (sweep=%.2fm)\n", _n_waypoints, sweep_m);
    }

    // Returns [E,N] array pointer for FSM
    float (*getWaypoints())[2] {
        return _wps_flat;
    }

    // Flatten to raw array for FSM (rebuilds each call — call infrequently)
    void flattenWaypoints() {
        for (int i=0;i<_n_waypoints;i++) {
            _wps_flat[i][0] = _wps_enu[i].e;
            _wps_flat[i][1] = _wps_enu[i].n;
        }
    }

    int  waypointCount()  { return _n_waypoints; }
    int  currentWPIndex() { return _wp_idx; }
    void setWPIndex(int i){ _wp_idx = i; }
    void advanceWP()      { if (_wp_idx < _n_waypoints-1) _wp_idx++; }

    // Get reference origin for EKF ENU conversion
    float getLat0()  { return _lat0; }
    float getLon0()  { return _lon0; }
    float getMlat()  { return _m_lat; }
    float getMlon()  { return _m_lon; }

private:
    GPSPoint _poly_gps[MAX_POLY_VERTS];
    ENUPoint _poly_enu[MAX_POLY_VERTS];
    ENUPoint _wps_enu [MAX_WAYPOINTS];
    float    _wps_flat[MAX_WAYPOINTS][2];
    int      _n_poly, _n_waypoints, _wp_idx;
    float    _lat0, _lon0, _m_lat, _m_lon;
};
