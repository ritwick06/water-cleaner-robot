/*
 * fsm.h — Finite State Machine for AWSCR mission control
 *
 * States:
 *   FSM_IDLE            → waiting for mission polygon from MQTT
 *   FSM_LAWNMOWER       → following boustrophedon coverage path
 *   FSM_ATTACK          → diverting to collect detected trash
 *   FSM_RETURN_TO_PATH  → returning to lawnmower after collection
 */
#pragma once
#include <stdint.h>
#include "ekf.h"

// State enum
enum FSMState { FSM_IDLE=0, FSM_LAWNMOWER, FSM_ATTACK, FSM_RETURN_TO_PATH };

// Detection result (from HIL serial or camera inference)
struct Detection {
    float world_e;   // ENU East estimated position [m from origin]
    float world_n;   // ENU North [m]
    float confidence;
    char  cls[24];   // class label
};

// Input to FSM per tick
struct FSMInput {
    EKFState     est;
    float        (*waypoints)[2];   // Mx2 array of [E,N] ENU metres
    int          n_waypoints;
    Detection*   detections;
    int          n_detections;
    float        wp_radius_m;
    float        collect_r_m;
    bool         danger_flag;
    float        dt;
    // Reference origin
    float        lat0, lon0;        // for ENU↔GPS conversion on-chip
    float        m_lat, m_lon;
};

// Output from FSM per tick
struct FSMOutput {
    float target_heading_rad;  // desired heading for PID
    float speed_frac;          // [0,1] speed fraction for motor
    int   current_wp_idx;
};

class FSMController {
public:
    FSMController() : _state(FSM_IDLE), _wp_idx(0), _attack_target_set(false) {}

    void setState(FSMState s) { _state = s; }
    FSMState getState()       { return _state; }
    const char* getStateName() {
        switch(_state) {
            case FSM_IDLE:           return "IDLE";
            case FSM_LAWNMOWER:      return "LAWNMOWER";
            case FSM_ATTACK:         return "ATTACK";
            case FSM_RETURN_TO_PATH: return "RETURN_TO_PATH";
            default:                 return "UNKNOWN";
        }
    }

    void tick(const FSMInput& in) {
        // Convert EKF position to ENU for distance computations
        float bot_e = (in.est.lon_deg - in.lon0) * in.m_lon;
        float bot_n = (in.est.lat_deg - in.lat0) * in.m_lat;

        switch (_state) {
            // ── IDLE ─────────────────────────────────────────────
            case FSM_IDLE:
                _out.speed_frac          = 0.0f;
                _out.target_heading_rad  = in.est.psi_rad;
                break;

            // ── LAWNMOWER ─────────────────────────────────────────
            case FSM_LAWNMOWER:
                if (_wp_idx >= in.n_waypoints) {
                    _state = FSM_IDLE;
                    break;
                }
                // Waypoint reached?
                {
                float dE = in.waypoints[_wp_idx][0] - bot_e;
                float dN = in.waypoints[_wp_idx][1] - bot_n;
                if (sqrtf(dE*dE+dN*dN) < in.wp_radius_m) {
                    _wp_idx++;
                    if (_wp_idx >= in.n_waypoints) { _state = FSM_IDLE; break; }
                }
                // Steer to current waypoint
                dE = in.waypoints[_wp_idx][0] - bot_e;
                dN = in.waypoints[_wp_idx][1] - bot_n;
                _out.target_heading_rad = atan2f(dE, dN);
                _out.speed_frac         = 0.6f;
                }
                // Check trash detection → ATTACK
                if (!in.danger_flag) {
                    for (int d=0; d<in.n_detections; d++) {
                        if (in.detections[d].confidence >= 0.75f) {
                            _attack_e   = in.detections[d].world_e;
                            _attack_n   = in.detections[d].world_n;
                            _rejoin_idx = _wp_idx;
                            _state      = FSM_ATTACK;
                            break;
                        }
                    }
                }
                break;

            // ── ATTACK ────────────────────────────────────────────
            case FSM_ATTACK:
                {
                float dE  = _attack_e - bot_e;
                float dN  = _attack_n - bot_n;
                float dist= sqrtf(dE*dE+dN*dN);
                if (dist < in.collect_r_m) {
                    // Collected
                    _state = FSM_RETURN_TO_PATH;
                    // Simple re-join: find nearest remaining WP
                    _wp_idx = _rejoin_optimizer(bot_e, bot_n, in.waypoints,
                                                 in.n_waypoints, _rejoin_idx);
                    break;
                }
                _out.target_heading_rad = atan2f(dE, dN);
                _out.speed_frac         = 0.85f;
                }
                break;

            // ── RETURN_TO_PATH ─────────────────────────────────────
            case FSM_RETURN_TO_PATH:
                {
                float dE = in.waypoints[_wp_idx][0] - bot_e;
                float dN = in.waypoints[_wp_idx][1] - bot_n;
                _out.target_heading_rad = atan2f(dE, dN);
                _out.speed_frac         = 0.7f;
                if (sqrtf(dE*dE+dN*dN) < in.wp_radius_m) {
                    _state = FSM_LAWNMOWER;
                }
                }
                break;
        }
        _out.current_wp_idx = _wp_idx;
    }

    FSMOutput getOutput() { return _out; }
    int currentWPIndex()  { return _wp_idx; }
    void resetWP()        { _wp_idx = 0; }

private:
    FSMState  _state;
    int       _wp_idx;
    float     _attack_e, _attack_n;
    bool      _attack_target_set;
    int       _rejoin_idx;
    FSMOutput _out;

    // Embedded re-join optimizer: cost(j) = dist + λ*(j - start)²
    int _rejoin_optimizer(float be, float bn, float (*wps)[2], int n, int start) {
        const float lambda = 0.5f;
        float best_cost = 1e9f;
        int   best_j    = start;
        for (int j = start; j < n; j++) {
            float dE = wps[j][0] - be;
            float dN = wps[j][1] - bn;
            float dist = sqrtf(dE*dE + dN*dN);
            float cost = dist + lambda * (float)((j-start)*(j-start));
            if (cost < best_cost) {
                best_cost = cost;
                best_j    = j;
            }
        }
        return best_j;
    }
};
