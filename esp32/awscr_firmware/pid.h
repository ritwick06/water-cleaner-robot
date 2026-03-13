/*
 * pid.h — Discrete PID heading controller with anti-windup (ESP32)
 */
#pragma once
#include <math.h>

class PIDController {
public:
    PIDController() : _Kp(1.5f), _Ki(0.3f), _Kd(0.45f),
                      _out_max(1.0f), _integral(0), _e_prev(0) {}

    void setGains(float Kp, float Ki, float Kd) {
        _Kp=Kp; _Ki=Ki; _Kd=Kd;
    }
    void setOutputLimits(float lo, float hi) {
        _out_min=lo; _out_max=hi;
    }

    float compute(float error, float dt) {
        // Wrap to [-pi, pi]
        error = atan2f(sinf(error), cosf(error));

        float P   = _Kp * error;
        float D   = _Kd * (error - _e_prev) / dt;
        float raw = P + _Ki * _integral + D;

        // Anti-windup: conditional integration
        if (raw > _out_max || raw < _out_min) {
            if (error * raw < 0) _integral += error * dt;  // only if reducing
        } else {
            _integral += error * dt;
        }

        // Clamp integral
        float max_int = _out_max / (_Ki + 0.001f);
        if (_integral >  max_int) _integral =  max_int;
        if (_integral < -max_int) _integral = -max_int;

        float out = P + _Ki * _integral + D;
        if (out >  _out_max) out =  _out_max;
        if (out < _out_min)  out =  _out_min;

        _e_prev = error;
        return out;
    }

    void reset() { _integral=0; _e_prev=0; }

private:
    float _Kp, _Ki, _Kd;
    float _out_max, _out_min;
    float _integral, _e_prev;
};

// ── motor_ctrl.h ─────────────────────────────────────────────────────────
/*
 * motor_ctrl.h — A2212 1000KV BLDC + 30A ESC PWM driver (ESP32)
 *
 * ESC input: 50Hz PWM, 1000μs = stop, 2000μs = full throttle
 *
 * Uses ESP32 LEDC peripheral for PWM generation.
 */
#pragma once
#include <Arduino.h>

class MotorController {
public:
    void init(int pin_L, int pin_R) {
        _pin_L = pin_L;
        _pin_R = pin_R;
        // LEDC channels 0 and 1 at 50Hz (20ms period), 16-bit resolution
        ledcSetup(0, 50, 16);   // CH0 = left
        ledcSetup(1, 50, 16);   // CH1 = right
        ledcAttachPin(pin_L, 0);
        ledcAttachPin(pin_R, 1);
        _rpm_L = 0; _rpm_R = 0;
    }

    void arm() {
        // Send neutral (1500μs) for 2 seconds to arm ESCs
        _writePWM(0, 1500);
        _writePWM(1, 1500);
        delay(2000);
    }

    // speed_frac: [0,1] forward speed,  u: [-1,1] differential (+ = turn right)
    void setDifferential(float speed_frac, float u) {
        float base = 1000.0f + speed_frac * 700.0f;   // 1000–1700 μs
        float gain = 200.0f;

        float pw_L = base - u * gain;
        float pw_R = base + u * gain;

        pw_L = constrain(pw_L, 1000, 2000);
        pw_R = constrain(pw_R, 1000, 2000);

        _writePWM(0, (int)pw_L);
        _writePWM(1, (int)pw_R);

        // Approximate RPM (linear mapping)
        const float KV = 1000.0f, V = 11.1f;
        float max_rpm  = KV * V;
        _rpm_L = (int)(((pw_L - 1000.0f) / 1000.0f) * max_rpm);
        _rpm_R = (int)(((pw_R - 1000.0f) / 1000.0f) * max_rpm);
    }

    void setThrottle(float thr_L, float thr_R) {
        int pw_L = 1000 + (int)(thr_L * 1000);
        int pw_R = 1000 + (int)(thr_R * 1000);
        _writePWM(0, pw_L);
        _writePWM(1, pw_R);
        _rpm_L = 0; _rpm_R = 0;
    }

    int getRPM_L() { return _rpm_L; }
    int getRPM_R() { return _rpm_R; }

private:
    int _pin_L, _pin_R;
    int _rpm_L, _rpm_R;

    void _writePWM(int ch, int pulse_us) {
        // 65535 ticks = 20ms → 1μs = 65535/20000 ticks
        uint32_t duty = (uint32_t)(pulse_us * 65535UL / 20000UL);
        ledcWrite(ch, duty);
    }
};
