/*
 * AWSCR ESP32 Firmware — Main Arduino Sketch
 * Autonomous Water Surface Cleaning Robot
 *
 * Architecture:
 *   - EKF sensor fusion (GPS + IMU + Compass) at 50Hz
 *   - Boustrophedon path following
 *   - FSM: IDLE → LAWNMOWER → ATTACK → RETURN_TO_PATH
 *   - PID heading controller → ESC PWM
 *   - MQTT telemetry to IoT dashboard
 *   - Serial HIL interface with MATLAB simulator
 *
 * Pin Assignments (ESP32):
 *   GPIO 21 = SDA  (MPU6050 + QMC5883L)
 *   GPIO 22 = SCL
 *   GPIO 16 = UART2 RX  (NEO-6M GPS TX)
 *   GPIO 17 = UART2 TX  (NEO-6M GPS RX)
 *   GPIO 25 = ESC Left  PWM (50Hz)
 *   GPIO 26 = ESC Right PWM (50Hz)
 *   GPIO 13 = Ultrasonic Front TRIG
 *   GPIO 12 = Ultrasonic Front ECHO
 *   GPIO 14 = Ultrasonic Left  TRIG
 *   GPIO 27 = Ultrasonic Left  ECHO
 *   GPIO 32 = Ultrasonic Right TRIG
 *   GPIO 33 = Ultrasonic Right ECHO
 *   GPIO  0 = UART0 RX  (HIL Serial MATLAB, also used for programming)
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "ekf.h"
#include "path_planner.h"
#include "fsm.h"
#include "pid.h"
#include "motor_ctrl.h"
#include "sensors.h"
#include "mqtt_client_awscr.h"
#include "hil_serial.h"

// ── WiFi & MQTT Configuration ─────────────────────────────────────────────
#define WIFI_SSID     "YOUR_SSID"
#define WIFI_PASS     "YOUR_PASSWORD"
#define MQTT_BROKER   "broker.hivemq.com"
#define MQTT_PORT     1883
#define MQTT_CLIENT_ID "awscr_esp32_001"
#define TOPIC_TELEM   "awscr/telemetry"
#define TOPIC_CMD     "awscr/command"
#define TOPIC_MISSION "awscr/mission"

// ── Hardware ───────────────────────────────────────────────────────────────
#define PIN_ESC_L    25
#define PIN_ESC_R    26
#define PIN_US_F_TR  13
#define PIN_US_F_EC  12
#define PIN_US_L_TR  14
#define PIN_US_L_EC  27
#define PIN_US_R_TR  32
#define PIN_US_R_EC  33

// ── Global Objects ────────────────────────────────────────────────────────
WiFiClient      wifiClient;
PubSubClient    mqtt(wifiClient);

EKF             ekf;
PathPlanner     pathPlanner;
FSMController   fsm;
PIDController   pid_heading;
MotorController motors;
SensorManager   sensors;
HILSerial       hil;

// ── Timing ────────────────────────────────────────────────────────────────
static uint32_t t_last_50hz   = 0;
static uint32_t t_last_1hz    = 0;
static uint32_t t_last_mqtt   = 0;
static uint32_t t_last_sensor = 0;

// ── Mission state ─────────────────────────────────────────────────────────
static bool mission_active = false;

// ── MQTT callback ─────────────────────────────────────────────────────────
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    String msg = "";
    for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

    if (String(topic) == TOPIC_MISSION) {
        StaticJsonDocument<1024> doc;
        if (deserializeJson(doc, msg) == DeserializationError::Ok) {
            JsonArray polygon = doc["polygon"];
            if (polygon.size() >= 3) {
                pathPlanner.setPolygon(polygon);
                pathPlanner.computeBoustrophedon(0.6f);  // 0.6m sweep width
                fsm.setState(FSM_LAWNMOWER);
                mission_active = true;
                Serial.println("[MQTT] Mission polygon received, planning path...");
                Serial.printf("[MQTT] %d waypoints generated\n", pathPlanner.waypointCount());
            }
        }
    }

    if (String(topic) == TOPIC_CMD) {
        if (msg == "stop")  { fsm.setState(FSM_IDLE); mission_active = false; }
        if (msg == "start") { fsm.setState(FSM_LAWNMOWER); mission_active = true; }
    }
}

// ─────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("\n== AWSCR ESP32 Boot ==");

    // I2C
    Wire.begin(21, 22);
    Wire.setClock(400000);

    // Sensors
    sensors.init(PIN_US_F_TR, PIN_US_F_EC,
                 PIN_US_L_TR, PIN_US_L_EC,
                 PIN_US_R_TR, PIN_US_R_EC);

    // Motors (ESC)
    motors.init(PIN_ESC_L, PIN_ESC_R);
    motors.arm();   // send neutral PWM for 2s

    // PID
    pid_heading.setGains(1.5f, 0.3f, 0.45f);
    pid_heading.setOutputLimits(-1.0f, 1.0f);

    // EKF — initialized with zero position; GPS will correct
    ekf.init();

    // WiFi
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500); Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] Connected: %s\n", WiFi.localIP().toString().c_str());
        mqtt.setServer(MQTT_BROKER, MQTT_PORT);
        mqtt.setCallback(mqtt_callback);
        mqtt.setBufferSize(2048);
    } else {
        Serial.println("\n[WiFi] OFFLINE — MQTT disabled");
    }

    // HIL Serial (UART0 / USB for MATLAB link)
    hil.init(&Serial, 115200);

    Serial.println("[BOOT] System ready.\n");
}

// ─────────────────────────────────────────────────────────────────────────
void loop() {
    uint32_t now = millis();

    // ── MQTT keep-alive ─────────────────────────────────────────────────
    if (WiFi.status() == WL_CONNECTED) {
        if (!mqtt.connected()) {
            if (mqtt.connect(MQTT_CLIENT_ID)) {
                mqtt.subscribe(TOPIC_CMD);
                mqtt.subscribe(TOPIC_MISSION);
                Serial.println("[MQTT] Connected");
            }
        }
        mqtt.loop();
    }

    // ── 50Hz Control Loop ────────────────────────────────────────────────
    if (now - t_last_50hz >= 20) {
        t_last_50hz = now;

        // 1. Read sensors (real or HIL)
        SensorData sd;
        if (hil.hasPacket()) {
            hil.readPacket(sd);   // HIL: get sensor data from MATLAB
        } else {
            sensors.read(sd);     // Real hardware sensors
        }

        // 2. EKF predict + update
        ekf.predict(sd.imu_ax, sd.imu_ay, sd.imu_gz, 0.02f);
        if (sd.gps_valid) {
            ekf.updateGPS(sd.gps_lat, sd.gps_lon);
        }
        ekf.updateCompass(sd.compass_hdg_deg);

        EKFState est = ekf.getState();

        // 3. Obstacle check (ultrasonic)
        bool danger = (sd.us_front_cm < 60 || sd.us_left_cm < 40 || sd.us_right_cm < 40);

        // 4. FSM tick
        FSMInput fsmIn;
        fsmIn.est          = est;
        fsmIn.waypoints    = pathPlanner.getWaypoints();
        fsmIn.n_waypoints  = pathPlanner.waypointCount();
        fsmIn.detections   = sd.detections;
        fsmIn.n_detections = sd.n_detections;
        fsmIn.wp_radius_m  = 0.4f;
        fsmIn.collect_r_m  = 0.3f;
        fsmIn.danger_flag  = danger;
        fsmIn.dt           = 0.02f;
        fsm.tick(fsmIn);

        FSMOutput fsmOut = fsm.getOutput();

        // 5. PID heading control
        float e_hdg = normalizeAngle(fsmOut.target_heading_rad - est.psi_rad);
        float u     = pid_heading.compute(e_hdg, 0.02f);

        // 6. Motor commands
        if (fsm.getState() == FSM_IDLE || !mission_active || danger) {
            motors.setThrottle(0.0f, 0.0f);
        } else {
            motors.setDifferential(fsmOut.speed_frac, u);
        }

        // 7. HIL feedback to MATLAB
        hil.sendFeedback(fsm.getState(), pathPlanner.currentWPIndex(),
                         motors.getRPM_L(), motors.getRPM_R());
    }

    // ── 1Hz Telemetry ─────────────────────────────────────────────────────
    if (now - t_last_mqtt >= 1000 && mqtt.connected()) {
        t_last_mqtt = now;
        EKFState est = ekf.getState();

        StaticJsonDocument<512> doc;
        doc["t"]        = now / 1000.0f;
        doc["lat"]      = est.lat_deg;
        doc["lon"]      = est.lon_deg;
        doc["hdg"]      = degrees(est.psi_rad);
        doc["speed"]    = sqrt(est.vx*est.vx + est.vy*est.vy);
        doc["mode"]     = fsm.getStateName();
        doc["wp"]       = pathPlanner.currentWPIndex();
        doc["n_wp"]     = pathPlanner.waypointCount();
        doc["us_f"]     = sensors.getFront();
        doc["us_l"]     = sensors.getLeft();
        doc["us_r"]     = sensors.getRight();

        String payload;
        serializeJson(doc, payload);
        mqtt.publish(TOPIC_TELEM, payload.c_str());
    }
}

// ─────────────────────────────────────────────────────────────────────────
float normalizeAngle(float a) {
    while (a >  PI) a -= 2*PI;
    while (a < -PI) a += 2*PI;
    return a;
}
