# 🌊 AWSCR — Autonomous Water Surface Cleaning Robot

> **Hardware-in-the-Loop (HIL) Proof of Concept** | ESP32 + MATLAB Simulation

[![ESP32](https://img.shields.io/badge/MCU-ESP32-blue)](https://www.espressif.com/en/products/socs/esp32)
[![MATLAB](https://img.shields.io/badge/Simulation-MATLAB-orange)](https://www.mathworks.com/products/matlab.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](LICENSE)

## Overview

AWSCR is an unmanned surface vehicle (USV) that autonomously cleans water bodies using:
- **Boustrophedon (lawnmower) coverage path planning** for systematic area cleaning
- **Real-time trash detection** via ESP32-CAM + TinyML with pixel-to-world localization
- **Attack mode** with Dubins-path intercept and smart re-join to the coverage path
- **EKF sensor fusion** of GPS + IMU + Compass for robust localization
- **MATLAB HIL simulation** of all sensors and USV dynamics (no hardware required for demo)

## System Architecture

```
IoT Dashboard (Leaflet.js) ←→ HiveMQ MQTT ←→ ESP32 (MCU)
                                                    ↕ Serial HIL
                                            MATLAB Simulator
                              (USV Dynamics + All Sensors + Visualization)
```

## Hardware (for real deployment)
| Component | Spec |
|---|---|
| MCU | ESP32 (240MHz dual-core, WiFi, BT) |
| IMU | MPU6050 (accel + gyro, I2C) |
| GPS | NEO-6M (UART, 9600 baud, ±2.5m CEP) |
| Compass | QMC5883L (I2C, 3-axis) |
| Ultrasonic | 3× HC-SR04 (front/left/right, 2cm–400cm) |
| LiDAR | RPLiDAR A1 (360°, 8m, 5.5Hz) |
| Camera | ESP32-CAM (OV2640, 320×240) |
| Motors | 2× A2212 1000KV BLDC |
| ESC | 2× 30A ESC (PWM 50Hz) |
| Battery | 3S LiPo 11.1V |
| Regulator | LM2596 (for ESP32 5V rail) |

## Repository Structure

```
wsv_robot/
├── matlab/            ← MATLAB HIL simulation (run this for demo)
│   ├── main_simulation.m      ← Entry point
│   ├── sensors/               ← All 6 sensor models
│   ├── navigation/            ← EKF, Boustrophedon, Dubins, Re-join
│   ├── control/               ← PID heading controller
│   ├── dynamics/              ← Fossen 3-DOF USV model
│   └── viz/                   ← Live visualization
├── esp32/             ← Arduino/PlatformIO firmware
│   └── awscr_firmware/
├── dashboard/         ← Web IoT dashboard
└── docs/              ← Architecture, math, BOM
```

## Documentation & Guides

For a detailed breakdown of the mathematical models, how to run the system, and a line-by-line explanation of the entire codebase, please see the `docs/` folder:

1. 📖 **[Codebase Explanation](docs/CODEBASE_EXPLANATION.md)** — Explains the behavior of every file, algorithm, and line group.
2. 🚀 **[How to Run](docs/HOW_TO_RUN.md)** — Step-by-step guide for pure MATLAB simulation, unit tests, Web Dashboard, and full ESP32 Hardware-in-the-Loop.
3. 🧮 **[Mathematical Derivations](docs/math_derivations.md)** — Haversine, EKF matrix math, optimal sweep algorithms, Dubins, and Fossen dynamics equations.
4. 📦 **[Bill of Materials](docs/BOM.md)** — Hardware list and cost estimates.

## Quick Start (Simulation Only — No Hardware Needed)

```matlab
% In MATLAB:
cd /path/to/wsv_robot/matlab
main_simulation
```

This opens a live visualization showing:
- The user-defined polygon (operation area)
- Boustrophedon coverage path
- Real-time bot position + heading
- Trash detections and attack intercepts
- EKF state estimates vs ground truth

## Modules

| # | Module | Description |
|---|---|---|
| 1 | USV Dynamics | Fossen 3-DOF surge-sway-yaw model |
| 2 | Sensor Simulation | MPU6050, NEO-6M, QMC5883L, Ultrasonic×3, LIDAR |
| 3 | EKF Estimator | GPS + IMU + Compass sensor fusion |
| 4 | Path Planning | Boustrophedon BCD + Dubins intercept |
| 5 | Control | PID heading controller, ESC thrust mapping |
| 6 | Detection | Trash pixel-to-world localization |
| 7 | FSM | IDLE→LAWNMOWER→ATTACK→RETURN_TO_PATH |
| 8 | HIL Protocol | UART packet TX/RX (MATLAB↔ESP32) |
| 9 | Dashboard | Leaflet.js + MQTT real-time telemetry |

## Key Algorithms

- **Coverage**: Boustrophedon Cell Decomposition (BCD), O(n²), 100% coverage guarantee
- **Localization**: 5-state EKF [lat, lon, heading, v_x, v_y]
- **Attack Path**: Dubins LSL/RSL shortest path (min radius 1.25m)
- **Re-join**: Cost-optimal re-entry to nearest forward waypoint
- **Obstacle Avoidance**: VFH+ using LIDAR scan histogram

## License

MIT — see [LICENSE](LICENSE)
