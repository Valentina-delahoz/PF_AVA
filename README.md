# AVA — Autonomous Vehicle Prototype Powered by AI Agents

> A scale autonomous vehicle with two specialized AI agents for safe navigation in controlled urban environments.

**Universidad del Norte** · Electrical and Electronic Engineering · 2026-1

**Authors:** Valentina De la Hoz · Roger Coronell · Samir Albor  
**Advisors:** Ph.D. Christian Quintero · Ph.D. Mauricio Pardo

---

## What is AVA?

AVA is a scale autonomous vehicle prototype built entirely with Universidad del Norte resources (including a 100% 3D-printed body). It integrates two artificial intelligence agents:

- **Perception agent**: detects traffic signs, traffic lights and vehicles, and follows the lane in real time
- **Decision agent**: interprets the perception output and executes autonomous driving actions through a Finite State Machine (FSM)

The goal is a low-cost, replicable educational and research platform for experimenting with AI agents in autonomous driving, without the cost of industrial vehicles or the limits of purely virtual simulators.

---

## Results

| Metric | Value |
|---|---|
| mAP@0.5 (object detection) | 91.3% |
| F1-Score | 0.94 |
| Mean global ADR (30 experiments) | 90.60% |
| System latency | ~50 ms |
| Real-world frame rate | ~10 FPS |
| Battery life | ~2 hours |

> With **95% confidence**, AVA correctly resolves more than **85%** of navigation events across tracks of varying complexity (one-tailed Student's t-test, α = 0.05).

*ADR (Accurate Decision Rate) is the percentage of checkpoints a run resolves correctly, e.g. taking a curve without leaving the lane, stopping at a red light, or detecting a stop sign.*

---

## System Architecture

```
Webcam
    │
    ▼
Raspberry Pi 4  ──── YOLOv8 (object detection)
    │           ──── Hough Transform (lane following)
    │           ──── FSM (decision agent)
    │
    ▼  UART (Serial)
ESP32
    │
    ├── DC motor 25GA370 (traction) → L298N driver
    ├── MG995 servo (steering)
    └── 2× HC-SR04 ultrasonic sensors (obstacle detection)
```

**Software:** Python · ROS2 · OpenCV · YOLOv8 (Ultralytics)  
**Hardware:** Raspberry Pi 4 · ESP32 · 3S 2200mAh LiPo · Lightbek-PLU webcam

---

## Repository Structure

```
PF_AVA/
├── raspberry_pi/
│   └── ava_main.py          # Perception and control agent (Python)
├── esp32/
│   └── esp32_firmware.ino   # Actuation and telemetry firmware (C++)
└── README.md
```

---

## Classes Detected by YOLOv8

| Class | Vehicle action |
|---|---|
| `green-traffic-light` | Go |
| `red-traffic-light` | Stop (until green) |
| `stop-sign` | Stop for 3 seconds |
| `vehicle` | Slow down |

The model is a pretrained YOLOv8 fine-tuned on a custom dataset captured on the project's own test tracks. Detection rates were perfect for traffic lights and vehicles; the stop sign (mAP@0.5 of 66.5%) is the main area for improvement, due to its smaller training set.

---

## Decision Hierarchy (FSM)

```
P1 — Obstacle < 15 cm              → STOPPED (ultrasonic sensors)
P2 — Red light / Stop sign         → STOPPED (vision)
P3 — Vehicle detected              → SLOW
P4 — Clear road / green light      → NORMAL
```

---

## Test Tracks

| Track | Description | Checkpoints |
|---|---|---|
| Track 1 | Curves and straights, no signs | 4 |
| Track 2 | Straight road with traffic lights and signs | 5–6 |
| Track 3 | Full intersection with obstacles | 6 |

Each track was run 10 times (30 runs in total). Mean ADR per track: **91.5%** (Track 1), **90.3%** (Track 2), **90.0%** (Track 3), so higher environment complexity did not significantly degrade performance.

---

## Installation

### Raspberry Pi 4

```bash
pip install ultralytics opencv-python pyserial
python3 raspberry_pi/ava_main.py
```

### ESP32

1. Open `esp32/esp32_firmware.ino` in the Arduino IDE
2. Install the `ESP32Servo` library
3. Select the `ESP32 Dev Module` board
4. Upload the firmware

---

## How to Use AVA

1. Fully charge the 3S 2200mAh LiPo battery
2. Turn on the vehicle's physical switch
3. Connect over SSH via ZeroTier
4. Autonomous navigation starts automatically
5. To stop, turn off the physical switch

---

## Future Work

- Migrate to an NVIDIA Jetson Nano to reduce latency and increase inference FPS
- Expand the stop-sign training dataset
- Replace or complement the FSM with a deep reinforcement learning agent

---

## Publication

The final report is available in the Universidad del Norte Institutional Repository: http://hdl.handle.net/10584/14118

---

*Capstone project — Electrical and Electronic Engineering, Universidad del Norte, Barranquilla, Colombia, 2026.*
