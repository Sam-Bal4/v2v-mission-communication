# Operation Touchdown — Field Deployment Guide
**Mission 2030 Codebase | Cube Orange+ / Jetson Orin (UAV) + Pixhawk / Jetson/Pi (UGV)**

---

## Hardware Summary
| Item | Spec |
|------|------|
| UAV frame | Tarot 680 Hexa-X |
| Flight controller (UAV) | Cube Orange+ |
| Companion computer (UAV) | Jetson Orin |
| Camera (UAV) | ZED X (USB-C, bottom-facing) |
| Rangefinder (UAV) | LidarLite v3 (I2C) |
| Optical flow (UAV) | Holybro H-Flow (DroneCAN 1) |
| Radio bridge (UAV) | ESP32 on `/dev/ttyUSB0` |
| Flight controller (UGV) | Pixhawk/SpeedyBee (Rover firmware) |
| Companion computer (UGV) | Jetson Nano / Raspberry Pi |
| Obstacle sensor (UGV) | TF-Nova Lidar (UART `/dev/ttyAMA0`) |
| Radio bridge (UGV) | ESP32 on `/dev/ttyUSB0` |
| LED indicators (UGV) | Green=GPIO16, Red=GPIO19 |

---

## One-Time Setup (Do Once Per Machine)

### 1. Clone the repo
```bash
git clone https://github.com/raccoon-exe/v2v-mission-communication.git
cd v2v-mission-communication
```

### 2. Auto-Config (Recommended)
You don't need to manually install dependencies. Our automated launchers now create a virtual environment and install everything for you on the first run.

**On the Drone/Rover (Linux):**
```bash
bash mission_2030/uav/start_mission.sh  # for UAV
bash mission_2030/ugv/start_mission.sh  # for UGV
```

**On Development PC (Windows):**
Simply double-click the `.bat` files:
- `mission_2030/uav/start_mission.bat`
- `mission_2030/ugv/start_mission.bat`

### 4. Flash ESP32 bridges
Each machine needs its own ESP32 flashed:
```bash
# Install PlatformIO first: pip3 install platformio

# Flash UAV bridge
cd mission_2030/esp32/uav_bridge
pio run --target upload

# Flash UGV bridge
cd mission_2030/esp32/ugv_bridge
pio run --target upload
```
> **Important**: Before flashing, edit `mission_2030/esp32/uav_bridge/src/main.cpp` and `ugv_bridge/src/main.cpp` and replace the `peerAddress[]` array with the actual MAC address of the opposing ESP32. Get MACs by running a simple `WiFi.macAddress()` sketch first.

### 5. Load ArduPilot parameters
- Open Mission Planner → Full Parameters List
- Load `mission_2030/nogps_lidar_optical_compass_geofence.param`
- **Manually set** `PLND_OPTIONS = 1` (moving landing target support — NOT in the param file)
- Write and reboot

---

## Competition Day Startup Sequence

### UAV Side (Jetson Orin)
Run the automated launcher and pick your mission from the menu:
```bash
bash mission_2030/uav/start_mission.sh
```

### UGV Side (Jetson/Pi)
Run the automated launcher:
```bash
bash mission_2030/ugv/start_mission.sh
```

> **Start UGV first**, then start UAV. The UAV waits for a UGV heartbeat before arming.

---

## What to Expect

### Mission 1
1. UAV connects to Cube Orange+
2. UAV waits for UGV radio heartbeat (ESP32 ↔ ESP32)
3. UGV is sitting with its landing pad marker (ArUco ID 0) on top
4. UAV arms → climbs to 1.3 m → enters LAND mode
5. UAV streams `LANDING_TARGET` packets to Cube, guiding it onto marker ID 0
6. Cube auto-disarms on touchdown (printed: `Cube heartbeat: motors DISARMED`)
7. 30-second ride timer starts
8. **`MISSION 1 COMPLETE`**

### Mission 2
1–3. Same as Mission 1 startup
4. UAV climbs → scouts for ArUco markers ID 1–4 (destination markers on field)
5. When found, sends ZED 3D coordinates to UGV via V2V radio
6. UGV drives toward destination
7. UAV follows UGV, streaming `LANDING_TARGET` for marker ID 0
8. Touchdown confirmed → 30-second ride → **`MISSION 2 COMPLETE`**

### Mission 3
Same as Mission 2, but:
- When UGV detects a bucket obstacle (TF-Nova < 1.5 m), it broadcasts an avoidance state
- UAV detects this via V2V and pauses descent (re-enters GUIDED, climbs slightly)
- Once UGV clears obstacle, UAV resumes LAND
- **`MISSION 3 COMPLETE`**

---

## Progressive Hardware Tests (Run in Order!)

Always run these tests before the competition mission. Each test is fully self-contained.

### Using Automated Launchers
The easiest way is to use the `start_mission.sh` script, which includes a dedicated menu for all 10 hardware tests:
```bash
bash mission_2030/uav/start_mission.sh
```
(Select `t1` through `t10`)

---

## Emergency Stop
- **On UAV**: Press `Ctrl+C` — the SIGINT handler will trigger `LAND` mode immediately. The drone will descend and disarm. The Python process will not die until the Cube confirms touchdown.
- **On UGV**: Press `Ctrl+C` — motors stop, vehicle disarms, bridge closes cleanly.
- **RC transmitter**: Always keep armed and in range. Switch to LAND or RTL manually if needed.

---

## Serial Port Reference
| Device | Port | Baud |
|--------|------|------|
| Cube Orange+ (UAV) | `/dev/ttyACM0` | 57600 |
| ESP32 Radio Bridge | `/dev/ttyUSB0` | 115200 |
| TF-Nova Lidar (UGV) | `/dev/ttyAMA0` | 115200 |
| Pixhawk/Rover FC (UGV) | `/dev/ttyACM0` | 115200 |

> If `/dev/ttyACM0` is busy: `sudo chmod 666 /dev/ttyACM0`  
> If ESP32 not found: check `ls /dev/ttyUSB*` and update `ESP32_PORT` in the runner file.

---

## Common Issues

| Symptom | Fix |
|---------|-----|
| `ModuleNotFoundError` | Use the automated launchers; they handle the environment for you |
| Drone never arms | Check `ARMING_CHECK` in params; ensure EKF3 is healthy (optical flow + lidar must be outputting) |
| `DISTANCE_SENSOR` reads 0 | LidarLite not initialized — check I2C wiring and `RNGFND1_TYPE=3`, `RNGFND1_ADDR=98` |
| Landing never confirms | Check `PLND_OPTIONS=1` is set in Mission Planner |
| UGV never gets destination | Check ESP32 MACs are correct in both `main.cpp` files |
| Motors stop before landing | Was fixed — `is_disarmed()` now filters by `get_srcSystem()`. Update to latest code. |
| ZED camera not found | Run `ZED_Explorer` first to verify SDK installed. Check USB-C connection. |

---

## File Structure
```
mission_2030/
├── uav/
│   ├── mission1_runner.py     ← Competition Mission 1
│   ├── mission2_runner.py     ← Competition Mission 2
│   ├── mission3_runner.py     ← Competition Mission 3
│   ├── ardupilot_control.py   ← Mode switching, arm/disarm, velocity commands
│   ├── precision_landing.py   ← LANDING_TARGET stream + touchdown detect
│   ├── takeoff.py             ← GUIDED + NAV_TAKEOFF + altitude wait
│   ├── ugv_tracking.py        ← ZED point cloud → MAVLink angles
│   ├── vision/aruco_detector.py  ← OpenCV ArUco wrapper
│   └── sensors/
│       ├── rangefinder_monitor.py  ← LidarLite v3 altitude reader
│       └── optical_flow_monitor.py ← H-Flow health check
├── ugv/
│   ├── ugv_runner.py          ← UGV main entry point
│   ├── obstacle_avoidance.py  ← TF-Nova lidar + GPIO LEDs
│   ├── telemetry_sender.py    ← Broadcasts UGV phase to UAV
│   └── state_machine.py       ← UGV states enum
├── radio/
│   ├── v2v_bridge.py          ← Serial ESP32 framing (TX/RX)
│   ├── v2v_protocol.py        ← Packet format constants
│   └── message_types.py       ← Pydantic data models
├── esp32/
│   ├── uav_bridge/            ← PlatformIO project for UAV ESP32
│   └── ugv_bridge/            ← PlatformIO project for UGV ESP32
├── dennis_test/
│   ├── drone/                 ← Tests 01-10, run alone on UAV
│   └── groundvehicle/         ← Matching UGV scripts for tests 03-07
└── nogps_lidar_optical_compass_geofence.param
```
