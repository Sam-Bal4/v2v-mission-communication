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

### 2. Install Python dependencies
```bash
# On UAV (Jetson Orin) — pyzed comes from ZED SDK installer, not pip
pip3 install pymavlink opencv-python numpy pyserial pydantic

# On UGV (Jetson/Pi)
pip3 install pymavlink dronekit pyserial gpiozero numpy pydantic
```
> **ZED SDK**: Install from https://www.stereolabs.com/developers/release — this installs `pyzed` automatically.

### 3. Set PYTHONPATH (required every terminal session, or add to ~/.bashrc)
```bash
cd v2v-mission-communication
export PYTHONPATH=$(pwd)
```
Or permanently:
```bash
echo 'export PYTHONPATH=/path/to/v2v-mission-communication' >> ~/.bashrc
source ~/.bashrc
```

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

```bash
cd v2v-mission-communication
export PYTHONPATH=$(pwd)
```

Then pick the correct mission:

| Mission | Command |
|---------|---------|
| Mission 1 (Basic hover + land on UGV) | `python3 mission_2030/uav/mission1_runner.py` |
| Mission 2 (Scan field + send to UGV + land on UGV) | `python3 mission_2030/uav/mission2_runner.py` |
| Mission 3 (Mission 2 + obstacle avoidance relay) | `python3 mission_2030/uav/mission3_runner.py` |

### UGV Side (Jetson/Pi)

```bash
cd v2v-mission-communication
export PYTHONPATH=$(pwd)
python3 mission_2030/ugv/ugv_runner.py
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

```bash
cd v2v-mission-communication
export PYTHONPATH=$(pwd)
cd mission_2030/dennis_test
```

### Drone Tests
| Test | Command | What it Validates |
|------|---------|-------------------|
| 01 | `python3 drone/01_basic_flight.py` | Motor arming, takeoff, alt hold, safe land |
| 02 | `python3 drone/02_aruco_land.py` | ZED camera opens, ArUco detection works |
| 03 | `python3 drone/03_move_ugv_5ft.py` | ESP32 V2V link, UAV can command UGV |
| 04 | `python3 drone/04_ugv_to_aruco.py` | ZED 3D point cloud works, coords sent to UGV |
| 05 | `python3 drone/05_fly_side_by_side_forward.py` | Forward velocity vector in GUIDED mode |
| 06 | `python3 drone/06_fly_side_by_side_left.py` | Left sway vector |
| 07 | `python3 drone/07_fly_side_by_side_right.py` | Right sway vector |
| 08 | `python3 drone/08_center_hover_high.py` | Proportional centering at 1.3 m |
| 09 | `python3 drone/09_center_hover_low.py` | Centering at 0.5 m (landing approach) |
| 10 | `python3 drone/10_precision_land.py` | Full LANDING_TARGET precision touchdown |

### UGV Tests (run simultaneously with drone tests 3, 4, 5, 6, 7)
| Test | Command |
|------|---------|
| 03 | `python3 groundvehicle/03_move_ugv_5ft.py` |
| 04 | `python3 groundvehicle/04_ugv_to_aruco.py` |
| 05 | `python3 groundvehicle/05_fly_side_by_side_forward.py` |
| 06 | `python3 groundvehicle/06_fly_side_by_side_left.py` |
| 07 | `python3 groundvehicle/07_fly_side_by_side_right.py` |

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
| `ModuleNotFoundError: mission_2030` | Run `export PYTHONPATH=$(pwd)` from the repo root |
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
