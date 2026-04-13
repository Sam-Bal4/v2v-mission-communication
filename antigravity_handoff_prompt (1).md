# Antigravity IDE Handoff: ArduPilot-Based Operation Touchdown System

## Purpose
Build a competition-ready **ArduPilot-based UAV/UGV collaborative autonomy system** for the Raytheon AVC “Operation Touchdown” challenge using the exact hardware below. The system must support:

1. UAV autonomous launch from the UGV.
2. UAV scouting and finding the destination ArUco marker.
3. UAV sending destination information **directly to the UGV without a ground station**.
4. UGV driving to the destination.
5. UAV landing on the **moving** UGV.
6. Entire system arriving at the destination.
7. No GPS required for the UAV landing flow.

This prompt is intended for an AI coding assistant that will write the full implementation.

---

## Hardware already chosen

### UAV
- Cube Orange+
- Hexa X airframe
- Jetson Nano companion computer
- ZED X camera mounted downward
- Garmin / PulsedLight LidarLite v3 downward
- Holybro H-Flow optical flow sensor
- ESP32-S radio module for direct UAV↔UGV link

### UGV
- ESP32-S radio module for direct UAV↔UGV link
- Existing UGV autopilot/control stack and drive computer are already being coded around MAVLink/DroneKit-style control

### Existing uploaded code to reuse/adapt
- `mission1.py` — rough UAV mission script prototype
- `v2v_bridge.py` — serial framing + ESP32 bridge protocol
- `esp_uav.cpp` — ESP-NOW bridge firmware for UAV-side ESP32
- `esp_ugv.cpp` — ESP-NOW bridge firmware for UGV-side ESP32
- `ground_station.py` — rough UGV motion control prototype

Treat these files as **starting references**, not finished production code.

---

## Competition requirements extracted from the uploaded rules PDF
These are non-negotiable system constraints.

### Core scenario
- The UAV must autonomously launch and scout the field containing **1 destination ArUco marker** and obstacles.
- The UAV must **directly communicate** the destination to the UGV **without using a ground station**.
- The UGV then travels to the destination while avoiding obstacles.
- The UAV lands on the moving UGV and continues with it to the destination.
- Both UAV and UGV must behave **fully autonomously**.

### Challenge timing and motion constraints
- Minimum UAV altitude during challenge flight: **4 feet**.
- Minimum UAV flight time: **5 seconds**.
- For Challenge 2 and 3, the UAV must land on the moving UGV within **10 minutes from first UGV movement**.
- After landing, the UAV must remain with the moving UGV for **10 seconds** without separating.
- UGV minimum speed: **0.2 mph**.

### Destination and obstacle constraints
- The destination is the space within a **5-foot radius** circle from the center of the designated ArUco marker.
- Challenge 3 obstacles are real 3D objects the UGV cannot drive over.
- Obstacles may include cardboard boxes, traffic cones, and buckets.
- Minimum obstacle spacing: **5 feet**.

### Operational and management constraints
- Teams must document a **requirements matrix** and **test documentation**.
- Code and design should be easy to adjust for field conditions.
- The rules explicitly warn teams to test ArUco detection distance, angle sensitivity, and lighting robustness.

---

## Design decision: use ArduPilot precision landing, not RC override flight logic
The uploaded `mission1.py` currently uses RC override / throttle pulses and a rough vision loop. Do **not** keep that as the main architecture.

The UAV stack must be rewritten around:

- **ArduCopter on Cube Orange+**
- **Precision Landing using MAVLink `LANDING_TARGET` messages from Jetson Nano**
- **Optical flow + downward rangefinder for low-altitude GPS-denied stability**
- **ArUco-based relative target estimation on the Jetson**
- **Direct UAV↔UGV communication through the ESP32 bridge**

The UGV stack may continue to use a MAVLink/DroneKit-style control program if that is already your path, but the interface should be cleaned up and formalized.

---

## High-level architecture

### UAV responsibilities
1. Boot Jetson services.
2. Connect to Cube Orange+ over MAVLink.
3. Acquire camera frames from the downward-facing ZED X.
4. Detect the destination ArUco on the field.
5. Estimate the destination location relative to a chosen frame.
6. Send destination payload directly to the UGV over the ESP32 bridge.
7. Track the moving UGV landing marker.
8. Send `LANDING_TARGET` updates to ArduPilot at a high rate during the landing phase.
9. Command ArduPilot modes cleanly: arm, takeoff, navigate/search, land.
10. Log all detections, communications, and mission state transitions.

### UGV responsibilities
1. Receive destination payload directly from the UAV.
2. Acknowledge receipt to the UAV.
3. Drive toward the destination while avoiding obstacles.
4. Broadcast motion state, speed, and optional heading back to the UAV.
5. Expose a stable landing deck with a dedicated landing ArUco marker.
6. Continue moving after touchdown long enough to satisfy ride requirements.

### ESP32 bridge responsibilities
1. Provide direct, low-latency UAV↔UGV messaging.
2. Frame packets over serial using the existing `0xAA + type + len + payload + xor` protocol.
3. Forward telemetry, commands, and debug messages over ESP-NOW.
4. Support reliable heartbeats and sequence numbering.

---

## Physical marker strategy
Use **two different marker roles**.

### Marker A: destination marker on the field
- This is the competition destination marker placed by judges.
- The UAV must detect it during scouting.
- The UAV sends the destination information to the UGV.

### Marker B: landing marker on top of the UGV
- This is your own landing fiducial mounted flat on the UGV landing deck.
- It should be large, high-contrast, and easy to detect while the UGV is moving.
- The UAV uses Marker B for moving-platform precision landing.

Do not reuse the same mission logic for both markers. Use separate IDs and separate state-machine roles.

---

## Key implementation assumptions
These assumptions should be made explicit in code/config:

1. **ArUco dictionary**: use a configurable dictionary and marker IDs.
2. **Camera**: downward-facing ZED X, calibrated, with saved intrinsics/extrinsics.
3. **Coordinate frame**:
   - Use vehicle body frame for `LANDING_TARGET` position fields.
   - Prefer full `x, y, z` body-frame landing target when pose is available.
   - Fall back to `angle_x`, `angle_y` only if needed.
4. **Altitude source**: LidarLite v3 is the primary low-altitude height-above-ground sensor.
5. **Flow source**: Holybro H-Flow provides optical flow; onboard lidar from a flow unit should not be the primary altitude sensor if a better lidar is available.
6. **Moving target support**: ArduPilot precision landing must be configured for a moving landing target.
7. **No ground station in mission logic**: direct UAV↔UGV messaging only for competition mission data flow.
8. **Safety**: emergency stop, comm-loss handling, and target-loss handling must be implemented.

---

## Required software deliverables
The AI should generate a full project tree like this.

```text
operation_touchdown/
  README.md
  requirements.txt
  pyproject.toml
  config/
    airframe.yaml
    camera.yaml
    markers.yaml
    mission.yaml
    mavlink.yaml
    radio.yaml
    ugv.yaml
  docs/
    requirements_matrix.md
    test_plan.md
    wiring.md
    field_ops_checklist.md
  common/
    __init__.py
    logging_utils.py
    math_utils.py
    frames.py
    time_sync.py
    health.py
  radio/
    __init__.py
    v2v_protocol.py
    v2v_bridge.py
    message_types.py
    heartbeat.py
  uav/
    __init__.py
    main.py
    state_machine.py
    mission_manager.py
    mavlink_client.py
    ardupilot_control.py
    precision_landing.py
    takeoff.py
    search_pattern.py
    destination_detection.py
    ugv_tracking.py
    camera/
      __init__.py
      zedx_camera.py
      calibration.py
    vision/
      __init__.py
      aruco_detector.py
      pose_estimator.py
      landing_target_encoder.py
      target_filter.py
    sensors/
      __init__.py
      rangefinder_monitor.py
      optical_flow_monitor.py
    safety/
      __init__.py
      failsafes.py
      watchdog.py
  ugv/
    __init__.py
    main.py
    state_machine.py
    motion_controller.py
    mavlink_ground_vehicle.py
    destination_receiver.py
    obstacle_avoidance.py
    telemetry_sender.py
    deck_status.py
  esp32/
    uav_bridge/
      platformio.ini
      src/main.cpp
    ugv_bridge/
      platformio.ini
      src/main.cpp
  scripts/
    run_uav.sh
    run_ugv.sh
    calibrate_camera.py
    print_marker_pack.py
    test_radio_link.py
    bench_test_landing_target.py
    replay_log.py
  tests/
    test_protocol.py
    test_aruco_pose.py
    test_state_machine.py
    test_landing_target_encoding.py
    test_config_loading.py
```

---

## Mission state machine to implement

### UAV state machine
```text
BOOT
  -> WAIT_FOR_AUTOPILOT
  -> WAIT_FOR_RADIO_LINK
  -> PREFLIGHT_CHECKS
  -> ARM
  -> TAKEOFF
  -> SEARCH_DESTINATION_MARKER
  -> ESTIMATE_DESTINATION
  -> SEND_DESTINATION_TO_UGV
  -> WAIT_FOR_UGV_ACK
  -> SEARCH_UGV_LANDING_MARKER
  -> TRACK_MOVING_UGV
  -> PRECISION_LANDING_APPROACH
  -> TOUCHDOWN_CONFIRM
  -> RIDE_ALONG_TIMER
  -> MISSION_COMPLETE

FAILSAFE states:
  COMM_LOSS
  TARGET_LOSS
  RANGEFINDER_BAD
  CAMERA_BAD
  EKF_BAD
  ABORT_LAND
  EMERGENCY_LAND
```

### UGV state machine
```text
BOOT
  -> WAIT_FOR_RADIO_LINK
  -> PREFLIGHT_CHECKS
  -> WAIT_FOR_DESTINATION
  -> ACK_DESTINATION
  -> START_MOTION
  -> NAVIGATE_TO_DESTINATION
  -> AVOID_OBSTACLE
  -> EXPECT_UAV_TOUCHDOWN
  -> TOUCHDOWN_CONFIRMED
  -> CONTINUE_TO_DESTINATION
  -> STOP_AT_DESTINATION
  -> MISSION_COMPLETE

FAILSAFE states:
  COMM_LOSS
  DESTINATION_TIMEOUT
  ESTOP
  DRIVE_FAULT
```

---

## MAVLink and ArduPilot behavior required

### UAV control rules
- Use proper ArduPilot mode changes and arming flow.
- Use guided/takeoff/landing APIs rather than RC override as the primary strategy.
- Use Precision Landing via **MAVLink `LANDING_TARGET`** for the moving UGV landing phase.
- Send `LANDING_TARGET` at high rate during terminal guidance.
- Prefer filling `x`, `y`, `z` body-frame fields with `position_valid=1` when full pose is known.

### Important ArduPilot parameters to support in config
These should be present in config and documented in setup notes:

- `PLND_ENABLED = 1`
- `PLND_TYPE = 1` for MAVLink `LANDING_TARGET`
- `PLND_ORIENT`
- `PLND_YAW_ALIGN`
- `PLND_CAM_POS_X`
- `PLND_CAM_POS_Y`
- `PLND_CAM_POS_Z`
- `PLND_LAND_OFS_X`
- `PLND_LAND_OFS_Y`
- `PLND_XY_DIST_MAX`
- `PLND_STRICT`
- `PLND_ALT_MIN`
- `PLND_ALT_MAX`
- `PLND_OPTIONS` with moving-target support enabled
- `FLOW_TYPE`
- `FLOW_POS_X`, `FLOW_POS_Y`, `FLOW_POS_Z`
- `RNGFND1_TYPE`
- `RNGFND1_MIN`
- `RNGFND1_MAX`
- `RNGFND1_ORIENT`
- `LAND_SPD_MS`

### Optical flow and CAN assumptions
For a DroneCAN optical flow unit, support config patterns such as:
- `FLOW_TYPE = 6` if using DroneCAN flow
- `CAN_P1_DRIVER = 1`
- `CAN_D1_PROTOCOL = 1`

### LidarLite v3 caution
Treat LidarLite v3 integration as bench-test critical. The code and docs must note:
- PWM is preferred over I2C if available due to documented I2C issues.
- The rangefinder must be validated on the bench and in logs before flight use.

---

## Vision implementation requirements

### Destination marker detection
Implement a module that:
1. Grabs frames from the ZED X.
2. Detects the destination ArUco marker.
3. Estimates its pose relative to the camera using calibrated intrinsics.
4. Converts the observation into a destination payload suitable for the UGV.
5. Logs confidence, marker ID, timestamp, and relative pose.

### Moving UGV landing marker detection
Implement a separate tracking module that:
1. Detects the landing marker on the UGV deck.
2. Estimates relative pose at real-time rate.
3. Filters jitter using a configurable filter.
4. Rejects outliers and stale detections.
5. Converts target pose into `LANDING_TARGET` messages.
6. Continues feeding ArduPilot until touchdown or target loss.

### Required helper modules
- camera calibration loader
- image-to-body-frame transform
- marker-size-aware pose estimator
- temporal filter (EMA or Kalman)
- confidence gate
- target-loss detector

---

## Radio / direct communication requirements
Build on the uploaded `v2v_bridge.py`, `esp_uav.cpp`, and `esp_ugv.cpp`, but clean them up.

### Keep the same wire protocol concept
- Start byte: `0xAA`
- Frame fields: `type`, `len`, `payload`, xor checksum
- Packet families:
  - telemetry
  - commands
  - debug/status text

### Add formal message schemas
Define explicit payloads for:

#### UAV -> UGV
- heartbeat
- destination found
- destination pose
- mission phase
- estop
- touchdown predicted
- touchdown confirmed

#### UGV -> UAV
- heartbeat
- motion state
- speed
- yaw / heading if available
- destination ack
- deck ready
- estop

### Reliability requirements
- sequence numbers
- duplicate rejection
- heartbeat timeout
- reconnection handling
- telemetry timestamping
- mission event logging

---

## Safety and failsafe requirements
Implement all of these explicitly.

### UAV failsafes
- MAVLink link loss
- radio link loss to UGV
- camera stream failure
- landing marker lost during descent
- rangefinder unhealthy or stale
- optical flow unhealthy
- EKF unhealthy
- low battery hook
- emergency land command

### UGV failsafes
- loss of destination message
- loss of UAV heartbeat
- estop from UAV or operator
- obstacle avoidance timeout
- drive controller fault

### Behavior rules
- If the landing target is lost above the configured threshold, hold or retry based on mission config.
- If the target is lost below the configured threshold, allow controlled vertical landing only if explicitly configured.
- If UGV heartbeat is lost during final approach, abort landing and climb/hold if safe.
- If any critical health monitor fails, transition to a safe state and log it.

---

## Exact coding tasks for the AI
The AI should write all of the following.

### 1. Config system
- Strongly typed config loader.
- YAML-based configs for airframe, camera, markers, mission, radio, and UGV assumptions.
- Validation for missing keys and invalid values.

### 2. MAVLink wrapper
- Robust Python MAVLink client.
- Connection retry logic.
- Heartbeat wait.
- Mode change helper.
- Arm/disarm helper.
- Takeoff helper.
- Land helper.
- `LANDING_TARGET` sender.
- Health subscribers for EKF, distance sensor, optical flow, battery, and mode.

### 3. ArUco vision stack
- OpenCV-based detector.
- Camera calibration loading.
- Pose estimation using marker size.
- Two logical trackers:
  - destination marker tracker
  - landing marker tracker
- Jitter filtering and outlier rejection.

### 4. Precision landing module
- Converts filtered target pose into MAVLink `LANDING_TARGET`.
- Supports angle-only mode and full-position mode.
- Supports moving-platform landing.
- Runs in a timed loop with configurable rate.

### 5. UAV mission manager
- Full UAV state machine.
- Timeouts for each phase.
- Structured logs.
- Event callbacks.
- Mission result summary.

### 6. UGV mission manager
- Receives destination packet.
- Sends ACK.
- Starts motion.
- Exposes drive commands.
- Handles obstacle avoidance hooks.
- Sends deck-ready and motion telemetry.

### 7. ESP32 firmware cleanup
- Rewrite both ESP32 programs into clean production code.
- Preserve protocol compatibility where practical.
- Add heartbeat frames and status LEDs.
- Add compile-time config for peer MAC addresses.
- Add serial diagnostics.

### 8. Testing tools
- Mock video replay for ArUco testing.
- Radio protocol test script.
- Bench `LANDING_TARGET` publisher for SITL.
- Log replay utility.
- Unit tests for protocol and state machines.

### 9. Documentation
- setup guide
- wiring guide
- parameter checklist
- field test checklist
- requirements matrix
- test matrix mapped back to competition requirements

---

## Recommended message definitions
Use dataclasses or pydantic models in Python and packed structs on ESP32.

### Example: destination message
```python
class DestinationFound:
    seq: int
    timestamp_ms: int
    marker_id: int
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    confidence: float
```

### Example: UGV motion state
```python
class UgvTelemetry:
    seq: int
    timestamp_ms: int
    speed_mps: float
    yaw_rad: float
    deck_ready: bool
    estop: bool
    mission_phase: int
```

### Example: landing target observation
```python
class LandingObservation:
    timestamp_ms: int
    marker_id: int
    x_body_m: float
    y_body_m: float
    z_body_m: float
    angle_x_rad: float
    angle_y_rad: float
    confidence: float
    valid: bool
```

---

## Implementation guidance for existing uploaded files

### `mission1.py`
Current status:
- useful for showing rough mission ordering
- not acceptable as final flight-control logic
- uses RC throttle override and placeholder camera input
- should be replaced by state-machine-driven ArduPilot control

### `ground_station.py`
Current status:
- useful as a rough UGV movement and MAVLink control prototype
- obstacle avoidance is placeholder only
- should be refactored into modules with explicit state handling

### `v2v_bridge.py`
Current status:
- useful protocol base
- should be cleaned up, typed, and expanded with formal message schemas

### `esp_uav.cpp` and `esp_ugv.cpp`
Current status:
- useful starting bridge firmware
- should be cleaned up and documented
- preserve the framing logic unless there is a strong reason to change it

---

## Minimum bench-test sequence
The generated project must include scripts and docs for this exact validation order.

### Bench phase 1: radio only
- verify UAV↔UGV heartbeat
- verify command/telemetry round trip
- verify sequence numbers and timeout handling

### Bench phase 2: vision only
- verify destination marker detection distance
- verify landing marker detection distance
- verify off-axis and lighting performance
- verify pose estimator scale correctness

### Bench phase 3: autopilot only
- verify flow health
- verify rangefinder health
- verify stable non-GPS hover near ground
- verify mode changes and arming logic

### Bench phase 4: SITL / dry run
- publish synthetic `LANDING_TARGET`
- verify precision landing state transitions
- verify moving-target handling logic

### Flight phase 1
- stationary landing marker on the ground

### Flight phase 2
- stationary UGV landing deck

### Flight phase 3
- slow-moving UGV landing

### Flight phase 4
- full challenge flow

---

## Wiring / integration notes to document in code comments and setup docs

### Cube Orange+
- ArduCopter installed
- Holybro H-Flow connected via CAN/DroneCAN
- LidarLite v3 connected as downward rangefinder
- Jetson Nano connected via telemetry UART/USB for MAVLink companion link

### Jetson Nano
- ZED X downward camera
- Python runtime for mission manager and vision stack
- serial link to ESP32-S bridge
- MAVLink link to Cube

### ESP32-S
- one unit on UAV side
- one unit on UGV side
- ESP-NOW peer MAC addresses configurable in source or build config

---

## Coding quality requirements
- Python 3.10+
- type hints everywhere
- no giant monolithic scripts
- logging module, not print-only debugging
- all constants moved into config
- no hardcoded magic numbers except protocol constants
- exception-safe cleanup
- every long-running loop has timeout and health checks
- every state transition logged with reason

---

## Deliverable quality bar
The AI must generate code that is:
- modular
- typed
- documented
- bench-testable
- not dependent on GPS for the UAV landing phase
- consistent with the uploaded competition rules
- consistent with ArduPilot precision landing architecture

Do not generate toy pseudocode. Generate real files with runnable scaffolding and clear TODOs where hardware-specific values are needed.

---

## Final instruction to the coding AI
Use the uploaded files as references, but rewrite the system into a clean ArduPilot-first architecture. Preserve the useful radio bridge concept. Replace the RC-override UAV control pattern with proper ArduPilot companion-computer control and `LANDING_TARGET`-based precision landing. Build the project so it can be tested in stages and adjusted quickly on competition day.
