# Challenge 1 - Precision Landing on Moving UGV

This directory contains standalone, fully working code to execute **Challenge 1** as perfectly described. The system relies entirely on local stability (Optical Flow + Lidar) and ArduPilot's native `LANDING_TARGET` functionality utilizing MAVLink.

## 1. ArduPilot Configuration (Mission Planner)
You **must** configure these settings on your Cube Orange+ for the drone before flying.

### Local Navigation Stack (EK3)
This ensures the drone uses Optical Flow + Lidar for high-precision local flight, while GPS is only kept around for the geofence and global fallback.
* `EK3_SRC1_POSXY` = 3 (GPS)
* `EK3_SRC1_VELXY` = 5 (Optical Flow)
* `EK3_SRC1_POSZ` = 2 (Lidar Height / Rangefinder)
* `EK3_SRC1_VELZ` = 0 (None)
* `EK3_SRC1_YAW` = 1 (Compass)

### Precision Landing Parameters (PLND)
This explicitly enables moving-target landing.
* `PLND_ENABLED` = 1
* `PLND_TYPE` = 1 (MAVLink LANDING_TARGET)
* `PLND_OPTIONS` = 1 (Bit 0 = Moving Target)
* `PLND_STRICT` = 2 (Hover if target is lost)
* `PLND_XY_DIST_MAX` = 0.5 (Don't descend unless within 50cm of the center marker)
* `PLND_ALT_MAX` = 2.5 (Below this altitude, abort descending if no marker found)
* `PLND_ALT_MIN` = 0.3 (Below 30cm, just drop blind to secure the landing)
* `PLND_RET_MAX` = 3
* `LAND_SPEED` = 30 (30 cm/s approach speed)

## 2. Hardware Setup

### UGV (Rover)
Print two ArUco markers and attach them to the UGV's landing pad:
* **Large Marker**: ID `10`, printed exactly `50 cm` wide.
* **Small Marker**: ID `20`, printed exactly `16 cm` wide, glued perfectly in the center of the large marker.

### UAV (Drone)
* Point the **ZED X** camera strictly downwards.
* Set the Lidar Lite V3 pointing strictly downwards.
* Ensure you calibrate your compass in the field.

## 3. Running the Challenge

1. **Start the UGV:**
   SSH into the UGV's Raspberry Pi and run:
   ```bash
   python3 challenge1/ugv/moving_platform.py
   ```
   *The UGV will arm and drive forward constantly, stopping automatically if its Lidar sees a physical obstacle.*

2. **Start the UAV Precision Lander:**
   SSH into the UAV's Jetson/Companion computer and run:
   ```bash
   python3 challenge1/uav/precision_land.py
   ```
   *The script will takeoff to 2.0 meters, search for the markers, automatically enter `PRECISION LOITER`, and when perfectly aligned, switch to `LAND` mode.*
