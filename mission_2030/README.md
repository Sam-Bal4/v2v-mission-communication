# Operation Touchdown - ArduPilot UAV/UGV Cooperative System

This repository contains the competition-ready stack for the Raytheon AVC "Operation Touchdown".

## Features
- Fully autonomous UAV / UGV control without GPS.
- V2V direct serial bridge telemetry and messaging.
- Precision landing based on MAVLink `LANDING_TARGET` tracking via ZED X and ArUco markers.
- Platform IO ESP32 bridge links.

## Architecture
- `uav/`: Core drone state machine, MAVLink proxy wrapper, ArUco vision stack.
- `ugv/`: Ground vehicle destination handler and navigation states.
- `radio/`: Shared V2V direct messaging structure.
- `esp32/`: Low latency ESP-NOW link source codes.
- `config/`: System YAML configuration definitions.
