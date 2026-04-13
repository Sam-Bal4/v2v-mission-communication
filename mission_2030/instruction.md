# Operation Touchdown: Start-to-Finish Deployment Guide

This guide is written assuming you just ran `git clone` on a completely fresh Jetson Orin (for the UAV) and a fresh Jetson/Raspberry Pi (for the UGV).

## Phase 1: Environment Setup (Do this on BOTH the UAV and UGV)

1. **Clone the repository and enter it:**
   ```bash
   git clone <your-repository-url>
   cd v2v-mission-communication
   ```

2. **Install Python dependencies:**
   *(Ensure you are using Python 3)*
   ```bash
   pip3 install -r mission_2030/requirements.txt
   ```

3. **Install the ZED SDK (UAV ONLY):**
   You must install the official ZED SDK from Stereolabs to get the `pyzed` module required for tracking.
   - Go to: https://www.stereolabs.com/developers/release
   - Download the SDK for Jetson (L4T) and run their installer script.

4. **Set the Python Path (CRUCIAL):**
   Because you are running scripts from inside deeper folders (`uav` / `ugv`), you must tell Python where the `mission_2030` folder is or you will get `ModuleNotFoundError`. 
   
   Make sure you are *inside* the cloned `v2v-mission-communication` folder, and run this exact command:
   ```bash
   export PYTHONPATH=$(pwd)
   ```
   *(Note: You literally type `$(pwd)`. The Linux terminal will automatically replace that snippet with your current location!)*

---

## Phase 2: Hardware Flashing (The ESP32 Radios)
You have two ESP32 chips. One plugs into the UGV via USB, the other into the UAV.

1. **Find the Mac Addresses**: 
   - Flash the default code to both chips, open the Serial Monitor at 115200 baud, and look for "MAC: XX:XX:XX:XX:XX". Write both down.
2. **Flash UGV Bridge**:
   - Open `mission_2030/esp32/ugv_bridge/src/main.cpp`
   - Change `UAV_MAC[6] = {...}` to the MAC address of the **UAV's** ESP32.
   - Upload via PlatformIO.
3. **Flash UAV Bridge**:
   - Open `mission_2030/esp32/uav_bridge/src/main.cpp`
   - Change `UGV_MAC[6] = {...}` to the MAC address of the **UGV's** ESP32.
   - Upload via PlatformIO.

---

## Phase 3: Field Execution

At the field, power everything on and connect your ESP32 bridges via USB to the Jetson/Pi.
*Note: Depending on your Linux setup, you might need `sudo` to access `/dev/ttyUSB0` or `/dev/ttyACM0`.*

### Terminal 1: Starting the UGV (Ground Rover)
SSH into the UGV computer. Navigate to the cloned folder, set the path, and run python natively (no `bash` keyword needed):

```bash
cd v2v-mission-communication
export PYTHONPATH=$(pwd)

# Run the UGV listener
python3 mission_2030/ugv/ugv_runner.py
```
> **What to Expect**: The UGV terminal will print `UGV Base Connection fully established` and `TF-Nova lidar opened`. The rover will **NOT** move yet. It waits silently for the drone to radio down coordinates.


### Terminal 2: Starting the UAV (Hexacopter)
Ensure you are standing completely clear of the Hexa. SSH into the drone's Jetson Orin, set the path, and run your chosen mission:

```bash
cd v2v-mission-communication
export PYTHONPATH=$(pwd)

# Run Mission 3 (Search, Avoidance Relay, and Moving Landing)
python3 mission_2030/uav/mission3_runner.py
```
> **What to Expect**: 
> 1. Terminal verifies Cube Orange connection.
> 2. The drone will automatically ARM props and Takeoff to 1.3 meters.
> 3. It scans with the ZED 2 camera for the Destination ArUco.
> 4. Upon spotting the marker, it radios the UGV. (You will see the UGV terminal wake up and start driving!).
> 5. The drone circles overhead. If you walk in front of the UGV's Lidar, the RED light on the UGV flashes, and the Drone terminal will warn that it is hovering while waiting for the UGV to dodge the obstacle.
> 6. It centers over the target marker and spirals down until the props disarm.
