from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import serial
import struct

# ------------------- SET THESE PORTS -------------------
CUBE_PORT = "/dev/ttyACM0"     # Cube Orange (USB)
CUBE_BAUD = 115200

ESP32_PORT = "/dev/ttyACM1"    # UAV ESP32 (USB)
ESP32_BAUD = 115200

# ------------------- Mission params -------------------
DIST_M = 3.048       # 10 ft
SPEED_MPS = 1.0
MOVE_TIME_S = DIST_M / SPEED_MPS

DO_TAKEOFF = True
TAKEOFF_ALT_M = 2.0
DO_LAND = False

# ------------------- Protocol -------------------
SOF = 0xAA
TYPE_TELEM = 1
TYPE_CMD = 2

TELEM_FMT = "<IIffBB"  # 18 bytes
CMD_FMT   = "<IBB"     # 6 bytes

def chk_xor(type_b, len_b, payload: bytes) -> int:
    c = (type_b ^ len_b) & 0xFF
    for b in payload:
        c ^= b
    return c & 0xFF

def send_telem_frame(ser, seq, t_ms, vx, vy, marker, estop):
    payload = struct.pack(TELEM_FMT, seq, t_ms, vx, vy, marker, estop)
    ln = len(payload)
    chk = chk_xor(TYPE_TELEM, ln, payload)
    ser.write(bytes([SOF, TYPE_TELEM, ln]) + payload + bytes([chk]))

def read_cmd_frame(ser):
    # scan bytes until SOF
    b = ser.read(1)
    if not b:
        return None
    if b[0] != SOF:
        return None

    hdr = ser.read(2)
    if len(hdr) != 2:
        return None
    typ, ln = hdr[0], hdr[1]

    payload = ser.read(ln)
    if len(payload) != ln:
        return None

    chk = ser.read(1)
    if len(chk) != 1:
        return None

    if chk[0] != chk_xor(typ, ln, payload):
        return None

    if typ != TYPE_CMD or ln != struct.calcsize(CMD_FMT):
        return None

    cmdSeq, cmd, estop = struct.unpack(CMD_FMT, payload)
    return cmdSeq, cmd, estop

def send_body_velocity(vehicle, vx, vy, vz, duration_s):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    end_t = time.time() + duration_s
    while time.time() < end_t:
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(0.1)

def arm_and_takeoff(vehicle, target_alt_m):
    while not vehicle.is_armable:
        print("[Cube] Waiting for armable...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(0.2)

    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.2)

    print(f"[Cube] Takeoff to {target_alt_m:.1f} m")
    vehicle.simple_takeoff(target_alt_m)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt >= target_alt_m * 0.95:
            break
        time.sleep(0.3)

def send_cmd_frame(ser, cmdSeq, cmd, estop):
    payload = struct.pack(CMD_FMT, cmdSeq, cmd, estop)
    ln = len(payload)
    chk = chk_xor(TYPE_CMD, ln, payload)
    ser.write(bytes([SOF, TYPE_CMD, ln]) + payload + bytes([chk]))

def main():
    print("[Jetson] Connect Cube:", CUBE_PORT)
    vehicle = connect(CUBE_PORT, baud=CUBE_BAUD, wait_ready=True)

    print("[Jetson] Open UAV ESP32:", ESP32_PORT)
    esp = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=0.05)

    telem_seq = 0
    cmd_seq = 0
    estop_state = 0

    try:
        if DO_TAKEOFF:
            arm_and_takeoff(vehicle, TAKEOFF_ALT_M)

        # REACHED ALTITUDE! Now tell the Ground Vehicle to start its 10ft move
        print("[Mission] Commanding UGV to MOVE 10ft")
        send_cmd_frame(esp, cmd_seq, cmd=5, estop=0) # cmd=5 means 'Move Forward'
        cmd_seq += 1

        print(f"[Mission] Drone Moving Forward {DIST_M:.2f}m")
        start = time.time()

        while True:
            # Optional: read estop commands coming back from Pi
            cmd = read_cmd_frame(esp)
            if cmd:
                cmdSeq, cmdVal, estop = cmd
                estop_state = int(estop)
                if estop_state == 1:
                    print(f"[E-STOP] cmdSeq={cmdSeq} cmd={cmdVal} -> stopping")
                    # immediate stop
                    send_body_velocity(vehicle, 0.0, 0.0, 0.0, 1.0)

            if estop_state == 1:
                # keep sending “stopped” commands
                send_body_velocity(vehicle, 0.0, 0.0, 0.0, 0.2)
            else:
                elapsed = time.time() - start
                if elapsed >= MOVE_TIME_S:
                    break
                send_body_velocity(vehicle, SPEED_MPS, 0.0, 0.0, 0.2)

            # Telemetry from Cube (vehicle.velocity is NED)
            v = vehicle.velocity or (0.0, 0.0, 0.0)
            vx_ned = float(v[0])
            vy_ned = float(v[1])

            t_ms = int(time.time() * 1000) & 0xFFFFFFFF
            marker = 0
            send_telem_frame(esp, telem_seq, t_ms, vx_ned, vy_ned, marker, estop_state)
            telem_seq += 1

        # Stop at end
        print("[Mission] STOP")
        send_body_velocity(vehicle, 0.0, 0.0, 0.0, 2.0)

        # Send a few final “stopped” telemetry frames
        for _ in range(10):
            v = vehicle.velocity or (0.0, 0.0, 0.0)
            t_ms = int(time.time() * 1000) & 0xFFFFFFFF
            send_telem_frame(esp, telem_seq, t_ms, float(v[0]), float(v[1]), 0, estop_state)
            telem_seq += 1
            time.sleep(0.1)

        if DO_LAND:
            print("[Mission] LAND")
            vehicle.mode = VehicleMode("LAND")

        print("[Mission] Done")

    finally:
        try:
            esp.close()
        except:
            pass
        vehicle.close()

if __name__ == "__main__":
    main()