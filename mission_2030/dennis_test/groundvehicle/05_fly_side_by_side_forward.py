import time
from dronekit import connect, VehicleMode

def build_attitude_msg(vehicle, throttle, yaw_rate=0.0):
    return vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0, 0xA3, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, yaw_rate, throttle
    )

def main():
    print("--- Test 5: Pacing Forward UGV ---")
    vehicle = connect("/dev/ttyACM0", wait_ready=True, baud=115200)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(5) # Wait for drone takeoff

    print("Driving Forward for pacing test...")
    start_t = time.time()
    while time.time() - start_t < 10:
        msg = build_attitude_msg(vehicle, 1.0)
        vehicle.send_mavlink(msg)
        time.sleep(0.1)
        
    print("Stop.")
    for _ in range(5): vehicle.send_mavlink(build_attitude_msg(vehicle, 0.0))
    vehicle.armed = False
    vehicle.close()

if __name__ == "__main__":
    main()
