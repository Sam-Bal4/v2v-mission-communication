"""
Test 11 - UGV Arm & Obstacle Avoidance (UAV side = radio commander only)
=========================================================================
No drone arming or flight. This script simply opens the ESP32 V2V bridge
and broadcasts phase=11 so the UGV knows to arm and start driving.
After 60 seconds it sends phase=0 (stop) and exits cleanly.

Run from repo root:
  export PYTHONPATH=$(pwd)
  python3 mission_2030/dennis_test/drone/11_arm_ugv_avoid.py
"""
import time
import signal
import sys
import os

sys.path.append(os.path.abspath("../../"))
from mission_2030.radio.v2v_bridge import V2VBridge

ESP32_PORT = "/dev/ttyUSB0"
BROADCAST_DURATION_S = 60       # how long to tell UGV to keep driving

_abort = False
def _sigint(sig, frame):
    global _abort
    _abort = True
signal.signal(signal.SIGINT,  _sigint)
signal.signal(signal.SIGTERM, _sigint)

def main():
    print("=" * 50)
    print("  TEST 11 - UGV ARM & AVOID (Radio Commander)")
    print("  Ctrl+C -> sends stop to UGV and exits cleanly")
    print("=" * 50)

    bridge = V2VBridge(ESP32_PORT)
    bridge.connect()
    print("ESP32 bridge connected [OK]")

    print(f"Broadcasting Phase 11 (UGV: Arm & Drive) for {BROADCAST_DURATION_S}s...")
    seq = 0
    start_t = time.time()
    try:
        while not _abort and (time.time() - start_t) < BROADCAST_DURATION_S:
            bridge.send_uav_heartbeat(seq, (int(time.time() * 1000) & 0xFFFFFFFF), 11, False)
            seq += 1
            elapsed = time.time() - start_t
            print(f"\r  Driving... {elapsed:.0f}s / {BROADCAST_DURATION_S}s", end="", flush=True)
            time.sleep(0.5)
    finally:
        print("\nSending STOP signal to UGV (phase=0)...")
        for _ in range(5):          # blast stop 5x for reliability
            bridge.send_uav_heartbeat(seq, (int(time.time() * 1000) & 0xFFFFFFFF), 0, False)
            time.sleep(0.1)

        bridge.stop()
        print("Test 11 UAV-side complete [OK]")

if __name__ == "__main__":
    main()
