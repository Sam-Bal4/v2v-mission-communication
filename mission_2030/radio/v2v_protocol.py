import struct

SOF = 0xAA
TYPE_HEARTBEAT_UAV = 0x10
TYPE_HEARTBEAT_UGV = 0x11
TYPE_DESTINATION   = 0x20
TYPE_TELEMETRY_UGV = 0x30
TYPE_MSG           = 0x40

# Formats using struct
FMT_HEARTBEAT = "<IIBB"           # seq, timestamp_ms, mission_phase, estop
FMT_DESTINATION = "<IIiffff"      # seq, timestamp_ms, marker_id, x_m, y_m, z_m, yaw_rad
FMT_TELEMETRY_UGV = "<IIffBB"     # seq, timestamp_ms, speed_mps, yaw_rad, deck_ready, mission_phase (estop combined)

def chk_xor(type_b: int, len_b: int, payload: bytes) -> int:
    c = (type_b ^ len_b) & 0xFF
    for b in payload:
        c ^= b
    return c & 0xFF
