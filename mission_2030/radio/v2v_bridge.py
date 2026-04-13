import serial
import threading
import time
import struct
from mission_2030.radio.v2v_protocol import (
    SOF, chk_xor,
    TYPE_HEARTBEAT_UAV, TYPE_HEARTBEAT_UGV,
    TYPE_DESTINATION, TYPE_TELEMETRY_UGV,
    FMT_HEARTBEAT, FMT_DESTINATION, FMT_TELEMETRY_UGV
)
from mission_2030.radio.message_types import DestinationFound, UgvTelemetry, UavHeartbeat


class V2VBridge:
    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self._running = False
        self._lock = threading.Lock()
        self._thread = None

        # Latest received state
        self.latest_telemetry: UgvTelemetry | None = None
        self.latest_destination: DestinationFound | None = None
        self.latest_uav_heartbeat: UavHeartbeat | None = None
        self.latest_heartbeat_ugv_time: float = 0.0

    def connect(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.ser:
            self.ser.close()

    def _read_loop(self):
        while self._running:
            try:
                if not self.ser or self.ser.in_waiting == 0:
                    time.sleep(0.001)
                    continue

                b = self.ser.read(1)
                if not b or b[0] != SOF:
                    continue

                hdr = self.ser.read(2)
                if len(hdr) < 2:
                    continue
                f_type, f_len = hdr[0], hdr[1]

                payload = self.ser.read(f_len)
                if len(payload) < f_len:
                    continue

                chk_byte = self.ser.read(1)
                if not chk_byte:
                    continue

                if chk_byte[0] == chk_xor(f_type, f_len, payload):
                    self._handle_packet(f_type, payload)
            except Exception:
                time.sleep(0.01)

    def _handle_packet(self, f_type: int, payload: bytes):
        with self._lock:
            if f_type == TYPE_HEARTBEAT_UGV:
                self.latest_heartbeat_ugv_time = time.time()
                # Also decode the UGV phase from the heartbeat payload
                if len(payload) == struct.calcsize(FMT_HEARTBEAT):
                    unpacked = struct.unpack(FMT_HEARTBEAT, payload)
                    self.latest_telemetry = UgvTelemetry(
                        seq=unpacked[0], timestamp_ms=unpacked[1],
                        speed_mps=0.0, yaw_rad=0.0,
                        deck_ready=True, estop=bool(unpacked[3]),
                        mission_phase=unpacked[2]
                    )

            elif f_type == TYPE_HEARTBEAT_UAV:
                if len(payload) == struct.calcsize(FMT_HEARTBEAT):
                    unpacked = struct.unpack(FMT_HEARTBEAT, payload)
                    self.latest_uav_heartbeat = UavHeartbeat(
                        seq=unpacked[0], timestamp_ms=unpacked[1],
                        mission_phase=unpacked[2], estop=bool(unpacked[3])
                    )

            elif f_type == TYPE_DESTINATION and len(payload) == struct.calcsize(FMT_DESTINATION):
                unpacked = struct.unpack(FMT_DESTINATION, payload)
                self.latest_destination = DestinationFound(
                    seq=unpacked[0], timestamp_ms=unpacked[1], marker_id=unpacked[2],
                    x_m=unpacked[3], y_m=unpacked[4], z_m=unpacked[5],
                    yaw_rad=unpacked[6], confidence=1.0
                )

            elif f_type == TYPE_TELEMETRY_UGV and len(payload) == struct.calcsize(FMT_TELEMETRY_UGV):
                unpacked = struct.unpack(FMT_TELEMETRY_UGV, payload)
                self.latest_telemetry = UgvTelemetry(
                    seq=unpacked[0], timestamp_ms=unpacked[1], speed_mps=unpacked[2],
                    yaw_rad=unpacked[3], deck_ready=bool(unpacked[4] & 1),
                    estop=bool(unpacked[4] & 2), mission_phase=unpacked[5]
                )

    def send_uav_heartbeat(self, seq: int, t_ms: int, phase: int, estop: bool):
        """UAV sends its own heartbeat to UGV."""
        payload = struct.pack(FMT_HEARTBEAT, seq, t_ms, phase, 1 if estop else 0)
        self._transmit(TYPE_HEARTBEAT_UAV, payload)

    def send_ugv_heartbeat(self, seq: int, t_ms: int, phase: int, estop: bool):
        """UGV sends its own heartbeat to UAV."""
        payload = struct.pack(FMT_HEARTBEAT, seq, t_ms, phase, 1 if estop else 0)
        self._transmit(TYPE_HEARTBEAT_UGV, payload)

    def send_destination(self, dest: DestinationFound):
        """UAV sends destination coordinates to UGV."""
        payload = struct.pack(
            FMT_DESTINATION,
            dest.seq, dest.timestamp_ms, dest.marker_id,
            dest.x_m, dest.y_m, dest.z_m, dest.yaw_rad
        )
        self._transmit(TYPE_DESTINATION, payload)

    def _transmit(self, f_type: int, payload: bytes):
        chk = chk_xor(f_type, len(payload), payload)
        frame = bytes([SOF, f_type, len(payload)]) + payload + bytes([chk])
        with self._lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(frame)
                    self.ser.flush()
                except Exception:
                    pass
