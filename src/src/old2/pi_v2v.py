import serial
import struct
import time

SOF = 0xAA
TYPE_TELEM = 1
TYPE_CMD = 2

TELEM_FMT = "<IIffBB"   # 18 bytes
CMD_FMT   = "<IBB"      # 6 bytes

def chk_xor(type_b, len_b, payload: bytes) -> int:
    c = (type_b ^ len_b) & 0xFF
    for b in payload:
        c ^= b
    return c & 0xFF

def read_frame(ser):
    # scan for SOF
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

    return typ, payload

def send_cmd(ser, cmdSeq, cmd, estop):
    payload = struct.pack(CMD_FMT, cmdSeq, cmd, estop)
    ln = len(payload)
    chk = chk_xor(TYPE_CMD, ln, payload)
    ser.write(bytes([SOF, TYPE_CMD, ln]) + payload + bytes([chk]))

def main():
    PORT = "/dev/ttyACM0"  # change if needed
    ser = serial.Serial(PORT, 115200, timeout=0.1)
    print("[Pi] Connected:", PORT)

    last_cmd_time = time.time()
    cmdSeq = 0

    while True:
        frm = read_frame(ser)
        if frm:
            typ, payload = frm
            # 1. Handle Telemetry from the Air
            if typ == TYPE_TELEM and len(payload) == struct.calcsize(TELEM_FMT):
                seq, t_ms, vx, vy, marker, estop = struct.unpack(TELEM_FMT, payload)
                print(f"[TELEM] seq={seq} vx={vx:.2f} vy={vy:.2f} marker={marker} estop={estop}")

            # 2. Handle Commands coming FROM the Air (The Jetson Mission)
            elif typ == TYPE_CMD and len(payload) == struct.calcsize(CMD_FMT):
                cSeq, cVal, eStop = struct.unpack(CMD_FMT, payload)
                if cVal == 5:
                    print("************************************")
                    print("!!! [AIR COMMAND] MOVE 10 FT FRONT !!!")
                    print("************************************")
                    # Here you would call your UGV motor control function!
                elif eStop == 1:
                    print("[E-STOP] Emergency stop triggered from air!")


        time.sleep(0.01)

if __name__ == "__main__":
    main()