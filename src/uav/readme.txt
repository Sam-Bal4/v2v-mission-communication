from v2v_bridge import V2VBridge

# 1. Start it
bridge = V2VBridge("/dev/ttyACM1")
bridge.connect()

# 2. Use it
bridge.send_command(cmdSeq=1, cmd=5, estop=0) # Tell ground to move
data = bridge.get_telemetry() # Get latest status


//template for drone mission codes.
