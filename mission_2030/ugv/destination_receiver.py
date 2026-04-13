from mission_2030.radio.v2v_bridge import V2VBridge

class DestinationReceiver:
    def __init__(self, bridge: V2VBridge):
        self.bridge = bridge

    def get_destination(self):
        """ Polls bridge for new coordinate payloads """
        if self.bridge.latest_destination is not None:
             dest = self.bridge.latest_destination
             self.bridge.latest_destination = None # consume
             return dest
        return None
