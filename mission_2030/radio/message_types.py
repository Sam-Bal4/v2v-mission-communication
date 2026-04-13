from pydantic import BaseModel

# Schema definitions for direct V2V Bridge
class DestinationFound(BaseModel):
    seq: int
    timestamp_ms: int
    marker_id: int
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    confidence: float

class UgvTelemetry(BaseModel):
    seq: int
    timestamp_ms: int
    speed_mps: float
    yaw_rad: float
    deck_ready: bool
    estop: bool
    mission_phase: int

class UavHeartbeat(BaseModel):
    seq: int
    timestamp_ms: int
    mission_phase: int
    estop: bool
