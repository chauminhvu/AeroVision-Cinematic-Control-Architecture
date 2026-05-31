from enum import Enum

class control_mode(Enum):
    """Gimbal control mode."""
    position = 0  # Attitude is set by giving a position (degrees)
    velocity = 1  # Attitude is set by giving a velocity

class frame_of_reference(Enum):
    """Frame of reference for gimbal axes."""
    none = 0      # None, references are ignored
    relative = 1  # Drone frame of reference
    absolute = 2  # NED (North-East-Down) frame of reference
