from dataclasses import dataclass
from typing import Any

@dataclass
class set_target:
    """
    Mock implementation of olympe.messages.gimbal.set_target.
    Intercepts gimbal commands to apply them to your Webots/Sphinx simulation.
    """
    gimbal_id: int
    control_mode: Any  # Expects enums.gimbal.control_mode
    yaw_frame_of_reference: Any
    yaw: float
    pitch_frame_of_reference: Any
    pitch: float
    roll_frame_of_reference: Any
    roll: float
    
    # Olympe internal expectation parameters (mocked for compatibility)
    _timeout: int = 10
    _no_expect: bool = False
    _float_tol: tuple = (1e-07, 0.1)
    _int_tol: int = 0
