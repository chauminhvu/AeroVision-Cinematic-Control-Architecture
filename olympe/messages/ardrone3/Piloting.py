# olympe/messages/ardrone3/Piloting.py
from dataclasses import dataclass
from typing import Any

@dataclass
class PCMD:
    """Piloting Command (Velocity/Attitude)"""
    flag: int
    roll: int
    pitch: int
    yaw: int
    gaz: int
    psi: float = 0.0

@dataclass
class TakeOff:
    """Initiates drone takeoff sequence."""
    _timeout: int = 10
    _no_expect: bool = False
    _float_tol: tuple = (1e-07, 1e-09)
    _int_tol: int = 0

@dataclass
class Landing:
    """Initiates drone landing sequence."""
    _timeout: int = 10
    _no_expect: bool = False
    _float_tol: tuple = (1e-07, 1e-09)
    _int_tol: int = 0

@dataclass
class Emergency:
    """Emergency motor cut / failsafe trigger."""
    _timeout: int = 10
    _no_expect: bool = False
    _float_tol: tuple = (1e-07, 1e-09)
    _int_tol: int = 0
