import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pid import PID

@dataclass
class DroneCommand:
    """
    Standardized command output for both Olympe (Physical) and Webots (Sim).
    Velocities are in m/s, angles in degrees.
    """
    vx: float = 0.0           # Forward velocity (m/s)
    vy: float = 0.0           # Lateral velocity (m/s)
    vz: float = 0.0           # Vertical velocity (m/s)
    yaw_rate: float = 0.0     # Yaw rate (deg/s)
    gimbal_pitch: float = 0.0 # Gimbal pitch (deg)
    status: str = "EXECUTING"

class Maneuver(ABC):
    @abstractmethod
    def execute(self, target_data, drone_state=None) -> DroneCommand:
        pass

class HoverManeuver(Maneuver):
    """A safe default that simply holds the drone perfectly steady."""
    def execute(self, target_data, drone_state=None) -> DroneCommand:
        return DroneCommand(status="HOVERING")

class CinematicApproachManeuver(Maneuver):
    """
    Approaches the target using IBVS (Image-Based Visual Servoing).
    Maintains target area (distance) and centering (yaw/gimbal).
    """
    def __init__(self, desired_area=0.15, max_vx=2.0, max_yaw_rate=40.0):
        self.state = "APPROACHING"
        self.desired_area = desired_area
        
        # PIDs operate on normalized image coordinates [-0.5, 0.5]
        # Setpoint is 0.0 (center of the image)
        self.yaw_pid = PID(kp=80.0, ki=10.0, kd=20.0, setpoint=0.0, output_limits=(-max_yaw_rate, max_yaw_rate))
        self.gimbal_pid = PID(kp=40.0, ki=5.0, kd=10.0, setpoint=0.0, output_limits=(-45, 45))
        self.distance_pid = PID(kp=5.0, ki=0.5, kd=1.0, setpoint=0.0, output_limits=(-max_vx, max_vx))

    def execute(self, target_data, drone_state=None) -> DroneCommand:
        current_time = time.time()
        cmd = DroneCommand(status=self.state)

        if not target_data.found:
            cmd.status = "SEARCHING"
            return cmd

        # Error from center (0.5 is center of normalized xywhn)
        err_x = float(target_data.xywhn[0]) - 0.5
        err_y = float(target_data.xywhn[1]) - 0.5
        err_area = self.desired_area - target_data.current_area

        if target_data.current_area >= self.desired_area * 0.95:
            cmd.status = "FINISHED"
            return cmd

        cmd.yaw_rate = self.yaw_pid(err_x, current_time)
        cmd.gimbal_pitch = self.gimbal_pid(err_y, current_time)
        cmd.vx = self.distance_pid(err_area, current_time)
        cmd.vz = 0.0  # Hold altitude

        return cmd

class HelixRevealManeuver(Maneuver):
    """
    Cinematic spiral-out dronie.
    Moves backward and upward while slowly orbiting, keeping the target framed.
    """
    def __init__(self, duration=15.0, max_vx=-2.0, max_vz=1.5, orbit_speed=1.0, max_yaw_rate=40.0):
        self.state = "HELIX_REVEAL"
        self.start_time = None
        self.duration = duration
        self.max_vx = max_vx  
        self.max_vz = max_vz
        self.orbit_speed = orbit_speed
        
        self.yaw_pid = PID(kp=80.0, ki=15.0, kd=25.0, setpoint=0.0, output_limits=(-max_yaw_rate, max_yaw_rate))
        self.gimbal_pid = PID(kp=40.0, ki=5.0, kd=10.0, setpoint=0.0, output_limits=(-45, 45))

    def _smoothstep(self, t):
        """Minimum-jerk inspired smoothstep for cinematic acceleration."""
        t = max(0.0, min(1.0, t))
        return t * t * (3.0 - 2.0 * t)

    def execute(self, target_data, drone_state=None) -> DroneCommand:
        current_time = time.time()
        cmd = DroneCommand(status=self.state)

        if not target_data.found:
            cmd.status = "LOST_TARGET"
            return cmd

        if self.start_time is None:
            self.start_time = current_time

        t = current_time - self.start_time
        progress = t / self.duration

        if progress >= 1.0:
            cmd.status = "FINISHED"
            return cmd

        err_x = float(target_data.xywhn[0]) - 0.5
        err_y = float(target_data.xywhn[1]) - 0.5
        
        cmd.yaw_rate = self.yaw_pid(err_x, current_time)
        cmd.gimbal_pitch = self.gimbal_pid(err_y, current_time)

        # Cinematic velocity profile (Ramp up, hold, ramp down)
        if progress < 0.2:
            profile = self._smoothstep(progress / 0.2)
        elif progress > 0.8:
            profile = self._smoothstep((1.0 - progress) / 0.2)
        else:
            profile = 1.0

        cmd.vx = self.max_vx * profile  
        cmd.vz = self.max_vz * profile  
        cmd.vy = self.orbit_speed * profile 

        return cmd

class ParallaxOrbitManeuver(Maneuver):
    """
    Continuous circular orbit around the target.
    Achieved by maintaining a constant distance (via image area) while 
    applying a constant lateral strafe.
    """
    def __init__(self, desired_area=0.10, orbit_speed=1.5, max_vx=2.0, max_yaw_rate=40.0):
        self.state = "ORBITING"
        self.desired_area = desired_area
        self.orbit_speed = orbit_speed 
        
        self.yaw_pid = PID(kp=80.0, ki=15.0, kd=25.0, setpoint=0.0, output_limits=(-max_yaw_rate, max_yaw_rate))
        self.gimbal_pid = PID(kp=40.0, ki=5.0, kd=10.0, setpoint=0.0, output_limits=(-45, 45))
        self.distance_pid = PID(kp=5.0, ki=0.5, kd=1.0, setpoint=0.0, output_limits=(-max_vx, max_vx))

    def execute(self, target_data, drone_state=None) -> DroneCommand:
        current_time = time.time()
        cmd = DroneCommand(status=self.state)

        if not target_data.found:
            cmd.status = "LOST_TARGET"
            return cmd

        err_x = float(target_data.xywhn[0]) - 0.5
        err_y = float(target_data.xywhn[1]) - 0.5
        err_area = self.desired_area - target_data.current_area

        cmd.yaw_rate = self.yaw_pid(err_x, current_time)
        cmd.gimbal_pitch = self.gimbal_pid(err_y, current_time)
        cmd.vx = self.distance_pid(err_area, current_time)
        cmd.vy = self.orbit_speed  # Constant strafe creates the orbit
        cmd.vz = 0.0

        return cmd
