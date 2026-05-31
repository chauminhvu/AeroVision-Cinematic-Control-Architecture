import time
import logging
from maneuvers import DroneCommand, CinematicApproachManeuver, HelixRevealManeuver, ParallaxOrbitManeuver

class CinemaDirector:
    def __init__(self):
        self.phase = "SCOUTING"
        self.shot_list = [
            CinematicApproachManeuver(desired_area=0.15),
            HelixRevealManeuver(),
            ParallaxOrbitManeuver()
        ]
        self.active_maneuver = None
        self.framing_start_time = None
        self.reacquire_start_time = None
        self.scan_start_time = None
        self.last_known_pitch = 0.0
        
    def update(self, target_data, frame_h, frame_w):
        current_time = time.time()
        
        if self.phase == "SCOUTING":
            if self.scan_start_time is None:
                self.scan_start_time = current_time
                
            if target_data.found:
                self.phase = "FRAMING"
                self.framing_start_time = current_time
                self.scan_start_time = None
            else:
                elapsed = current_time - self.scan_start_time
                cycle_time = elapsed % 8.0
                yaw_cmd = 15.0 if cycle_time < 2.0 else (-15.0 if cycle_time < 6.0 else 15.0)
                return DroneCommand(vx=0, vy=0, vz=0, yaw_rate=yaw_cmd, gimbal_pitch=0.0, status="SCOUTING"), "SCOUTING", 0.0

        if self.phase == "FRAMING":
            if not target_data.found:
                self.phase = "REACQUIRE"
                self.reacquire_start_time = current_time
                return None, "LOST", self.last_known_pitch
                
            bbox = target_data.bbox
            cx = (bbox[0] + bbox[2]) / 2
            cy = (bbox[1] + bbox[3]) / 2
            area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1]) / (frame_h * frame_w)
            
            error_x = (cx - frame_w / 2) / (frame_w / 2) 
            yaw_cmd = error_x * 20.0 
            
            # 🚨 VISUAL GIMBAL TRACKING 🚨
            # error_y is -1.0 (top of screen) to 1.0 (bottom of screen)
            error_y = (cy - frame_h / 2) / (frame_h / 2)
            # If car is low in frame (error_y > 0), tilt down (more negative).
            target_pitch = -15.0 - (error_y * 30.0) 
            target_pitch = max(-60.0, min(10.0, target_pitch))
            self.last_known_pitch = target_pitch
            
            vz_cmd = 1.0 if area > 0.05 else 0.0 
            is_framed = (area < 0.05) and (abs(error_x) < 0.1)
            
            if is_framed or (current_time - self.framing_start_time > 10.0):
                self.phase = "ACTION"
                self.active_maneuver = self.shot_list.pop(0)
            
            framing_cmd = DroneCommand(vx=0, vy=0, vz=vz_cmd, yaw_rate=yaw_cmd, gimbal_pitch=target_pitch, status="FRAMING")
            return framing_cmd, "FRAMING", target_pitch

        if self.phase == "ACTION":
            if not target_data.found:
                self.phase = "REACQUIRE"
                self.reacquire_start_time = current_time
                return None, "LOST", self.last_known_pitch
                
            cmd = self.active_maneuver.execute(target_data)
            if cmd.status == "FINISHED":
                if self.shot_list:
                    self.active_maneuver = self.shot_list.pop(0)
                else:
                    self.phase = "WRAP"
            self.last_known_pitch = cmd.gimbal_pitch
            return cmd, f"ACTION ({type(self.active_maneuver).__name__})", cmd.gimbal_pitch

        if self.phase == "REACQUIRE":
            if target_data.found:
                self.phase = "FRAMING" 
                self.framing_start_time = current_time
                return None, "REACQUIRED", self.last_known_pitch
                
            if current_time - self.reacquire_start_time > 5.0:
                self.phase = "SCOUTING"
                self.scan_start_time = None
                return None, "SEARCHING", 0.0
                
            return DroneCommand(vx=0, vy=0, vz=0, yaw_rate=0.0, gimbal_pitch=self.last_known_pitch, status="REACQUIRE"), "REACQUIRE", self.last_known_pitch

        return DroneCommand(vx=0, vy=0, vz=0, yaw_rate=0, gimbal_pitch=0, status="HOVER"), "WRAP", 0.0
    