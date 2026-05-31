from dotenv import load_dotenv
load_dotenv()
import cv2
import numpy as np
import math
from controller import Robot

def clamp(value, low, high):
    return max(low, min(value, high))

class MockExpectation:
    def wait(self, _timeout=None):
        pass

class Drone:
    def __init__(self, ip_address):
        self.ip_address = ip_address
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)
        
        self.camera_pitch_motor = self.robot.getDevice("camera pitch")
        self.current_gimbal_pitch = 0.0
        
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
        
        motor_names = ["front left propeller", "front right propeller", "rear left propeller", "rear right propeller"]
        self.motors = []
        for name in motor_names:
            motor = self.robot.getDevice(name)
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)
            self.motors.append(motor)
            
        self.target_altitude = 1.0
        self.target_yaw_velocity = 0.0
        self.target_pitch = 0.0
        self.target_roll = 0.0
        
        self.k_vertical_thrust = 68.5
        self.k_vertical_offset = 0.6
        self.k_vertical_p = 3.0
        self.k_roll_p = 50.0
        self.k_pitch_p = 30.0

    def connect(self):
        print(f"[Shadow SDK] Emulating connection to ANAFI at {self.ip_address}")
        for _ in range(10):
            self.robot.step(self.timestep)
        self.target_altitude = self.gps.getValues()[2]
        self.set_gimbal_pitch(0.0)
        return True

    def start_video_streaming(self):
        print("[Shadow SDK] Connected to virtual camera pipe")

    def set_gimbal_pitch(self, angle_rad):
        if self.camera_pitch_motor:
            # WEBOTS INVERSION: Olympe negative (DOWN) -> Webots positive (DOWN)
            webots_rad = -angle_rad
            self.current_gimbal_pitch = clamp(webots_rad, -1.57, 1.57)
            self.camera_pitch_motor.setPosition(self.current_gimbal_pitch)

    def get_position(self):
        return self.gps.getValues()

    def get_next_frame(self):
        if self.robot.step(self.timestep) == -1:
            return None
        self._stabilize()
        image_data = self.camera.getImage()
        if not image_data:
            return None
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        frame = np.frombuffer(image_data, np.uint8).reshape((height, width, 4))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame

    def _stabilize(self):
        roll, pitch, _ = self.imu.getRollPitchYaw()
        altitude = self.gps.getValues()[2]
        roll_velocity, pitch_velocity, _ = self.gyro.getValues()

        roll_input = self.k_roll_p * clamp(roll, -1.0, 1.0) + roll_velocity + self.target_roll
        pitch_input = self.k_pitch_p * clamp(pitch, -1.0, 1.0) + pitch_velocity + self.target_pitch
        yaw_input = self.target_yaw_velocity

        clamped_diff_altitude = clamp(self.target_altitude - altitude + self.k_vertical_offset, -1.0, 1.0)
        vertical_input = self.k_vertical_p * (clamped_diff_altitude ** 3)

        front_left = self.k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
        front_right = self.k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
        rear_left = self.k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
        rear_right = self.k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input

        self.motors[0].setVelocity(front_left)
        self.motors[1].setVelocity(-front_right)
        self.motors[2].setVelocity(-rear_left)
        self.motors[3].setVelocity(rear_right)

    def __call__(self, message):
        msg_name = message.__class__.__name__
        
        if msg_name == "PCMD":
            self.target_roll = (message.roll / 100.0)
            self.target_pitch = -(message.pitch / 100.0) * 2.0
            self.target_yaw_velocity = -(message.yaw / 100.0) * 1.3
            gaz_bias = (message.gaz / 100.0) * 0.05
            self.target_altitude += gaz_bias

        elif msg_name == "TakeOff":
            print("[Shadow SDK] TakeOff initiated. Ascending to 1.5m...")
            self.target_altitude = 1.5

        elif msg_name == "Landing":
            print("[Shadow SDK] Landing initiated. Descending to ground...")
            self.target_altitude = 0.1

        elif msg_name == "Emergency":
            print("[Shadow SDK] EMERGENCY STOP! Cutting motors.")
            for motor in self.motors:
                motor.setVelocity(0.0)

        elif msg_name == "set_target":
            # ROUTER: Catches the gimbal command and applies Webots physics
            if hasattr(message, 'pitch') and message.pitch is not None:
                pitch_rad = math.radians(message.pitch)
                self.set_gimbal_pitch(pitch_rad)
                
        return MockExpectation()
