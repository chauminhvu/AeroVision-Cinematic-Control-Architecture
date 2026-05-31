import time

class PID:
    """
    A robust discrete-time PID controller with integral anti-windup and derivative filtering.
    Used for keeping the camera orientation (Yaw) and gimbal perfectly locked on the target.
    """
    def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(None, None), 
                 integral_limits=(None, None), alpha=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.integral_limits = integral_limits
        
        # Derivative low-pass filter coefficient (0.0 to 1.0)
        # 1.0 = no filtering, closer to 0.0 = heavy filtering (smooths jitter)
        self.alpha = alpha 

        self._integral = 0.0
        self._last_error = 0.0
        self._last_derivative = 0.0
        self._last_time = None

    def reset(self):
        self._integral = 0.0
        self._last_error = 0.0
        self._last_derivative = 0.0
        self._last_time = None

    def __call__(self, measurement, current_time=None):
        if current_time is None:
            current_time = time.time()
            
        if self._last_time is None:
            self._last_time = current_time
            self._last_error = self.setpoint - measurement
            return 0.0  # Cannot compute dt on first tick
            
        dt = current_time - self._last_time
        if dt <= 0.0:
            return 0.0
            
        error = self.setpoint - measurement

        # Proportional term
        P = self.kp * error

        # Integral term with clamping (Anti-Windup)
        self._integral += error * dt
        if self.integral_limits[0] is not None:
            self._integral = max(self.integral_limits[0], self._integral)
        if self.integral_limits[1] is not None:
            self._integral = min(self.integral_limits[1], self._integral)
            
        I = self.ki * self._integral

        # Derivative term with low-pass filtering
        raw_derivative = (error - self._last_error) / dt
        # EMA filter for derivative to prevent noise spikes from camera jitter
        self._last_derivative = self.alpha * raw_derivative + (1 - self.alpha) * self._last_derivative
        D = self.kd * self._last_derivative

        # Compute raw output
        output = P + I + D

        # Apply output limits
        if self.output_limits[0] is not None:
            output = max(self.output_limits[0], output)
        if self.output_limits[1] is not None:
            output = min(self.output_limits[1], output)

        # Store states for next iteration
        self._last_error = error
        self._last_time = current_time

        return output
