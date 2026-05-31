import numpy as np
import time
from scipy.optimize import minimize

class MPCController:
    """
    Kinematic Model Predictive Controller (MPC) with Integral Action.
    Optimizes a 3D world-frame velocity trajectory, then maps it to 
    body-frame roll/pitch/gaz commands using quadrotor differential flatness.
    """
    def __init__(self, dt=0.1, horizon=5, max_vel=5.0):
        self.dt = dt
        self.N = horizon
        self.max_vel = max_vel
        
        # MPC Weights
        self.w_pos = 10.0      
        self.w_int = 2.0       
        self.w_u = 1.0         
        self.w_du = 5.0        # Penalizes acceleration (jerk proxy) for cinematic smoothness
        
        # Internal states
        self.cumulative_error = np.zeros(3)
        self.last_u = np.zeros(3)
        self.last_time = None

    def reset(self):
        self.cumulative_error = np.zeros(3)
        self.last_u = np.zeros(3)
        self.last_time = None

    def _cost_function(self, u_flat, current_pos, target_trajectory):
        u = u_flat.reshape((self.N, 3))
        cost = 0.0
        
        pos = current_pos.copy()
        cum_err = self.cumulative_error.copy()
        prev_u = self.last_u.copy()
        
        for k in range(self.N):
            pos = pos + u[k] * self.dt
            target_pos = target_trajectory[k]
            pos_err = pos - target_pos
            cum_err = cum_err + pos_err * self.dt
            
            cost += self.w_pos * np.sum(pos_err**2)
            cost += self.w_int * np.sum(cum_err**2)
            cost += self.w_u * np.sum(u[k]**2)
            cost += self.w_du * np.sum((u[k] - prev_u)**2)
            
            prev_u = u[k]
            
        return cost

    def compute_control(self, current_pos, target_trajectory, current_yaw_rad, current_time=None):
        """
        current_pos: [x, y, z] in World Frame
        target_trajectory: (N, 3) in World Frame
        current_yaw_rad: Drone's current heading (psi) in radians
        """
        if current_time is None:
            current_time = time.time()
            
        if self.last_time is not None:
            actual_dt = current_time - self.last_time
            self.cumulative_error += (current_pos - target_trajectory[0]) * actual_dt
            
        self.last_time = current_time
        
        if len(target_trajectory) < self.N:
            pad = [target_trajectory[-1]] * (self.N - len(target_trajectory))
            target_trajectory = np.vstack((target_trajectory, pad))
        
        u0 = np.tile(self.last_u, self.N)
        bounds = [(-self.max_vel, self.max_vel)] * (self.N * 3)
        
        # Note: For production, replace SLSQP with OSQP or CasADi for real-time feasibility
        res = minimize(
            self._cost_function,
            u0,
            args=(current_pos, target_trajectory),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 30, 'ftol': 1e-2, 'disp': False}
        )
        
        optimal_u_seq = res.x.reshape((self.N, 3))
        v_world = optimal_u_seq[0] # Desired World-Frame Velocity [vx, vy, vz]
        self.last_u = v_world
        
        # --- RIGOROUS MAPPING TO BODY FRAME & ATTITUDE ---
        return self._map_to_pcmd(v_world, current_yaw_rad)

    def _map_to_pcmd(self, v_world, yaw_rad):
        """
        Maps World-Frame Velocity to Body-Frame PCMD commands using 
        differential flatness principles (Mahony et al., Eq. 30a/30b).
        """
        # 1. Rotate World velocity into Body velocity based on current yaw
        # v_body = R_z(-yaw) * v_world
        cos_psi = np.cos(yaw_rad)
        sin_psi = np.sin(yaw_rad)
        
        vx_body =  cos_psi * v_world[0] + sin_psi * v_world[1]  # Forward/Backward
        vy_body = -sin_psi * v_world[0] + cos_psi * v_world[1]  # Left/Right
        vz_body =  v_world[2]                                   # Up/Down
        
        # 2. Map Body Velocities to PCMD percentages
        # Assuming a P-controller mapping for the low-level API
        # Max physical limits for Parrot Anafi: 15 m/s horizontal, 4 m/s vertical
        MAX_V_XY = 5.0  # Clamped for cinematic smoothness
        MAX_V_Z = 2.0   
        
        # Pitch > 0 is forward, Roll > 0 is right, Gaz > 0 is up
        pitch_pct = int(np.clip((vx_body / MAX_V_XY) * 100, -100, 100))
        roll_pct  = int(np.clip((vy_body / MAX_V_XY) * 100, -100, 100))
        gaz_pct   = int(np.clip((vz_body / MAX_V_Z)  * 100, -100, 100))
        
        return {
            "roll": roll_pct,
            "pitch": pitch_pct,
            "gaz": gaz_pct,
            "v_world": v_world # Return for telemetry/logging
        }
