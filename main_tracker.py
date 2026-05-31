import cv2
import numpy as np
import time
import logging
import argparse  # Added for --debug flag
import olympe
from olympe.messages.ardrone3.Piloting import PCMD, TakeOff, Landing
from olympe.messages.gimbal import set_target
from olympe.enums.gimbal import control_mode, frame_of_reference
from detector import TargetDetector
from cinema_director import CinemaDirector

# ==========================================
# Argument Parsing
# ==========================================
parser = argparse.ArgumentParser()
parser.add_argument("--debug", action="store_true", help="Enable debug mode (fast speed, debug bbox style)")
args = parser.parse_args()
DEBUG_MODE = args.debug

# ==========================================
# 1. Logging & Hardware Limits
# ==========================================
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s', datefmt='%H:%M:%S')

ANAFI_MIN_PITCH = -90.0
ANAFI_MAX_PITCH = 90.0

# Dynamic Configuration based on --debug
if DEBUG_MODE:
    MAX_PITCH_PCT = 100  # Full tilt for aggressive approach
    ALPHA = 1.0          # No smoothing: Instant reaction to target movement
    print("[DEBUG MODE ENABLED] Aggressive speed and instant response active.")
else:
    MAX_PITCH_PCT = 80
    ALPHA = 0.10         # Cinematic smoothing: Slow, steady movements

MAX_ROLL_PCT = 20
MAX_YAW_PCT = 15
MAX_GAZ_PCT = 30

filtered_cmds = {'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0, 'gaz': 0.0}

# Rate limiter state
current_physical_pitch = 0.0

# ==========================================
# 2. Initialization
# ==========================================
drone = olympe.Drone("192.168.42.1")
detector = TargetDetector(model_name="yolo11n.pt", target_class_name="car")
director = CinemaDirector()

drone.connect()
drone.start_video_streaming()
time.sleep(2)

logging.info("Executing TakeOff...")
try: drone(TakeOff()).wait()
except AttributeError: drone(TakeOff())
time.sleep(3)

def get_smooth_pcmd(vx, vy, vz, yaw_rate):
    # Map velocities to percentages with current limits
    t_pitch = np.clip((-vx / 3.0) * MAX_PITCH_PCT, -MAX_PITCH_PCT, MAX_PITCH_PCT)
    t_roll  = np.clip((vy / 3.0) * MAX_ROLL_PCT, -MAX_ROLL_PCT, MAX_ROLL_PCT)
    t_yaw   = np.clip((-yaw_rate / 50.0) * MAX_YAW_PCT, -MAX_YAW_PCT, MAX_YAW_PCT)
    t_gaz   = np.clip((vz / 2.0) * MAX_GAZ_PCT, -MAX_GAZ_PCT, MAX_GAZ_PCT)
    
    # Apply smoothing filter
    filtered_cmds['pitch'] = (ALPHA * t_pitch) + ((1 - ALPHA) * filtered_cmds['pitch'])
    filtered_cmds['roll']  = (ALPHA * t_roll)  + ((1 - ALPHA) * filtered_cmds['roll'])
    filtered_cmds['yaw']   = (ALPHA * t_yaw)   + ((1 - ALPHA) * filtered_cmds['yaw'])
    filtered_cmds['gaz']   = (ALPHA * t_gaz)   + ((1 - ALPHA) * filtered_cmds['gaz'])
    
    p = int(filtered_cmds['pitch']) if abs(filtered_cmds['pitch']) > 3 else 0
    r = int(filtered_cmds['roll'])  if abs(filtered_cmds['roll']) > 3 else 0
    y = int(filtered_cmds['yaw'])   if abs(filtered_cmds['yaw']) > 3 else 0
    g = int(filtered_cmds['gaz'])   if abs(filtered_cmds['gaz']) > 3 else 0
    
    return PCMD(1, r, p, y, g, 0.0)

def send_gimbal_command(target_pitch_deg: float):
    """Rate-limits gimbal movement to prevent Webots motor vibration."""
    global current_physical_pitch
    
    # Rate limiter: Max 2 degrees per frame (unless debug mode, then faster)
    rate_limit = 10.0 if DEBUG_MODE else 2.0
    step = np.clip(target_pitch_deg - current_physical_pitch, -rate_limit, rate_limit)
    current_physical_pitch += step
    
    clamped_deg = max(ANAFI_MIN_PITCH, min(ANAFI_MAX_PITCH, current_physical_pitch))
    
    drone(set_target(
        gimbal_id=0, control_mode=control_mode.position,
        yaw_frame_of_reference=frame_of_reference.none, yaw=0.0,
        pitch_frame_of_reference=frame_of_reference.absolute, pitch=clamped_deg,
        roll_frame_of_reference=frame_of_reference.none, roll=0.0
    ))

# ==========================================
# 3. Main Loop
# ==========================================
try:
    while director.phase != "WRAP":
        frame = drone.get_next_frame()
        if frame is None: continue
        h, w, _ = frame.shape
        
        target_data = detector.find_target(frame)
        cmd, state_text, gimbal_target = director.update(target_data, h, w)
        
        # Execute Commands
        if cmd:
            pcmd = get_smooth_pcmd(cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate)
            drone(pcmd)
            send_gimbal_command(gimbal_target)
        else:
            drone(PCMD(1, 0, 0, 0, 0, 0.0))
            send_gimbal_command(gimbal_target)
            
        # ==========================================
        # Visual Overlays
        # ==========================================
        
        # 1. Debug Bounding Box (Matches your reference image)
        if target_data.found and DEBUG_MODE:
            bbox = target_data.bbox
            x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
            
            # Draw Red Rectangle
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
            
            # Draw Label (Car + Confidence)
            conf_str = f"{int(target_data.confidence * 100)}%" if hasattr(target_data, 'confidence') else ""
            label = f"car {conf_str}"
            # Text positioned at top-left of the box
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        
        # 2. State HUD Text
        cv2.putText(frame, f"[REC] {state_text}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        cv2.imshow("Director Viewport", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'): break

except KeyboardInterrupt:
    logging.info("Cut! Shoot aborted.")
finally:
    logging.info("Wrapping production. Landing drone.")
    drone(PCMD(1, 0, 0, 0, 0, 0.0))
    try: drone(Landing()).wait()
    except AttributeError: drone(Landing())
    drone.disconnect()
    cv2.destroyAllWindows()
