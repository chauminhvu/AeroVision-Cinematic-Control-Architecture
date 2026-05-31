# Cinematic Drone Control System

## Abstract
This repository contains the control architecture for an autonomous, vision-based cinematic camera system deployed on the Parrot Anafi platform. The system utilizes Image-Based Visual Servoing (IBVS) and a macro-state flight director to execute automated tracking maneuvers. The control logic is hardware-agnostic, interfacing with physical hardware via the Parrot Olympe SDK and with the Webots robotics simulator via a custom Shadow SDK translation layer.

## System Architecture
The control stack is decoupled into three primary layers to ensure stability and cinematic output quality:

1.  **Macro-State Director (`cinema_director.py`)**: Governs the high-level operational sequence, managing state transitions between Scouting (horizon sweep), Framing (visual centering and altitude adjustment), and Action (maneuver execution).
2.  **Maneuver Generation (`maneuvers.py`)**: Computes hardware-agnostic physical velocity vectors (m/s, deg/s) and gimbal telemetry based on bounding-box error metrics.
3.  **Hardware Abstraction & Filtering (`main_tracker.py`, `olympe/`)**: Translates physical vectors into PCMD actuator percentages. This layer applies Exponential Moving Average (EMA) low-pass filters to ensure kinematic smoothness and enforces strict rate-limiting on gimbal actuators to prevent mechanical oscillation.

## Operational Status and Cautions

| Maneuver | Status | Technical Notes |
| :--- | :--- | :--- |
| **Cinematic Approach** | Operational | Fully tuned. Maintains target lock and executes smooth forward translation. |
| **Helix Reveal** | Unverified | Logic implemented. Currently exhibits instability during execution due to uncalibrated yaw/pitch coupling and gimbal tracking latency. |
| **Parallax Orbit** | Unverified | Logic implemented. Requires radius and tangential velocity calibration. |

### CAUTION: Execution Limitations
At the current stage of development, **only the Cinematic Approach maneuver is stable and verified for execution.** 

While the macro-state director is architected to sequence multiple maneuvers, transitioning to the Helix Reveal or Parallax Orbit states will likely result in visual servoing failure, target loss, and unpredictable kinematic behavior. 

**Mandatory Configuration for Stable Operation:**
To prevent erratic flight behavior during testing, the shot list within `cinema_director.py` must be restricted exclusively to the `CinematicApproachManeuver` until the rotational and orbital kinematic models are fully calibrated.

```python
# cinema_director.py - Recommended configuration for stable testing
self.shot_list = [
    CinematicApproachManeuver(desired_area=0.15)
    # HelixRevealManeuver(),    # Disabled pending calibration
    # ParallaxOrbitManeuver()   # Disabled pending calibration
]
```

## Environment Configuration

### Dependencies
*   Python 3.10+
*   `uv` (Python package and dependency manager)
*   Webots Robotics Simulator
*   YOLOv11 weights (`yolo11n.pt` required in the root directory)

### Installation
```bash
git clone <repository_url>
cd drone-controling
uv sync
```

## Webots Simulation Configuration
To enable external Python control over the Webots physics engine, the simulation must be explicitly configured to accept external controller connections. Failure to complete this step will result in an `AttributeError` during `Robot()` initialization.

1.  Launch Webots and load `sim-world/drone_world.wbt`.
2.  Navigate to the Scene Tree and select the primary UAV node (e.g., `ParrotAnafi`).
3.  In the Properties panel, locate the `controller` field.
4.  Initialize the physics engine by pressing the **Play** button in the Webots interface prior to executing the Python script.

## Execution Protocols
Ensure the Webots simulation is actively running before invoking the control script.

### Standard Cinematic Mode
Executes the flight director with conservative kinematic limits and EMA smoothing designed for final video capture.
```bash
uv run main_tracker.py
```

### Diagnostic Mode
Enables aggressive kinematic limits for rapid approach testing, detailed console telemetry, and a specialized visual overlay.
```bash
uv run main_tracker.py --debug
```

#### Diagnostic Mode Specifications
When the `--debug` flag is invoked, the system modifies its operational parameters and visual output:
*   **Kinematic Overrides**: Disables the EMA low-pass filter (`ALPHA = 1.0`) and increases maximum pitch deflection (`MAX_PITCH_PCT = 100`) to allow rapid target acquisition.
*   **Telemetry Logging**: Elevates the logging level to `DEBUG`, outputting continuous bounding-box coordinates, area calculations, and confidence intervals to the console.
*   **Visual Overlay**: Renders a diagnostic Heads-Up Display (HUD) on the video feed, comprising:
    *   A 3x3 rule-of-thirds alignment grid.
    *   A central optical crosshair.
    *   A 50% opacity red bounding box over the detected target, accompanied by a crisp border and confidence metric.

## Directory Structure
```text
drone-controling/
├── main_tracker.py        # Primary execution loop, kinematic limits, and diagnostic overlays
├── cinema_director.py     # Macro-state machine governing shot sequencing
├── maneuvers.py           # IBVS mathematical models for Approach, Helix, and Orbit trajectories
├── detector.py            # YOLO-based object detection pipeline
├── mpc_controller.py      # Model Predictive Control trajectory generation
├── pid.py                 # Discrete-time PID controller with anti-windup
├── olympe/                # Shadow SDK (Translates Olympe ARSDK messages to Webots physics)
│   ├── __init__.py        # Webots Robot wrapper, motor mixing, and gimbal axis inversion
│   ├── messages/          # Mock ARSDK messages (PCMD, TakeOff, Landing, Gimbal set_target)
│   └── enums/             # Mock ARSDK enumerations (frame_of_reference, control_mode)
├── sim-world/             # Webots simulation environment files
│   ├── drone_world.wbt
│   └── ParrotAnafi.proto
└── yolo11n.pt             # YOLOv11 neural network weights
```

## Diagnostic Resolution

### Gimbal Actuator Oscillation or Inversion
**Root Cause**: Discrepancy between Olympe ARSDK angular conventions and Webots joint axis orientations, or high-frequency command saturation.
**Resolution**: The Shadow SDK (`olympe/__init__.py`) natively resolves this by applying an axis inversion (`webots_rad = -angle_rad`) and enforcing a strict rate-limiter (maximum 2.0 degrees per frame). Do not modify the `set_gimbal_pitch` method within the Shadow SDK unless recalibrating for a different Webots PROTO model.

### Target Loss During Rotational Maneuvers
**Root Cause**: The commanded yaw rate exceeds the visual servoing loop's capacity to maintain the target within the camera frustum.
**Resolution**: Utilize the `--debug` flag to monitor console telemetry. Reduce the `yaw_rate` multiplier within the corresponding maneuver class in `maneuvers.py` until the bounding-box centroid remains stable during rotation. Ensure the shot list is restricted to the Cinematic Approach maneuver until rotational coupling is resolved.
