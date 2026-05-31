# Technical Specification: Autonomous Cinematic Drone System

## 1. Introduction

Autonomous aerial cinematography requires precise coordination of unmanned aerial vehicle (UAV) kinematics and camera extrinsics to maintain smooth motion, consistent subject framing, and predictable distance evolution. This document specifies the theoretical foundation, simulation architecture, and control methodology for an autonomous cinematic drone system targeting the first-generation Parrot Anafi platform ($320$g, 4K UHD) within the Webots simulator.

The system utilizes a YOLO-based vision pipeline for real-time subject tracking. All mathematical formulations utilize modern standard notations prevalent in contemporary robotics literature (e.g., ICRA, IROS, RSS) to ensure seamless alignment with current research, while explicitly mapping back to foundational quadrotor dynamics literature for rigorous verification.

---

## 2. Rigid-Body Dynamics on $SE(3)$

The state of the UAV is defined by its position $\mathbf{p} \in \mathbb{R}^3$, linear velocity $\mathbf{v} \in \mathbb{R}^3$, attitude $\mathbf{R} \in SO(3)$, and angular velocity $\boldsymbol{\omega} \in \mathbb{R}^3$ expressed in the body-fixed frame $\mathcal{F}_B$.

*Note on Notation Translation:* The foundational text by Mahony et al. [1] uses $\boldsymbol{\xi}$ for inertial position and $\boldsymbol{\Omega}$ for body-fixed angular velocity. In this document, we adopt the modern standard notation $\mathbf{p}$ for position and $\boldsymbol{\omega}$ for angular velocity to align with contemporary geometric control literature [3], while explicitly mapping to the source equations. We denote the body $z$-axis unit vector as $\mathbf{e}_3 = [0, 0, 1]^T$.

The Newton-Euler equations of motion are given by [1, p. 21, Eq. 1a-1d]:

$$
\dot{\mathbf{p}} = \mathbf{v} \qquad (1a)
$$

$$
m \dot{\mathbf{v}} = m \mathbf{g} + \mathbf{R}\mathbf{F} \qquad (1b)
$$

$$
\dot{\mathbf{R}} = \mathbf{R} [\boldsymbol{\omega}]_{\times} \qquad (1c)
$$

$$
\mathbf{I} \dot{\boldsymbol{\omega}} = -\boldsymbol{\omega} \times \mathbf{I} \boldsymbol{\omega} + \boldsymbol{\tau} \qquad (1d)
$$

where $\mathbf{g} = [0, 0, -g]^T$ is the gravity vector, $[\boldsymbol{\omega}]_{\times}$ is the skew-symmetric matrix, and $\mathbf{F}, \boldsymbol{\tau} \in \mathbb{R}^3$ are the aerodynamic forces and moments.

For cinematic flights involving lateral translation, blade flapping and induced drag introduce significant horizontal damping. The exogenous force applied to the rotor is modeled as [1, p. 24, Eq. 10]:

$$
\mathbf{F} := T_R \mathbf{e}_3 - T_R \mathbf{D} \mathbf{v}' \qquad (2)
$$

where $T_R$ is total thrust, $\mathbf{v}' = \mathbf{R}^T \mathbf{v}$ is the body-fixed velocity, and $\mathbf{D}$ is the drag matrix. This natural horizontal damping is critical for state estimation and ensures stable low-speed cinematic maneuvers [1, p. 24].

---

## 3. Differential Flatness and Cinematic Trajectory Generation

Quadrotors are differentially flat systems with flat outputs $\boldsymbol{\sigma} = [\mathbf{p}^T, \psi]^T \in \mathbb{R}^4$, where $\psi$ is the yaw angle [1, p. 29-30; 2]. Any sufficiently differentiable trajectory in the flat output space corresponds to a dynamically feasible state and input trajectory.

**Minimum-Jerk Formulation:**
While aggressive racing maneuvers minimize snap [1, p. 30, Eq. 32; 2], cinematic shots prioritize smooth camera motion and minimal gimbal compensation. Furthermore, minimizing jerk strictly limits high-frequency mechanical vibrations, which is vital for preventing the propagation of existing structural stress fractures in older, lightweight composite airframes. We minimize the integral of squared jerk $\mathbf{j} = \dddot{\mathbf{p}}$ [5]:

$$
J = \int_{0}^{T} \|\dddot{\mathbf{p}}(t)\|^2 dt \qquad (3)
$$

The analytical solution for a 1D axis is a 5th-degree polynomial parameterized by normalized time $\tau = t/T \in [0, 1]$:

$$
p(\tau) = p_0 + (p_f - p_0) \left( 10\tau^3 - 15\tau^4 + 6\tau^5 \right) \qquad (4)
$$

This guarantees $C^2$ continuity and zero initial/final velocity and acceleration, eliminating visible motion discontinuities in rendered footage.

---

## 4. Geometric Tracking Control

To track the desired trajectory $(\mathbf{p}_{des}, \mathbf{v}_{des}, \mathbf{a}_{des}, \mathbf{R}_{des})$, we employ a cascaded geometric controller.

**Outer Loop (Position):**
Define position error $\mathbf{e}_p = \mathbf{p} - \mathbf{p}_{des}$ and velocity error $\mathbf{e}_v = \mathbf{v} - \mathbf{v}_{des}$. The desired acceleration is computed via PD feedback with feedforward [1, p. 29, Eq. 28-29]:

$$
\mathbf{a}_{des} = \ddot{\mathbf{p}}_{des} - \mathbf{K}_p \mathbf{e}_p - \mathbf{K}_d \mathbf{e}_v \qquad (5)
$$

This $\mathbf{a}_{des}$ is mapped to desired thrust and attitude via the differential flatness equations [1, p. 30].

**Inner Loop (Attitude):**
The attitude error on $SO(3)$ is defined using the vee map $(\cdot)^{\vee}$, which extracts the vector from a skew-symmetric matrix [3]:

$$
\mathbf{e}_R = \frac{1}{2} \left( \mathbf{R}_{des}^T \mathbf{R} - \mathbf{R}^T \mathbf{R}_{des} \right)^{\vee} \qquad (6)
$$

The control torque is [1, p. 29, Eq. 26-27; 3, Eq. 15]:

$$
\boldsymbol{\tau} = -\mathbf{K}_R \mathbf{e}_R - \mathbf{K}_{\omega} \boldsymbol{\omega} + \boldsymbol{\omega} \times \mathbf{I} \boldsymbol{\omega} \qquad (7)
$$

---

## 5. Visual Servoing with YOLO Object Detection

Autonomous cinematography requires maintaining the subject within specific aesthetic bounds. A YOLO object detection pipeline outputs bounding boxes for the target. Let $\mathbf{s} = [u, v, A]^T \in \mathbb{R}^3$ represent the image features, where $(u, v)$ is the bounding box center and $A$ is the normalized bounding box area.

The relationship between camera spatial velocity $\mathbf{v}_c = [\mathbf{v}_c^T, \boldsymbol{\omega}_c^T]^T \in \mathbb{R}^6$ and feature velocity $\dot{\mathbf{s}}$ is governed by the interaction matrix $\mathbf{L}_s \in \mathbb{R}^{3 \times 6}$ [1, p. 31, Eq. 33; 4]:

$$
\dot{\mathbf{s}} = \mathbf{L}_s \mathbf{v}_c \qquad (8)
$$

For a pinhole camera with focal length $f_{px}$ and target depth $Z$, the interaction matrix for the center point $(u,v)$ is [4, Eq. 15]:

$$
\mathbf{L}_{u,v} = \begin{bmatrix}
-\frac{f_{px}}{Z} & 0 & \frac{u}{Z} & \frac{uv}{f_{px}} & -(f_{px} + \frac{u^2}{f_{px}}) & v \\
0 & -\frac{f_{px}}{Z} & \frac{v}{Z} & f_{px} + \frac{v^2}{f_{px}} & -\frac{uv}{f_{px}} & -u
\end{bmatrix} \qquad (9)
$$

**Depth Estimation Fusion:**
Standard 2D visual servoing requires an approximation of the depth $Z$. Because pure trigonometric estimation via drone altitude $h$ and camera pitch $\theta_c$ fails during aggressive maneuvers over uneven terrain, we fuse the telemetry depth with the YOLO bounding box scale $A$. Let $A^*$ be the reference bounding box area at a known distance $Z^*$. The instantaneous depth is estimated as:

$$
Z \approx \alpha \left( \frac{h}{\tan(\theta_c)} \right) + (1-\alpha) \left( Z^* \sqrt{\frac{A^*}{A}} \right) \qquad (10)
$$

where $\alpha \in [0, 1]$ is a confidence tuning parameter.

---

## 6. Implementation Architecture

The system architecture is strictly divided into a physics simulator and an autonomous controller. Because the Parrot Olympe API is structurally incompatible with native Webots simulation environments, standard Olympe wrapper nodes cannot be utilized.

Instead, the controller is implemented as a Webots `<extern>` Python module. This entirely bypasses the Olympe API, granting the Python controller low-latency, direct access to the simulation's physics step, allowing the geometric $SE(3)$ controller to dictate rotor velocities directly.

To facilitate real-time trajectory optimization (MPC) running synchronously with the YOLO vision pipeline, the trajectory generator leverages PyTorch's Metal Performance Shaders (MPS) backend on Apple Silicon. This enables $O(1)$ vectorized minimum-jerk candidate scoring.

```python
import torch

def batch_minimum_jerk_mps(starts: torch.Tensor, goals: torch.Tensor, 
                           durations: torch.Tensor, times: torch.Tensor):
    """
    Vectorized minimum-jerk trajectory evaluation optimized for Apple Silicon (MPS).
    Maps to Eq. (4) in the theoretical framework.
    """
    # Ensure execution on the M-series GPU
    device = torch.device("mps")
    
    tau = torch.clamp(times / durations, 0.0, 1.0).unsqueeze(1).to(device)
    starts = starts.to(device)
    goals = goals.to(device)
    
    # Position polynomial: 10*tau^3 - 15*tau^4 + 6*tau^5
    pos_coeff = 10*tau**3 - 15*tau**4 + 6*tau**5
    positions = starts + (goals - starts) * pos_coeff
    
    # Velocity polynomial: derivative of position
    vel_coeff = (30*tau**2 - 60*tau**3 + 30*tau**4) / durations.unsqueeze(1).to(device)
    velocities = (goals - starts) * vel_coeff
    
    # Acceleration polynomial: derivative of velocity
    acc_coeff = (60*tau - 180*tau**2 + 120*tau**3) / (durations.unsqueeze(1).to(device)**2)
    accelerations = (goals - starts) * acc_coeff
    
    return positions, velocities, accelerations
```
## 7. Hardware-Agnostic Control Architecture

To ensure seamless transferability between the Webots simulation environment and the physical Parrot Anafi platform, the control pipeline strictly decouples maneuver logic from hardware execution. 

### 7.1 The `DroneCommand` Abstraction
The `maneuvers.py` module operates entirely in physical spatial units (meters per second and degrees per second) and outputs a standardized `DroneCommand` dataclass. It contains no simulator-specific or SDK-specific API calls. 

### 7.2 Hardware Mapping
The `main_tracker.py` module acts as the hardware abstraction layer. For the physical Anafi, it maps the physical velocities to Olympe's `PCMD` (Piloting Command) percentages based on the manufacturer's maximum physical limits (e.g., $15$ m/s horizontal, $4$ m/s vertical), clamped to cinematic thresholds ($3$ m/s horizontal, $2$ m/s vertical). 

For the Webots simulation, the exact same `maneuvers.py` module is imported. The hardware mapper is simply replaced with a `command_to_webots` function that maps the identical `DroneCommand` velocities to the `ParrotAnafi.proto` motor RPMs or velocity nodes via the internal $SE(3)$ geometric controller.

## 8. Bibliography

[1] R. Mahony, V. Kumar, and P. Corke, "Multirotor aerial vehicles: Modeling, estimation, and control of quadrotor," IEEE Robotics & Automation Magazine, vol. 19, no. 3, pp. 20–32, Sep. 2012.

[2] D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control for quadrotors," in Proc. IEEE Int. Conf. Robotics and Automation (ICRA), Shanghai, China, 2011, pp. 2520–2525.

[3] T. Lee, M. Leok, and N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3)," in Proc. IEEE Conf. Decision and Control (CDC), Atlanta, GA, 2010, pp. 5420–5425.

[4] F. Chaumette and S. Hutchinson, "Visual servo control. I. Basic approaches," IEEE Robotics & Automation Magazine, vol. 13, no. 4, pp. 82–90, Dec. 2006.

[5] T. Flash and N. Hogan, "The coordination of arm movements: An experimentally confirmed mathematical model," Journal of Neuroscience, vol. 5, no. 7, pp. 1688–1703, Jul. 1985.

[6] Parrot S.A., "Anafi Drone Technical Specifications," Parrot Official Documentation, 2018. [Online]. Available: https://www.parrot.com/en/drones/anafi/technical-specifications