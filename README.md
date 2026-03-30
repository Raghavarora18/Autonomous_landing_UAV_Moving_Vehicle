# Autonomous_landing_UAV_Moving_Vehicle
This project demonstrates a vision-based autonomous UAV system that performs precision landing on a moving vehicle. The system integrates real-time image processing with control logic to track and land on a dynamic target. The project highlights concepts such as visual servoing, predictive control, state machine design.
<div align="center">

<img src="docs/images/banner.gif" alt="Drone landing on moving vehicle" width="100%">

# 🚁 Autonomous Drone Landing on a Moving Vehicle
### PX4 · Gazebo Classic · ROS2 Humble · OpenCV ArUco · Python Offboard Control

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![PX4 Autopilot](https://img.shields.io/badge/PX4-Autopilot%20SITL-purple?logo=drone&logoColor=white)](https://px4.io/)
[![Python](https://img.shields.io/badge/Python-3.10+-green?logo=python&logoColor=white)](https://python.org)
[![OpenCV](https://img.shields.io/badge/OpenCV-ArUco-red?logo=opencv&logoColor=white)](https://opencv.org/)
[![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic-orange)](http://gazebosim.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**A fully autonomous drone that detects, tracks, and lands on a moving vehicle in simulation — using vision-based ArUco marker detection, a Kalman filter for predictive tracking, PID + lead-compensation control, and a robust multi-phase landing state machine running on PX4 offboard mode.**

[📹 Demo Video](#-demo) · [🧠 Architecture](#-system-architecture) · [⚙️ Setup](#️-setup-and-installation) · [📦 Modules](#-module-breakdown) · [🔮 Future Work](#-future-improvements)

</div>

---

## 📌 Project Overview

This project solves a challenging robotics problem: **can a drone autonomously land on a vehicle that is actively moving beneath it?**

A moving landing target introduces two hard problems:
1. **Perception** — the drone must continuously detect and estimate the target's position and velocity, even when detection is noisy or temporarily lost.
2. **Control** — the drone must not merely follow the target but *predict* its future position, align above it, and descend precisely enough to land on its roof.

This simulation tackles both using a pipeline built entirely on **open-source tools**: PX4 autopilot, ROS2, Gazebo Classic, and OpenCV.

| Component | Technology |
|---|---|
| Simulator | Gazebo Classic |
| Autopilot | PX4 SITL (Software-In-The-Loop) |
| Middleware | ROS2 Humble |
| Vision | OpenCV — ArUco Marker Detection |
| Tracking | Kalman Filter (custom Python) |
| Control | Python — PID + Predictive Lead Control |
| Vehicle Plugin | Gazebo Model Plugin (C++) |

---

## 🎬 Demo

> **Add your demo video here.** See the [How to Add Your Demo Video](#-how-to-add-your-demo-video) section below for exact instructions.

<div align="center">
<img src="docs/images/demo_screenshot.png" alt="Demo screenshot showing drone above moving vehicle" width="80%">
</div>

**What the demo shows:**
- Drone arms and takes off to 8m altitude in offboard mode
- ArUco marker is detected on the roof of a moving red hatchback
- Kalman filter smooths noisy detections and estimates vehicle velocity in real-time
- Drone aligns horizontally above the marker using PID + lead-prediction
- Drone descends adaptively — faster when centred, slower when drifting
- Blind landing phase engages when marker is lost near the ground
- Triple-condition landing confirmation triggers disarm on the car roof

---

## 🧠 System Architecture

The full pipeline flows from the Gazebo camera to PX4 motor commands:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        GAZEBO CLASSIC SIMULATION                         │
│                                                                           │
│   ┌──────────────────┐      ┌──────────────────┐    ┌─────────────────┐ │
│   │  Downward Camera │      │   Moving Vehicle  │    │  ArUco Marker   │ │
│   │  640×480, 80°FOV │      │  r=2.5m ω=0.5r/s │    │  ID=0, 1m×1m   │ │
│   └────────┬─────────┘      └──────────────────┘    └────────┬────────┘ │
│            │ /drone/downward_camera/image_raw                  │ on roof  │
└────────────┼─────────────────────────────────────────────────┼───────────┘
             │                                                  │
             ▼                                                  │
┌────────────────────────────────┐                             │
│         aruco_node.py          │◄────────────────────────────┘
│                                │  (detects marker in image)
│  CLAHE preprocess              │
│  ArUco detection (4×4_50)      │
│  Pose estimation (solvePnP)    │
│  Kalman filter [x, y, vx, vy]  │
│  Coasting (up to 15 frames)    │
└────────────────┬───────────────┘
                 │ /aruco_pose (PoseStamped)
                 │ position: (x, y, z)
                 │ orientation: (vx, vy, coast_flag, 1.0)
                 ▼
┌────────────────────────────────────────────────────────────────┐
│                     offboard_landing.py                         │
│                                                                  │
│  State machine: INIT→TAKEOFF→ALIGN→DESCEND→BLIND_LAND→DONE     │
│  PID control with altitude-adaptive gains                        │
│  Lead prediction: target_pos = marker + vel × LEAD_TIME         │
│  Camera → NED frame transform                                    │
│  Alignment window (70% of 20-frame sliding buffer)              │
│  Adaptive descent: FAST(0.4) / MED(0.25) / SLOW(0.1) / PAUSE  │
│  Triple landing confirmation: alt<0.8m + err<0.4m + stable×10  │
└────────────┬─────────────────────────────────────────────────┘
             │
             ├─► /fmu/in/offboard_control_mode   (20Hz heartbeat)
             ├─► /fmu/in/trajectory_setpoint     (velocity NED)
             └─► /fmu/in/vehicle_command          (arm / disarm)
                                │
                                ▼
                    ┌───────────────────────┐
                    │   PX4 SITL Autopilot  │
                    │  Position + Velocity  │
                    │  Controller → Motors  │
                    └───────────┬───────────┘
                                │
                                ▼
                    ┌───────────────────────┐
                    │  Iris Quadrotor Drone │
                    │  Translates, descends │
                    │  Lands on vehicle ✓   │
                    └───────────────────────┘
```

---

## 📁 Repository Structure

```
autonomous-drone-landing/
│
├── README.md                          ← You are here
├── LICENSE
│
├── src/                               ← Python ROS2 nodes
│   ├── aruco_node.py                  ← ArUco detection + Kalman tracker
│   └── offboard_landing.py            ← Offboard controller + state machine
│
├── plugins/                           ← Gazebo C++ plugins
│   └── circular_motion_plugin.cpp     ← Vehicle circular motion
│
├── config/                            ← Configuration files
│   ├── camera_params.yaml             ← Camera intrinsics (FOV, resolution)
│   └── controller_params.yaml         ← PID gains, thresholds
│
├── launch/                            ← ROS2 launch files
│   ├── aruco_detection.launch.py
│   └── full_system.launch.py
│
├── worlds/                            ← Gazebo world files
│   └── moving_vehicle_world.world
│
├── models/                            ← Gazebo model assets
│   └── aruco_marker/
│       ├── model.config
│       └── textures/
│           └── aruco_id0.png          ← ArUco marker texture for vehicle roof
│
├── docs/                              ← Documentation assets
│   ├── images/
│   │   ├── banner.gif                 ← Top banner animation
│   │   ├── demo_screenshot.png
│   │   ├── state_machine.png
│   │   ├── kalman_tracking.png
│   │   └── pid_diagram.png
│   └── architecture.md               ← Detailed architecture notes
│
└── requirements.txt                   ← Python dependencies
```

---

## 📦 Module Breakdown

### 1. `aruco_node.py` — Vision + Tracking

This node is the **eyes** of the system. It processes raw camera images and produces a smooth, reliable estimate of where the ArUco marker is, how fast it's moving, and whether detection is still valid.

#### What it does, step by step:

**Step 1 — Receive image**
Subscribes to `/drone/downward_camera/image_raw`. Each frame triggers `image_callback()`.

**Step 2 — CLAHE preprocessing**
Before detection, the grayscale image is enhanced using **CLAHE** (Contrast Limited Adaptive Histogram Equalization). This dramatically improves detection in uneven lighting — shadows from the drone's frame or sunlight variation can wash out the marker otherwise.

```python
self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
processed = self.clahe.apply(gray)
```

**Step 3 — ArUco detection**
OpenCV's `aruco.detectMarkers()` finds the four corners of the marker in pixel space. Custom `DetectorParameters` are tuned for robustness — smaller `minMarkerPerimeterRate` catches the marker at altitude, looser `errorCorrectionRate` handles partial occlusion.

**Step 4 — Pose estimation**
`aruco.estimatePoseSingleMarkers()` uses the known physical size of the marker (1.0m) and the camera's intrinsic matrix `K` to compute the 3D translation vector `tvec = [rx, ry, rz]`:
- `rx`: lateral offset (right = positive)
- `ry`: forward offset (down in image = positive)
- `rz`: depth — the distance from camera to marker plane, used as **altitude proxy**

Camera intrinsics are computed from the Gazebo SDF field of view:
```python
_fx = (IMAGE_W / 2.0) / math.tan(FOV_RAD / 2.0)
```

**Step 5 — Kalman filter**
A 4-state Kalman filter `[x, y, vx, vy]` runs every frame:
- **Predict**: advances the state estimate using constant-velocity model
- **Update**: fuses the new measurement when detection succeeds
- **Coast**: if detection fails, only the predict step runs — giving up to 15 frames of graceful prediction before the tracker resets

```python
# Q tuned for a moving vehicle — high velocity noise for quick adaptation
self.Q = np.diag([0.05, 0.05, 1.0, 1.0])
```

**Step 6 — Publish `/aruco_pose`**
The node packs position, velocity, and a coasting flag into a `PoseStamped` message using a custom encoding contract:
```
position.x/y = smoothed marker offset (camera frame, metres)
position.z   = depth / altitude above marker
orientation.x/y = Kalman vx, vy (m/s)
orientation.z = 0.0 (live) or 1.0 (coasting)
orientation.w = 1.0 (sentinel)
```

**Step 7 — Overlay + landing confirmation**
The camera window displays real-time vehicle speed (derived from Kalman velocity, shown in km/h) and a green "LANDED ON MARKER" banner when `rz < 0.8m` and horizontal error `< 0.4m`.

---

### 2. `offboard_landing.py` — Control + State Machine

This node is the **brain** of the system. It reads the ArUco pose and produces velocity commands that guide the drone through every phase of the autonomous landing.

#### State Machine

```
INIT ──────► TAKEOFF ──────► ALIGN ──────► DESCEND ──────► BLIND_LAND ──► DONE
             (climb to        (align        (adaptive        (open-loop     (disarm)
              8m alt)          above         descent)         final drop)
                              marker)
```

**INIT** — waits 50 control cycles for PX4 to be ready, then sends arm command and switches to offboard mode.

**TAKEOFF** — publishes position setpoints at increasing altitude (1.0 m/s rate) until the drone reaches `TARGET_ALT = 8.0m`.

**ALIGN** — tracks the marker at cruise altitude using PID + lead prediction. Uses a **sliding window buffer** of 20 frames: the drone transitions to DESCEND only when ≥70% of the last 20 frames show the marker within `ALIGN_RADIUS_M = 2.0m`. Brief marker losses add a zero to the window rather than resetting it, making the transition robust to transient detection failures.

**DESCEND** — adaptive descent rate based on horizontal tracking error:
| Error | Descent rate |
|---|---|
| `< 0.5m` | 0.40 m/s (FAST) |
| `0.5 – 1.8m` | 0.25 m/s (MED) |
| `1.8 – 2.5m` | 0.10 m/s (SLOW) |
| `> 2.5m` | 0.00 m/s (PAUSE) |

If the marker is lost for >2 seconds above 4m, the controller aborts back to ALIGN. Altitude is synced from the marker depth every 10 cycles (every 0.5 seconds) to avoid overwriting the descent counter.

**BLIND_LAND** — below 2.5m, if the marker is lost (physically occluded by proximity), the drone freezes its last tracking velocity and drops straight down at 2.0 m/s. This is the most critical phase: the marker cannot be detected at centimetre range by a downward camera, so the system must commit.

**DONE** — triggered by the **triple landing confirmation**:
1. `marker_z < 0.8m` — camera is very close to the marker plane
2. Horizontal error `< 0.4m` — drone is centred over the marker
3. Both conditions stable for 10 consecutive frames (~0.5 seconds)

On confirmation, `VehicleCommand(400, 0.0)` (disarm) is sent.

#### PID + Lead Prediction Control

The core velocity command is computed as:

```
target = marker_position + marker_velocity × LEAD_TIME + direction × STATIC_BIAS

error_NED = cam_to_NED(target)

vN = KFF×vel_N + KP×error_N + KI×integral_N
vE = KFF×vel_E + KP×error_E + KI×integral_E

vN_out = smooth(vN) − KD×Δsmooth/dt     # D-term on smoothed signal
```

**Lead prediction** is the key innovation: instead of chasing where the vehicle *is*, the drone aims at where the vehicle *will be* `LEAD_TIME` seconds from now. This eliminates the lag inherent in a reactive PID controller when tracking a moving target.

**Altitude-adaptive gains**: as the drone descends from 8m to 2m, `KP` increases from 0.25 to 0.40 and `KI` increases from 0.02 to 0.05, making the controller tighter near the ground where precision matters most.

#### Camera → NED Frame Transform

The downward camera uses a coordinate frame where:
- `x` points right in the image → maps to NED East
- `y` points down in the image → maps to NED North (with sign flip)

```python
NORTH_AXIS = 'y'
NORTH_SIGN = -1    # camera y (down) → NED North (forward = positive)
EAST_AXIS  = 'x'
EAST_SIGN  = +1
```

This mapping is configurable as ROS2 parameters to handle different camera mounting orientations.

---

### 3. `circular_motion_plugin.cpp` — Vehicle Simulation

A Gazebo ModelPlugin that drives the red hatchback vehicle in a circular path. The physics are grounded correctly — rather than setting absolute world positions, the plugin:
1. Computes linear speed `v = r × ω = 2.5 × 0.5 = 1.25 m/s`
2. Expresses it as a forward velocity in the **vehicle's local frame**
3. Rotates it into the world frame using the vehicle's current orientation quaternion
4. Preserves the Z-axis velocity (gravity/suspension) to avoid hovering
5. Sets angular velocity `ω = 0.5 rad/s` for turning

This produces a stable, physically realistic circular trajectory that the tracking and control systems can test against.

---

## ⚙️ Setup and Installation

### Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble Hawksbill |
| PX4 Autopilot | main / v1.14+ |
| Gazebo | Classic (Gazebo 11) |
| Python | 3.10+ |
| OpenCV | 4.6+ (with contrib for ArUco) |

### Step 1 — Install PX4

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl gazebo-classic_iris
```

### Step 2 — Install ROS2 Humble

Follow the [official ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), then:

```bash
sudo apt install ros-humble-cv-bridge ros-humble-vision-opencv
```

### Step 3 — Install px4_msgs and MicroXRCE-DDS

```bash
pip install --user -U empy==3.3.4 pyros-genmsg setuptools

# Micro XRCE-DDS Agent
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake ..
make
sudo make install

# px4_msgs
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Step 4 — Clone This Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/autonomous-drone-landing.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Step 5 — Install Python Dependencies

```bash
pip install opencv-contrib-python numpy
```

### Running the Simulation

Open four terminals:

**Terminal 1 — Start PX4 SITL + Gazebo:**
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris__moving_vehicle
```

**Terminal 2 — Start Micro XRCE-DDS bridge:**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3 — Start ArUco detection node:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run autonomous_drone_landing aruco_node
```

**Terminal 4 — Start offboard landing controller:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run autonomous_drone_landing offboard_landing
```

The drone will arm, take off, detect the marker, and autonomously land on the moving vehicle.

---

## 🔑 Key Technical Concepts

### ArUco Pose Estimation
ArUco markers are square fiducial markers with a unique binary pattern. OpenCV's `estimatePoseSingleMarkers()` uses the known physical size of the marker and the camera intrinsic matrix to solve a PnP (Perspective-n-Point) problem — determining the 3D rigid body transform between the camera and the marker. The result is a translation vector giving the marker's position in the camera frame.

### Kalman Filtering for Tracking
A linear Kalman filter with state `[x, y, vx, vy]` fuses noisy pose measurements with a constant-velocity motion model. The process noise `Q` is tuned with high velocity variance (`1.0`) so the filter adapts quickly when the vehicle changes direction. The measurement noise `R` is tight (`0.02`) because pose estimation is relatively accurate when the marker is detected. The result: smooth position estimates and a reliable velocity signal even with occasional detection failures.

### Predictive Lead Control
A standard PID controller targeting the current marker position will always lag behind a moving target — by the time the error is processed and the drone responds, the vehicle has moved. Lead prediction shifts the target point forward in time: `target = pos + vel × LEAD_TIME`. With `LEAD_TIME = 0.8s`, the drone anticipates 0.8 seconds of vehicle motion, nearly eliminating tracking lag at highway-speed-like velocities.

### PX4 Offboard Mode
Offboard mode allows an external computer to directly command the drone's position, velocity, or attitude without going through the standard RC/mission interface. It requires a continuous stream of setpoints at ≥2 Hz (this system runs at 20 Hz). The `TrajectorySetpoint` message accepts velocity commands in the NED (North-East-Down) frame, with unused axes set to `NaN`. The `OffboardControlMode` message declares which axes are under offboard control.

### Adaptive Descent
Rather than descending at a fixed rate, the controller adjusts descent speed based on how well the drone is currently tracking the marker. Fast descent only when well-centred; full stop when the error grows too large. This prevents the drone from "falling past" the vehicle during a momentary tracking error.

---

## 📸 Images to Add to Your Repository

For a professional-looking repo, include these images in `docs/images/`:

| Image | Description | How to capture |
|---|---|---|
| `banner.gif` | Animated GIF of the full landing sequence | Screen record Gazebo, convert with `ffmpeg` |
| `demo_screenshot.png` | Gazebo + camera window side-by-side | Screenshot during landing |
| `aruco_detection.png` | Camera window showing marker detected with axes | Screenshot from `aruco_node.py` OpenCV window |
| `state_machine.png` | State machine diagram | Export from draw.io or Mermaid |
| `landing_confirmation.png` | Camera window showing green "LANDED ON MARKER" banner | Screenshot at touchdown |
| `vehicle_overhead.png` | Bird's eye view of drone above vehicle | Gazebo camera view |

---

## 🎬 How to Add Your Demo Video

GitHub does not host video files. The recommended approach:

### Option A — YouTube (recommended for README)
1. Record your simulation using `obs-studio` or `kazam`: `sudo apt install obs-studio`
2. Upload to YouTube
3. Add to README:
```markdown
[![Demo Video](https://img.youtube.com/vi/YOUR_VIDEO_ID/maxresdefault.jpg)](https://www.youtube.com/watch?v=YOUR_VIDEO_ID)
```
The thumbnail becomes a clickable image that opens YouTube.

### Option B — GitHub Release Assets
1. Go to your repo → **Releases** → **Create a new release**
2. Drag your `.mp4` file into the release assets
3. Copy the direct link to the file and embed it in your README

### Option C — GitHub Issues trick
1. Open a new Issue in your own repo
2. Drag and drop the `.mp4` onto the comment box — GitHub auto-uploads it and gives you a URL
3. Copy the URL, close the Issue (don't submit), and paste the link in your README

### Recording Tips
- Record at 1080p, 30fps
- Capture both the Gazebo 3D view and the ArUco camera window side-by-side
- Let the full sequence run: takeoff → align → descend → blind land → "LANDED ON MARKER" confirmation
- Keep it under 3 minutes for maximum viewer retention

---

## 🔮 Future Improvements

### Perception
- [ ] **GPS-denied marker re-acquisition**: use IMU dead-reckoning to navigate back toward the last known marker position after a long detection gap
- [ ] **Multi-marker support**: use multiple ArUco markers on the vehicle for redundancy at steep angles
- [ ] **Deep learning detector**: replace ArUco with a YOLOv8 model fine-tuned on the vehicle, removing dependency on a physical marker
- [ ] **Depth camera**: replace monocular altitude estimation with a depth sensor for metric ground-truth altitude

### Control
- [ ] **MPC (Model Predictive Control)**: replace the PID with an MPC that optimizes over a prediction horizon, naturally handling constraints and lead time
- [ ] **Velocity feedforward tuning**: re-enable `KFF` once the Kalman velocity is computed in the world frame rather than the camera frame (currently disabled to prevent oscillation)
- [ ] **Adaptive LEAD_TIME**: scale lead time dynamically based on vehicle speed estimate from the Kalman filter
- [ ] **Wind disturbance rejection**: add disturbance estimation and feedforward compensation

### System
- [ ] **Real hardware deployment**: migrate from SITL to real PX4 hardware with a Raspberry Pi companion computer
- [ ] **ROS2 lifecycle nodes**: use managed nodes for cleaner startup/shutdown sequencing
- [ ] **Parameter hot-reloading**: expose PID gains and thresholds as dynamic reconfigure parameters
- [ ] **Simulation-to-real gap**: test with a physical wheeled robot carrying the marker before full drone deployment

---

## 📊 Performance Metrics (Simulation)

| Metric | Value |
|---|---|
| Vehicle speed | ~1.25 m/s (circular, r=2.5m) |
| Takeoff altitude | 8.0 m |
| Alignment radius threshold | 2.0 m |
| Alignment hold window | 1.0 s (70% of 20 frames) |
| Typical alignment time | ~5–10 seconds |
| Descent rate (centred) | 0.40 m/s |
| Blind landing threshold | 2.5 m altitude |
| Landing confirmation altitude | < 0.8 m |
| Landing confirmation error | < 0.4 m horizontal |
| Control loop rate | 20 Hz |

---

## 📄 License

This project is licensed under the **MIT License** — see [LICENSE](LICENSE) for details.

MIT is the right choice here because:
- It allows anyone to use, modify, and build on your work
- It's compatible with PX4, ROS2, and OpenCV (all permissive licenses)
- It's the most widely recognised open-source license, ideal for portfolio and research visibility
- It requires only attribution — anyone who uses your code must credit you

---

## 🤝 Acknowledgements

- [PX4 Autopilot](https://px4.io/) — open-source flight stack
- [ROS2 Humble](https://docs.ros.org/en/humble/) — robotics middleware
- [OpenCV ArUco](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) — marker detection
- [Gazebo Classic](http://gazebosim.org/) — robotics simulator

---

## 👨‍💻 Author

**[Your Name]**
> B.Tech / M.Tech in [Your Field] · [Your College]
> 
> [LinkedIn](https://linkedin.com/in/yourprofile) · [GitHub](https://github.com/yourusername) · [Email](mailto:you@email.com)

---

<div align="center">

*If this project helped you, please consider giving it a ⭐ — it helps others find it!*

</div>
