#  Autonomous Drone Landing on a Moving Vehicle
### PX4 · Gazebo Classic · ROS2 Humble · OpenCV ArUco · Python Offboard Control

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![PX4 Autopilot](https://img.shields.io/badge/PX4-Autopilot%20SITL-purple?logo=drone&logoColor=white)](https://px4.io/)
[![Python](https://img.shields.io/badge/Python-3.10+-green?logo=python&logoColor=white)](https://python.org)
[![OpenCV](https://img.shields.io/badge/OpenCV-ArUco-red?logo=opencv&logoColor=white)](https://opencv.org/)
[![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic-orange)](http://gazebosim.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**An autonomous drone that detects, tracks, and lands on a moving vehicle in simulation — using vision-based ArUco marker detection, a Kalman filter for predictive tracking, PID + lead-compensation control, and a robust multi-phase landing state machine running on PX4 offboard mode.**

[📹 Demo Video](#-demo) · [🧠 Architecture](#-system-architecture) · [⚙️ Setup](#️-setup-and-installation)

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

> **Add your demo video here.**

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

##  System Architecture

The full pipeline flows from the Gazebo camera to PX4 motor commands:
<img width="1360" height="1940" alt="image" src="https://github.com/user-attachments/assets/bba24f1a-bd72-4591-a7f1-82d5b13922a5" />

---


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

##  Key Technical Concepts

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

## 🎬 Demo Video
The demo is already embedded at the top of this README as a clickable YouTube thumbnail.
Direct link: https://youtu.be/U6qtfy8A2m4
The video shows the full sequence: drone arms and takes off → ArUco marker detected on moving vehicle → alignment → adaptive descent → blind landing → "LANDED ON MARKER" confirmation banner.
How the YouTube embed works on GitHub
GitHub README files cannot play video inline, but they can display a clickable image. The embed uses YouTube's auto-generated thumbnail URL:
markdown[![Alt text](https://img.youtube.com/vi/VIDEO_ID/maxresdefault.jpg)](https://www.youtube.com/watch?v=VIDEO_ID)

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

##  License

This project is licensed under the **MIT License** — see [LICENSE](LICENSE) for details.

---

##  Acknowledgements

- [PX4 Autopilot](https://px4.io/) — open-source flight stack
- [ROS2 Humble](https://docs.ros.org/en/humble/) — robotics middleware
- [OpenCV ArUco](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) — marker detection
- [Gazebo Classic](http://gazebosim.org/) — robotics simulator

---

##  Author

**RAGHAV ARORA**
> B.Tech  in AI-ML
> 
> [LinkedIn](https://linkedin.com/in/raghav-arora18) · [Email](mailto:arora.arraghav@email.com)

---

<div align="center">

*If this project helped you, please consider giving it a ⭐ — it helps others find it!*

</div>
