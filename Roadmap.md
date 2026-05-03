# AeroSAR — Autonomous Search and Rescue Drone

> Simulation-first autonomous SAR system built on ROS 2 Jazzy + PX4 SITL + Gazebo Harmonic.
> A modular pipeline from flight control to mission reporting, with research-grade methods at each stage.

---

## Demo

> GIF goes here after first successful end-to-end mission run

---

## System Architecture

```
Gazebo disaster world
        ↓
PX4 SITL (x500 quadrotor)
        ↓
uXRCE-DDS bridge
        ↓
┌─────────────────────────────────────────┐
│           ROS 2 Jazzy                   │
│                                         │
│  mission_manager ──► px4_interface      │
│        ▲                                │
│        │                                │
│     planner                             │
│        │                                │
│     mapping ◄── geolocation ◄── perception
│        │                                │
│     reporting + evaluator               │
└─────────────────────────────────────────┘
```

---

## Module Methods Overview

### 1. `1_interfaces/msgs` — Message Definitions
Custom ROS 2 interface layer shared by all nodes.

| Message               | Purpose                                                 |
| --------------------- | ------------------------------------------------------- |
| `Detection.msg`       | YOLO output: class, confidence, bounding box, timestamp |
| `VictimCandidate.msg` | Estimated world position + uncertainty                  |
| `CoverageCell.msg`    | Grid cell state: visited, confidence, count             |
| `MissionStatus.msg`   | Current state machine state + progress                  |
| `StartMission.srv`    | Trigger mission with area polygon + params              |
| `GenerateReport.srv`  | Trigger report generation                               |

---

### 2. `2_control/px4_interface` — PX4 Bridge
**Current method:** uXRCE-DDS offboard control
- Subscribes to `/fmu/out/vehicle_local_position`, `/fmu/out/vehicle_status`
- Publishes to `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`
- Handles: arm, disarm, takeoff, land, RTL, failsafe

**Upgrade path:**
- [ ] Trajectory setpoints → full SE(3) geometric control
- [ ] Position commands → velocity commands for smoother MPC integration

---

### 3. `2_control/mission_manager` — Mission State Machine
**Current method:** Explicit finite state machine (FSM)
```
IDLE → PREFLIGHT → ARM → TAKEOFF → SEARCH → RTL → LAND → REPORT
```
- Each state has entry/exit conditions
- Monitors vehicle health, battery proxy, geofence

**Upgrade path:**
- [ ] FSM → Behavior Tree (BehaviorTree.CPP or py_trees)
- [ ] Add replanning trigger: if victim detected → inspect mode → resume search

---

### 4. `3_autonomy/planner` — Coverage Path Planner
**Method 1 (baseline): Boustrophedon / lawnmower**
- Input: search area polygon + flight altitude
- Compute sweep line spacing from camera FOV + desired overlap
- Output: ordered waypoint list

```
spacing = altitude × tan(FOV/2) × 2 × (1 - overlap)
```

**Method 2 (upgrade): Model Predictive Control (MPC) with acados**
- Drone dynamics model in CasADi (point-mass or full quadrotor)
- MPC solves receding horizon optimization every 100ms
- Objectives: minimize time, minimize energy, respect kinematic constraints
- Compares directly against baseline: coverage time, path smoothness, energy proxy

**Method 3 (research): Probabilistic search map**
- Each grid cell has a detection probability updated via Bayesian inference
- Planner prioritizes cells with highest information gain
- Reference: optimal search theory (Stone 1975, adapted for UAV)

**Upgrade path:**
- [ ] Lawnmower → MPC trajectory optimization (acados + CasADi)
- [ ] Static pattern → adaptive replanning based on victim probability map

---

### 5. `3_autonomy/mapping` — Coverage + Victim Map
**Method: Occupancy grid + DBSCAN clustering**
- Coverage grid: 2D map of visited/unvisited cells, updated as drone flies
- Victim registry: all raw detections stored with position + confidence
- DBSCAN clusters detections within 5m radius → single victim candidate
- Prevents duplicate reporting of same person

**Parameters:**
```yaml
grid_resolution: 1.0      # meters per cell
cluster_radius: 5.0       # meters — merge detections within this distance
min_observations: 2       # minimum detections to confirm a victim
```

**Upgrade path:**
- [ ] Static grid → Gaussian process occupancy map
- [ ] DBSCAN → Kalman filter per victim track (handles moving targets)

---

### 6. `4_perception/perception` — Human Detection
**Method 1 (baseline): Pretrained YOLOv8n / YOLO11n**
- Input: `/drone/camera/image_raw`
- Model: YOLOv8n pretrained on COCO, class = "person"
- Output: `Detection.msg` per detected human

**Method 2 (upgrade): SAHI — Slicing Aided Hyper Inference**
- Problem: humans are tiny in aerial images (~10-20px tall at 30m altitude)
- SAHI slices each frame into overlapping tiles, runs YOLO per tile, merges results
- Measurable improvement on small targets — directly comparable to baseline

**Method 3 (research): Fine-tuning on aerial SAR datasets**
- SARD: Search and Rescue Drone dataset
- HERIDAL: Aerial human detection for wilderness SAR
- VisDrone: Drone-based pedestrian detection benchmark
- Export to ONNX for deployment

**Key parameter:**
```
altitude 10m → human ~40px → baseline YOLO ok
altitude 30m → human ~13px → SAHI needed
altitude 50m → human ~8px  → fine-tuning needed
```

**Upgrade path:**
- [ ] COCO pretrained → fine-tuned on SARD/HERIDAL
- [ ] Full frame inference → SAHI tiled inference
- [ ] PyTorch → ONNX export → TensorRT (GPU acceleration)

---

### 7. `4_perception/geolocation` — Pixel to World Projection
**Method: Pinhole camera model + tf2 + ray-ground intersection**

```
pixel (u, v)
    ↓
camera ray via intrinsics K
    ↓
transform ray: camera → body → world (tf2)
    ↓
intersect with ground plane z=0
    ↓
local ENU position → GPS estimate
    ↓
uncertainty = f(altitude, pixel_error, attitude_error)
```

**Output per detection:**
```yaml
victim_id: 3
position_local: {x: 42.3, y: -18.7, z: 0.0}
confidence: 0.86
uncertainty_m: 4.5
source_image: frame_000421.png
```

**Upgrade path:**
- [ ] Flat ground assumption → Digital Elevation Model (DEM)
- [ ] Single observation → multi-view triangulation for higher accuracy
- [ ] Fixed uncertainty → covariance propagation through full transform chain

---

### 8. `5_output/reporting` — Mission Report Generator
**Method: Structured report from aggregated mission data**

Outputs after every mission:
- `mission_report.md` — human-readable summary
- `mission_report.json` — machine-readable full data
- `detections.csv` — all detections with positions and confidence
- `mission_map.geojson` — victim markers + flight path for mapping tools

**Report sections:**
```
Mission summary: area, duration, coverage %
Victim list: ID, position, confidence, image evidence  
Flight path: waypoints executed, deviations
Evaluation: vs ground truth if available
```

---

### 9. `5_output/evaluator` — Metrics
**Metrics computed after each mission:**

| Category    | Metric                                          |
| ----------- | ----------------------------------------------- |
| Coverage    | coverage_pct, unvisited_cells, overlap_pct      |
| Time        | mission_duration_s, time_to_first_detection_s   |
| Perception  | precision, recall, mAP@0.5, false_positives     |
| Geolocation | mean_error_m, median_error_m, error_vs_altitude |
| Planning    | path_length_m, smoothness_score, energy_proxy   |

These metrics are what make this a research-quality project, not just a demo.

---

## Tech Stack

| Layer                   | Technology                             |
| ----------------------- | -------------------------------------- |
| Flight control          | PX4 Autopilot (SITL)                   |
| Simulation              | Gazebo Harmonic                        |
| Robotics middleware     | ROS 2 Jazzy                            |
| PX4-ROS 2 bridge        | uXRCE-DDS (Micro XRCE-DDS Agent)       |
| Trajectory optimization | acados + CasADi                        |
| Human detection         | Ultralytics YOLOv8n / YOLO11n + SAHI   |
| Small object detection  | SAHI (Slicing Aided Hyper Inference)   |
| Coordinate transforms   | tf2                                    |
| Clustering              | DBSCAN (scikit-learn)                  |
| Containerization        | Docker + VS Code devcontainer          |
| Language                | Python 3.12 (nodes) + C++ (interfaces) |
| GPU                     | NVIDIA RTX 5070 (CUDA 13)              |

---

## Quick Start

```bash
# Clone and open in devcontainer
git clone https://github.com/YOUR_USERNAME/AeroSAR-Autonomous-Search-and-Rescue-Drone
code AeroSAR-Autonomous-Search-and-Rescue-Drone
# → VS Code: Reopen in Container

# Build workspace
cd ~/ws && colcon build --symlink-install

# Run full demo (after Gazebo + PX4 are running)
./scripts/run_demo.sh
```

---

## Repository Structure

```
src/
├── 1_interfaces/     # ROS 2 message + service definitions
├── 2_control/        # PX4 bridge + mission state machine
├── 3_autonomy/       # Coverage planner (MPC) + victim mapping
├── 4_perception/     # YOLO detection + pixel-to-world geolocation
└── 5_output/         # Mission reporting + evaluation metrics

sim/                  # Gazebo worlds, drone models, scenarios
docker/               # Dockerfile, compose, devcontainer
docs/                 # Architecture, methods, results per module
scripts/              # run_sim.sh, run_demo.sh, evaluate_mission.py
```

---

## Roadmap

- [x] Docker environment (ROS 2 Jazzy + Gazebo + PX4 deps)
- [x] ROS 2 workspace with all packages
- [ ] PX4 SITL ↔ ROS 2 bridge working
- [ ] Basic offboard flight (arm → takeoff → hover → land)
- [ ] Lawnmower coverage pattern flight
- [ ] Custom Gazebo disaster world with victim models
- [ ] YOLO detection node on camera stream
- [ ] Geolocation: pixel → world coordinates
- [ ] Victim registry with DBSCAN deduplication
- [ ] Mission report generation
- [ ] MPC planner with acados (vs lawnmower baseline)
- [ ] SAHI small-object detection (vs full-frame baseline)
- [ ] End-to-end demo with metrics

---

## References

- PX4 Autopilot: https://docs.px4.io
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy
- uXRCE-DDS: https://micro-xrce-dds.docs.eprosima.com
- Ultralytics YOLO: https://docs.ultralytics.com
- SAHI: https://github.com/obss/sahi
- acados: https://docs.acados.org
- CasADi: https://web.casadi.org
- SARD dataset: https://github.com/VisDrone/VisDrone-Dataset
- VisDrone dataset: http://aiskyeye.com