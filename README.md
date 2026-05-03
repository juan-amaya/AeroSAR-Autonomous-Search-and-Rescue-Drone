# AeroSAR — Autonomous Search and Rescue Drone

<p align="center">
  <img src="media/demo.gif" alt="AeroSAR Demo" width="700"/>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Jazzy-blue" />
  <img src="https://img.shields.io/badge/PX4-SITL-orange" />
  <img src="https://img.shields.io/badge/Gazebo-Harmonic-green" />
  <img src="https://img.shields.io/badge/Python-3.12-yellow" />
  <img src="https://img.shields.io/badge/Docker-ready-blue" />
</p>

---

## Motivation

Every minute counts in a search and rescue operation. Ground teams are slow, search areas are large, and disasters leave terrain unpredictable. Drones offer speed and aerial perspective, but today they still depend on a human pilot scanning a video feed, which is exhausting, error-prone, and does not scale.

The goal is a drone that can be handed a search area, fly it autonomously, detect survivors using onboard vision, estimate their locations, and return a structured mission report autpnomously.

---

## What it does

AeroSAR is a simulation-first autonomous SAR system. A single command launches a full mission:

1. The drone takes off and executes a systematic coverage flight over a defined disaster area
2. An onboard YOLO model detects humans in the camera feed in real time
3. Detected victims are geolocated using camera geometry and drone pose
4. Duplicate detections are merged and a victim map is built during flight
5. On landing, a structured mission report is generated with victim positions and evidence

Everything runs in simulation on ROS 2 + PX4 SITL + Gazebo, making it reproducible and hardware-independent.

---

## Goals

- Build a complete autonomous UAV pipeline from flight control to mission reporting
- Demonstrate modular robotics software architecture following ROS 2 conventions
- Implement and compare classical and advanced methods at each stage:
  - Lawnmower coverage vs MPC trajectory optimization (acados)
  - Full-frame YOLO detection vs SAHI sliced inference for small aerial targets
- Produce measurable, comparable results — not just a demo

---

## Tech Stack

|            |                               |
| ---------- | ----------------------------- |
| Simulation | PX4 SITL + Gazebo Harmonic    |
| Robotics   | ROS 2 Jazzy                   |
| Detection  | YOLOv8 + SAHI                 |
| Trajectory | acados + CasADi (MPC)         |
| Container  | Docker + VS Code devcontainer |
| GPU        | CUDA 13 / RTX 5070            |

---

## Quick Start

```bash
git clone https://github.com/YOUR_USERNAME/AeroSAR-Autonomous-Search-and-Rescue-Drone
code AeroSAR-Autonomous-Search-and-Rescue-Drone
# → Reopen in Container
cd ~/ws && colcon build --symlink-install
./scripts/run_demo.sh
```

---

## Status

> Active development — simulation environment complete, flight control in progress.

---

## Documentation

Detailed method descriptions for each module are in [`docs/`](docs/).

--
## Author

**Juan Amaya** — Electrical Engineering, TU Munich  
[GitHub](https://github.com/juan-amaya) · [LinkedIn](www.linkedin.com/in/juan-diego-amaya-cueva-177aa5281)