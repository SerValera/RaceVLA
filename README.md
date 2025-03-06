# RaceVLA

The repository is currently under preparation for publication.
Arxiv link: [http://arxiv.org/abs/2503.02572]

The source code will be uploaded soon!


## System Overview

The project integrates:

- **Visual-Inertial Odometry (VIO)** for state estimation using [open_vins](https://github.com/rpng/open_vins)
- **FishEye T265** with Intel RealSense [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
- **Flight control** using **ArduPilot firmware (v4.4.4)** running on a **SpeedyBee F4 V4 flight controller**.
- **ROS 1** for data handling, visualization, and control logic.
- **Flask** for transferring data between the drone and a remote server.
---

## Tested Environment

- **Platform:** Intel NUC
- **OS:** Ubuntu 22.04
- **Flight Controller:** SpeedyBee STM32F405 ARM
- **Firmware:** ArduPilot v4.4.4
- **Middleware:** ROS 1 (Noetic)
- **Communication Framework:** Flask (Python-based server)

---

## Project Components

### 1. Visual-Inertial Odometry
- OpenVINS provides real-time visual-inertial state estimation.
- Repository: [https://github.com/rpng/open_vins](https://github.com/rpng/open_vins)

### 2. FishEye T265
- Intel RealSense T265 camera for VLA vision.
- ROS integration via realsense-ros.
- Repository: [https://github.com/IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros)

### 3. Flight Control
- ArduPilot firmware for vehicle stabilization and high-level commands.
- ROS integration via `ardupilot_ros` for bidirectional communication.
- Documentation: [https://ardupilot.org/dev/docs/ros.html](https://ardupilot.org/dev/docs/ros.html)
- Repository: [https://github.com/ArduPilot/ardupilot_ros](https://github.com/ArduPilot/ardupilot_ros)

### 4. Data Transfer Between Drone and Server
- **Flask** is used to transfer telemetry, images, or processed data between the drone's companion computer and an external server.
- Flask Documentation: [https://flask.palletsprojects.com/en/stable/](https://flask.palletsprojects.com/en/stable/)

---

## Hardware Setup

### Flight Controller
- **SpeedyBee F405 (STM32-based)**
- Firmware: **ArduPilot v4.4.4**

### Companion Computer
- **Intel NUC**
- Runs ROS and processes data from the RealSense and VIO system.

### Sensors
- **Intel RealSense T265** (or compatible) for fisheye frame.
- IMU and camera used by OpenVINS for state estimation.

---
