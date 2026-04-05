# PiBot Navigation System

This repository contains the ROS 2 workspace for the **PiBot** project, an autonomous mobile robot platform focusing on advanced path-planning and local obstacle avoidance algorithms.

**Associated Research Paper**


## 🛠 Hardware & System Architecture

The robot operates on **ROS 2 Humble** using a **Raspberry Pi 4** as its main computation unit.
- **Sensors:** RPLidar A1M8 (2D LiDAR), MPU6050 (IMU)
- **Mapping:** SLAM Toolbox
- **Localization:** AMCL + Extended Kalman Filter (robot_localization)
- **Global Planner:** Custom **Bi-directional Ant Colony Optimization (Bi-ACO)** 
- **Local Planner:** Custom **Adaptive Dynamic Window Approach (DWA)** 
- **Obstacle Detection:** obstacle_detector_2

## 📂 Packages Overview

- `pibot_bringup`: Main launch files to start the physical robot (`pibot_real.launch.py`), sensors, and core nav nodes.
- `pibot_controller`: Hardware interface and controller configurations.
- `pibot_description`: Robot URDF/Xacro models.
- `pibot_globalplan`: Implementation of the Bi-ACO Nav2 Global Planner.
- `pibot_localplan`: Implementation of the DWA Nav2 Local Planner controller.
- `pibot_localization`: Tuning configurations for EKF and AMCL.
- `pibot_mappping`: Tuning configurations for SLAM Toolbox.
- `obstacle_detector_2`: Advanced dynamic obstacle detection processing.

## 🚀 Building & Installation

1. **Install ROS 2 Humble** on your Ubuntu 22.04 system.
2. **Clone the repository:**
   ```bash
   mkdir -p ~/pibot_ws
   cd ~/pibot_ws
   git clone https://github.com/Manh12asd/pibot_ws.git src
   ```
3. **Install dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
4. **Build the workspace:**
   ```bash
   colcon build --symlink-install
   ```
5. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## 🏃 Usage (Example)
Launch the main robot stack on the real hardware:
```bash
ros2 launch pibot_bringup pibot_real.launch.py
```


