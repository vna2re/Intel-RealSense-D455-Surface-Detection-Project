# Intel-RealSense-D455-Surface-Detection-Project

## 🧭 Overview

The system performs the following steps:

1. **Capture RGB-D and IMU data** from the Intel RealSense D455 camera.  
2. **Fuse visual and inertial data** using the Madgwick filter for robust odometry.  
3. **Generate a 3D map** in **RTAB-Map** and export it as a `.db` or `.ply` file.  
4. **Process the saved map** in Python with **Open3D** to detect and visualize the floor surface.

---

## ⚙️ System Requirements

- Ubuntu 22.04 (recommended)
- ROS 2 Humble
- Intel RealSense D455
- Python 3.10+
- Open3D ≥ 0.18
- RTAB-Map ≥ 0.20

---

## 🧩 Installation

### 1. Install ROS 2 and dependencies

Install required ROS 2 packages:

```bash
sudo apt install ros-humble-realsense2-camera ros-humble-rtabmap-ros \
                 ros-humble-imu-filter-madgwick python3-pip
```
### 2. Install Python dependencies
```bash
pip install "numpy<2.0" open3d --break-system-packages
```
## 📷 Launching the RealSense D455

Start the RealSense D455 with RGB-D and IMU data:

```bash
ros2 launch realsense2_camera rs_launch.py \
  color_width:=640 color_height:=480 \
  depth_width:=640 depth_height:=480 depth_fps:=30 depth_max:=2.0 \
  enable_gyro:=true enable_accel:=true unite_imu_method:=1 \
  pointcloud.enable:=true align_depth.enable:=true
```

This enables:

- RGB and depth streams (aligned)

- IMU data (accelerometer + gyroscope)

- 3D point cloud publishing
