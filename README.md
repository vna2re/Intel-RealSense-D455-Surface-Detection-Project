# Intel-RealSense-D455-Surface-Detection-Project

---

## 🧭 Overview

The system performs:

1. **3D Mapping** — Capturing RGB-D data using Intel RealSense D455.
2. **SLAM Processing** — Building a consistent map with RTAB-Map using RGB, depth, and IMU data.
3. **Map Export** — Saving the generated 3D map as a `.db` and `.ply` file.
4. **Floor Detection** — Identifying the ground plane from the reconstructed point cloud using RANSAC-based segmentation.

---

## ⚙️ System Configuration

- **Operating System:** Ubuntu 22.04 LTS  
- **ROS 2 Distribution:** Humble Hawksbill  
- **Camera:** Intel RealSense D455  
- **Libraries:** RTAB-Map, Open3D, NumPy  

---

## 📸 1. Launching the RealSense D455 Camera

Start the RGB-D stream, depth alignment, and IMU sensors:

```bash
ros2 launch realsense2_camera rs_launch.py \
  color_width:=640 color_height:=480 \
  depth_width:=640 depth_height:=480 \
  depth_fps:=30 depth_max:=2.0 \
  enable_gyro:=true enable_accel:=true \
  unite_imu_method:=1 \
  pointcloud.enable:=true \
  align_depth.enable:=true
