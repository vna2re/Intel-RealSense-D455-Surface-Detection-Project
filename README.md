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
---
## 📷 Launching the RealSense D455

Start the Intel RealSense D455 camera with RGB-D and IMU data streams:

```bash
ros2 launch realsense2_camera rs_launch.py \
  color_width:=640 color_height:=480 \
  depth_width:=640 depth_height:=480 depth_fps:=30 depth_max:=2.0 \
  enable_gyro:=true enable_accel:=true unite_imu_method:=1 \
  pointcloud.enable:=true align_depth.enable:=true
```

| Parameter                              | Description                                                                                                           |
| -------------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| `color_width:=640 color_height:=480`   | Sets the resolution of the RGB (color) stream to **640×480** pixels.                                                  |
| `depth_width:=640 depth_height:=480`   | Sets the resolution of the depth stream to **640×480** — must match color resolution for alignment.                   |
| `depth_fps:=30`                        | Frame rate of the depth stream (**30 FPS**) for real-time operation.                                                  |
| `depth_max:=2.0`                       | Maximum valid depth distance (in meters). Points farther than **2 m** are ignored.                                    |
| `enable_gyro:=true enable_accel:=true` | Enables the **gyroscope** and **accelerometer** sensors in the D455.                                                  |
| `unite_imu_method:=1`                  | Fuses accelerometer and gyroscope data into a single IMU topic `/imu/data`. Method `1` uses **linear interpolation**. |
| `pointcloud.enable:=true`              | Publishes a **3D point cloud** generated from the depth image.                                                        |
| `align_depth.enable:=true`             | Aligns the depth image to the color frame — ensures both have the same pixel correspondence.                          |


This launch configuration provides:

- Aligned RGB-D streams (ready for SLAM or mapping)

- IMU fusion (accelerometer + gyroscope)

- 3D point cloud publishing for visualization or mapping (e.g. RTAB-Map, RViz)
---
## 🧮 IMU Fusion

Run the Madgwick filter to fuse raw IMU data (gyroscope + accelerometer) from the RealSense D455 and output orientation in quaternion form:

```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args \
  -r imu/data_raw:=/camera/camera/imu \
  -p use_mag:=false \
  -p publish_tf:=true \
  -p frame_id:=camera_link \
  -p fixed_frame:=camera_link
```

| Parameter                             | Description                                                                                                               |
| ------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| `-r imu/data_raw:=/camera/camera/imu` | Remaps the raw IMU topic from the RealSense camera to the input expected by the Madgwick filter.                          |
| `-p use_mag:=false`                   | Disables the magnetometer (the D455 doesn’t provide magnetometer data). The filter will rely only on gyro and accel data. |
| `-p publish_tf:=true`                 | Publishes a **TF transform** representing the orientation of the IMU in the ROS TF tree.                                  |
| `-p frame_id:=camera_link`            | Sets the IMU frame name to `camera_link` — this ensures consistency with RTAB-Map’s expected reference frame.             |
| `-p fixed_frame:=camera_link`         | Defines a fixed reference frame for orientation output (used when publishing TF).                                         |

Result:

This node:

- Subscribes to /camera/camera/imu (raw IMU data)

- Publishes filtered orientation as /imu/data

- Adds an orientation TF between camera_link and odom

- Provides fused inertial data that improves pose estimation in RTAB-Map for visual-inertial SLAM
