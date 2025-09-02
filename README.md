# Husky + Panda Mobile Manipulation (ROS 2)

End-to-end setup for simulating and operating a Clearpath **Husky** base with a **Franka Emika Panda** arm in ROS 2, including SLAM (slam_toolbox), navigation (Nav2), teleop, URDF/Xacro generation, and map serialization.

---

## ✨ Features

- Gazebo-based simulation of Husky + Panda stack
- 2D/3D LiDAR, RGB-D cameras, and IMU sensors
- Online SLAM with slam_toolbox (async)
- Nav2 bringup for autonomous navigation
- Keyboard teleoperation
- Map/pose-graph serialization
- Reusable Xacro/URDF pipeline

---

## 📦 Requirements

- ROS 2 (Humble or newer recommended)
- Gazebo / Gazebo (Ignition) installed
- `nav2_bringup`, `slam_toolbox`, `teleop_twist_keyboard`, `xacro`
- Colcon workspace (paths below assume your layout)

> Adjust file paths to your environment as needed.

---

## 🧱 Build

```bash

rosdep install --from-paths src --rosdistro jazzy --ignore-src -r -y


# From your colcon workspace root
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Quickstart

Open **separate terminals**, source your workspace in each (`source install/setup.bash`), then:

### 1) Start Simulation
```bash
ros2 launch husky_panda_description sim_husky_panda.launch.py
```

### 2) (Optional) Generate URDF from Xacro
```bash
ros2 run xacro xacro panda_mounted_husky.urdf.xacro -o panda_mount_husky.urdf
```

### 3) Keyboard Teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

latest:

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true \
  -r /cmd_vel:=/husky_velocity_controller/cmd_vel

```

### 4) SLAMTool Box (Online, Async)
Example A (IVC workspace):
```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_nav2_slam/config/slamtoolbox_params_online_async.yaml  use_sim_time:=true
```


### 5) Nav2 (Navigation)
Basic bringup:
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=/home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_nav2_slam/config/nav2_params.yaml


```

With a saved map + custom params:
```bash
ros2 launch nav2_bringup bringup_launch.py   use_sim_time:=true   map:=/home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_nav2_slam/map/map_v3/map_v3.yaml   params_file:=/home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_nav2_slam/config/nav2_params.yaml
```

### 6) (Optional) Utility Script
```bash
./python.sh /home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_panda_description/scripts/husky_isaac.py
```

---

## 🗺️ Save SLAM Pose-Graph / Map

Serialize slam_toolbox’s pose graph to disk:
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_nav2_slam/map'}"
```

> Use the resulting files with your later Nav2 sessions (see bringup with `map:=...` above).

---

## Validating Xacro
```bash
ros2 run  xacro xacro /home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_panda_description/husky_panda_model/panda_mounted_husky/panda_model_description/panda.urdf.xacro > /tmp/panda_merged.urdf

check_urdf /tmp/panda_merged.urdf
```

## Validating SDF
```bash
gz sdf -k /home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_panda_description/husky_panda_model/husky_small_house_world.sdf
```
## Validating yaml file
```bash
ros2 run demo_nodes_cpp parameter_blackboard __node:=dummy

ros2 param load /dummy /home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_panda_description/config/ompl_planning.yaml

```


## controller_manager commands
```bash
ros2 control list_controllers

ros2 param get /controller_manager ros__parameters

ros2 control list_hardware_interfaces
```

## 📐 Platform Specs

### Husky Base (from SDF)
- **Length:** ≈ 0.987 m  
- **Width:** ≈ 0.571 m  
- **Height:** ≈ 0.37–0.40 m (with sensor tower/velodyne mount)  
- **Mass (base_link):** 46.064 kg  
- **Wheels:** 4 × skid-steer, **radius** 0.1651 m, **width** 0.114 m

### Panda Arm (from SDF)
- **Joints:** 7 revolute + 2 finger joints  
- **Reach (joint1 → flange):** ~0.86 m  
- **Base width:** ~0.16 m  
- **Arm height (upright):** ~0.95 m  
- **Mass (without gripper):** ~18 kg  
- **Finger extension:** ~0.08 m

### Combined (Mounted)
- **Footprint (L × W):** ≈ 0.99 m × 0.57 m (Husky dominates)
- **Total height:** Husky + tower ~0.40 m, + Panda base → ~1.30 m (upright reach)
- **Total mass:** ≈ 64 kg (no payload)
- **Wheel radius:** 0.165 m

#### Summary Table

| Robot            | Length (m) | Width (m) | Height (m) | Mass (kg) | Wheel radius (m) |
|------------------|------------|-----------|------------|-----------|------------------|
| Husky            | ~0.99      | ~0.57     | ~0.40      | ~46       | 0.165            |
| Panda arm        | ~0.16      | ~0.16     | ~0.95      | ~18       | N/A              |
| Husky + Panda    | ~0.99      | ~0.57     | ~1.30      | ~64       | 0.165            |

---

## 🧭 Sensor Suite

### 1) 2D Planar LiDAR
- **Model (sim):** RPLidar S1 (Gazebo `gpu_ray`)
- **Range:** 0.3–25 m  
- **Resolution:** 0.01 m  
- **FOV:** 360°, **samples:** 1600/scan  
- **Rate:** 10 Hz  
- **Noise:** Gaussian σ=0.01  
- **Mount:** Husky front-bottom (~0.127 m)  
- **Use:** 2D SLAM, obstacle detection

### 2) 3D LiDAR (Front Tower)
- **Model (sim):** Ouster OS1-64 / Velodyne VLP-16 (Gazebo `gpu_lidar`)  
- **Range:** 0.8–120 m  
- **Horiz. FOV:** 360° (1024 samples); **Vert. FOV:** ~33° (−0.29→+0.29 rad), 64 channels  
- **Rate:** 20 Hz  
- **Noise:** Gaussian σ=0.03  
- **Mount:** Top sensor tower  
- **Use:** 3D mapping/localization, obstacle avoidance

### 3) Cameras on Husky
**Front RGB-D (Intel RealSense D435, sim):**  
- RGB: 320×240 (sim; real up to 1280×720)  
- Depth range: 0.1–10 m  
- HFOV: ~87°  
- Rate: ~30 Hz  
- Noise: Gaussian σ=0.007  
- Mount: ~0.36 m above `base_link`  
- Use: Obstacle perception, visual SLAM

**Downward RGB-D:**  
- Same specs; pitched ~30° downward  
- Use: Floor mapping, docking, VO

### 4) IMU
- **Type:** Generic MEMS IMU (Gazebo)  
- **Outputs:** Orientation, angular velocity, linear acceleration  
- **Rate:** 50 Hz  
- **Noise:** Gyro σ=0.009 rad/s, Accel σ=0.021 m/s²  
- **Use:** EKF fusion with odom/LiDAR

### 5) Panda Gripper Camera (optional)
- **Model:** Intel RealSense D435 (sim)  
- **RGB:** up to 1280×720, depth 0.1–10 m, HFOV ~87°  
- **Rate:** ~30 Hz  
- **Mount:** On gripper link  
- **Use:** Manipulation/perception (grasping, markers)

#### Sensor Summary

| Sensor        | Model (Sim)              | Range / FOV                       | Rate | Mount              | Purpose                           |
|---------------|--------------------------|-----------------------------------|------|--------------------|------------------------------------|
| 2D LiDAR      | RPLidar S1 (`gpu_ray`)   | 0.3–25 m, 360°, 1600 samples      | 10Hz | Front-bottom       | 2D SLAM, obstacle detection       |
| 3D LiDAR      | OS1-64 / VLP-16          | 0.8–120 m, 360°×~33°, 64 ch       | 20Hz | Top tower          | 3D mapping, navigation            |
| Front RGB-D   | RealSense D435           | 0.1–10 m, ~87° HFOV               | 30Hz | ~0.36 m height     | Perception, visual SLAM           |
| Downward RGB-D| RealSense D435           | 0.1–10 m, ~87° HFOV               | 30Hz | Tilted ~30° down   | Floor mapping, VO                 |
| IMU           | MEMS (sim)               | Gyro σ=0.009, Acc σ=0.021         | 50Hz | Base center        | Sensor fusion (EKF)               |
| Gripper Cam   | RealSense D435 (optional)| 0.1–10 m, ~87° HFOV               | 30Hz | Panda gripper      | Object detection, grasping        |

---

## 🔧 Config Files (examples)

- **SLAMTOOL BOX (mapper):**  
  - `/husky_nav2_slam/config/slamtoolbox_params_online_async.yaml`
- **Nav2 params:**  
  - `/husky_nav2_slam/config/nav2_params.yaml`
- **Maps:**  
  - `/husky_nav2_slam/map/map_v3/map_v3.yaml`

> Tune these to your robot namespace, frame ids, and topic names.

---

## 🧩 Common Topics (typical)

- `/cmd_vel` (geometry_msgs/Twist)  
- `/odom`, `/odometry/filtered`  
- `/scan` (2D LiDAR), `/points` or `/points_raw` (3D LiDAR)  
- `/camera/*/image_raw`, `/camera/*/depth`  
- `/imu/data`  
- `/tf`, `/tf_static`  
- `/map`, `/map_metadata`, `/amcl_pose` (when localized)

*Actual topic names may differ based on your launch files and namespaces.*

---

## 🧷 Tips & Troubleshooting

- **Sim time:** Ensure `use_sim_time:=true` for **both** SLAM and Nav2 when running in Gazebo.  
- **TF frames:** Verify base/laser/camera frames in your URDF/Xacro match your config (e.g., `base_link`, `laser_link`, `camera_link`).  
- **SLAM params:** If no map builds, reduce `resolution` / increase `max_laser_range` and confirm the `/scan` topic remap.  
- **Nav2 bringup:** If costmaps stay empty, check the sensor topics and `obstacle_layer` topic filters.  
- **Teleop:** Terminal focus must remain on the teleop window for key input to register.

---

## 🗂️ Repo Structure (suggested)

```
husky_panda_description/
  launch/
    sim_husky_panda.launch.py
  urdf/
    panda_mounted_husky.urdf.xacro
  scripts/
    husky_isaac.py
husky_nav2_slam/
  config/
    mapper_params_online_async.yaml
    slamtoolbox_params_online_async.yaml
    nav2_params.yaml
  map/
    map_v3/
      map_v3.yaml
```

---

## 📜 License

Add your license here (e.g., Apache-2.0 / BSD-3-Clause).

---

## 🙌 Acknowledgements

Clearpath Husky, Franka Emika Panda, Nav2, slam_toolbox, Gazebo, and the ROS 2 community.