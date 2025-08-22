ros2 launch husky_panda_description sim_husky_panda.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 launch slam_toolbox online_async_launch.py   slam_params_file:=/home/rebellion/mobile_robotics/industrial_vaccum_robot/ivc_robot_ws/src/husky_nav2_slam/config/mapper_params_online_async.yaml ue_sim_time:=true

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

ros2 run xacro xacro panda_mounted_husky.urdf.xacro -o panda_mount_husky.urdf



./python.sh /home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_panda_description/scripts/husky_isaac.py

ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
"{filename: '/home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_nav2_slam/map'}"

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true


ros2 launch slam_toolbox online_async_launch.py   slam_params_file:=/home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_nav2_slam/config/slamtoolbox_params_online_async.yaml use_sim_time:=true


ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:=/home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_nav2_slam/map/map_v3/map_v3.yaml\
  params_file:=/home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_nav2_slam/config/nav2_params.yaml


From your Husky SDF (snippet earlier + file):

Base footprint (base_link collisions):

Length ≈ 0.987 m

Width ≈ 0.571 m

Height ≈ 0.37–0.40 m (stack including sensor tower + velodyne mount)

Mass (base_link): 46.064 kg

Wheels:

Radius = 0.1651 m (~16.5 cm)

Width = 0.114 m (~11.4 cm)

4 wheels, skid-steer

From Panda SDF:

Link chain: 7 revolute joints + 2 finger joints

Reach (joint1 → flange): ~0.86 m

Base width: ~0.16 m

Height (arm upright): ~0.95 m

Weight (from inertial tags): ~18 kg (without gripper)

Finger joints: add ~0.08 m extension

From Panda Mounted on Husky:

Combined footprint:

Length ≈ 0.99 m (husky dominates)

Width ≈ 0.57 m (husky dominates)

Height:

Husky chassis + sensor tower = 0.40 m

Panda base mounted on top: adds 0.10–0.15 m

Panda upright reach ≈ 1.3 m total

Total weight:

Husky ≈ 46 kg

Panda ≈ 18 kg

Combined ≈ 64 kg (without payload/sensors)

Wheel size: same as husky — radius 0.165 m

✅ Summary Table

Robot	Length (m)	Width (m)	Height (m)	Mass (kg)	Wheel radius (m)
Husky	~0.99	~0.57	~0.40	~46	0.165
Panda arm	~0.16	~0.16	~0.95	~18	N/A
Husky + Panda	~0.99	~0.57	~1.30	~64	0.165




Sensors on the Husky + Panda Platform
1. 2D Planar Lidar

Model: Based on RPLidar S1

Mount: Front-bottom of Husky (at ~0.127 m height)

Specs (from SDF):

Type: gpu_ray (2D lidar)

Range: 0.3 – 25 m

Resolution: 0.01 m

Field of View: 360°

Samples: 1600 per scan

Update rate: 10 Hz

Noise: Gaussian (σ = 0.01)

👉 This lidar is used for 2D SLAM and obstacle detection. In real life, an RPLidar S1 is a budget-friendly 360° lidar with ~40 m max range.

2. 3D Lidar (Front Tower)

Model: Based on Ouster OS1-64 or Velodyne VLP-16

Mount: On sensor tower, front-top

Specs (from SDF):

Type: gpu_lidar

Range: 0.8 – 120 m

Resolution: 0.003 m

Horizontal FOV: 360°, 1024 samples

Vertical FOV: ~33° (–0.29 to +0.29 rad) with 64 channels

Update rate: 20 Hz

Noise: Gaussian (σ = 0.03)

👉 Used for 3D mapping, localization, obstacle avoidance in rough terrain.

3. Cameras on Husky

Front RGB-D Camera

Based on Intel RealSense D435

Mount: ~0.36 m above base_link

Specs:

RGB resolution: 320×240 (simulation scaled down, real is up to 1280×720)

Horizontal FOV: 87° (~1.52 rad)

Depth range: 0.1 – 10 m

Update rate: 30 Hz

Noise: Gaussian σ = 0.007

Use: Obstacle detection, visual SLAM, perception

Downward RGB-D Camera

Based on Intel RealSense D435

Mount: Tilted downwards (~30° pitch)

Specs: same as above (87° FOV, depth 0.1–10 m, RGB 320×240 in sim)

Use: Floor mapping, visual odometry, docking tasks

4. IMU Sensor

Type: Generic MEMS IMU (modeled in Gazebo)

Specs:

Orientation, Angular velocity, Linear acceleration

Update rate: 50 Hz

Noise:

Gyro: σ = 0.009 rad/s

Accel: σ = 0.021 m/s²

Includes bias and drift parameters

Use: Sensor fusion (EKF / robot_localization) with odometry + lidar

5. Panda Arm Camera (on Gripper)

Type: Intel RealSense D435 (modeled, not always included in default SDF)

Mount: On Panda’s wrist/gripper link

Specs (default RealSense):

RGB: up to 1280×720

Depth range: 0.1 – 10 m

FOV: 87° horizontal, 58° vertical

Update: ~30 Hz

Use: Manipulation tasks (pick-and-place, grasping, AR marker detection, etc.)

✅ Sensor Summary
Sensor	Model (Sim)	Range/FOV	Rate	Mounting Position	Purpose
2D Lidar	RPLidar S1 (gpu_ray)	0.3–25 m, 360°, 1600 samples	10Hz	Front-bottom	2D SLAM, obstacle detection
3D Lidar	Ouster OS1-64 / VLP16	0.8–120 m, 360°×33°, 64ch	20Hz	Top tower	3D mapping, navigation
Front Camera	Intel RealSense D435	RGB-D, 87° FOV, 0.1–10 m	30Hz	Front (0.36 m)	Obstacle perception, SLAM
Downward Cam	Intel RealSense D435	RGB-D, 87° FOV, 0.1–10 m	30Hz	Tilted downward	Floor mapping, odometry
IMU	MEMS (simulated)	Gyro σ=0.009, Acc σ=0.021	50Hz	Center of base	Sensor fusion (EKF)
Gripper Cam	Intel RealSense D435	RGB-D, 87° FOV, 0.1–10 m	30Hz	Panda wrist	Object detection, grasping