ros2 launch husky_panda_description sim_husky_panda.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 launch slam_toolbox online_async_launch.py   slam_params_file:=/home/rebellion/mobile_robotics/industrial_vaccum_robot/ivc_robot_ws/src/husky_nav2_slam/config/mapper_params_online_async.yaml ue_sim_time:=true

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

ros2 run xacro xacro panda_mounted_husky.urdf.xacro -o panda_mount_husky.urdf



./python.sh /home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_panda_description/scripts/husky_isaac.py

ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
"{filename: '/home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/husky_nav2_slam/map'}"

