from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    share_dir = get_package_share_directory("huskypanda_moveit_config")
    share_dir_description = get_package_share_directory("husky_panda_description")

    urdf_path = os.path.join(share_dir_description, "husky_panda_model","panda_mounted_husky", "panda_mounted_husky.urdf.xacro")
    srdf_path = os.path.join(share_dir, "config", "panda_mounted_husky.srdf")
    kin_path  = os.path.join(share_dir, "config", "kinematics.yaml")
    jl_path   = os.path.join(share_dir, "config", "joint_limits.yaml")
    ctrl_path = os.path.join(share_dir, "config", "moveit_controllers.yaml")

    # moveit_config = (
    #     MoveItConfigsBuilder(robot_name="panda_mounted_husky", package_name="huskypanda_moveit_config")
    #     .robot_description(file_path=urdf_path)
    #     .robot_description_semantic(file_path=srdf_path)
    #     .robot_description_kinematics(file_path=kin_path)
    #     .joint_limits(file_path=jl_path)
    #     .trajectory_execution(file_path=ctrl_path)
    #     .planning_pipelines(pipelines=["ompl"])  # Correct usage - only pipelines parameter
    #     .to_moveit_configs()
    # )

    moveit_cpp_config = {
        "planning_scene_monitor": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
            "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
            "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
        "planning_pipelines": {
            "pipeline_names": ["ompl"]
        },
        "plan_request_params": {
            "planning_attempts": 1,
            "planning_pipeline": "ompl",
            "max_velocity_scaling_factor": 0.2,
            "max_acceleration_scaling_factor": 0.2
        },
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": ["default_planning_request_adapters/ResolveConstraintFrames",
                            "default_planning_request_adapters/ValidateWorkspaceBounds",
                            "default_planning_request_adapters/CheckStartStateBounds",
                            "default_planning_request_adapters/CheckStartStateCollision"],
            "response_adapters": ["default_planning_response_adapters/AddTimeOptimalParameterization",
                             "default_planning_response_adapters/ValidateSolution",
                             "default_planning_response_adapters/DisplayMotionPath"],
            "start_state_max_bounds_error": 0.1
        },
        "octomap_frame": "base_link",
        "octomap_resolution": 0.05,
        "max_update_rate": 1.0,
       
    }

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="panda_mounted_husky", 
            package_name="huskypanda_moveit_config"
        )
        .robot_description(
            file_path=os.path.join(
                share_dir_description, 
                "husky_panda_model", 
                "panda_mounted_husky", 
                "panda_mounted_husky.urdf.xacro"
            )
        )
        .robot_description_semantic(file_path=srdf_path)
        .sensors_3d(file_path = os.path.join(
            get_package_share_directory("huskypanda_moveit_config"),
            "config/sensors_3d.yaml"))
        .to_moveit_configs()
    )


    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),moveit_cpp_config,
            {"use_sim_time": use_sim_time},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="huskypanda_moveit_config",
        executable="panda_arm_controller.py",
        output="both",
        parameters=[
            moveit_config.to_dict(),moveit_cpp_config,
            {"use_sim_time": True},
        ],
    )

    rviz_config = os.path.join(share_dir, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.to_dict(),moveit_cpp_config,
            {"use_sim_time": True},
        ],
    )

    tf_world_link_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world_to_odom",
        arguments=["--x","0","--y","0","--z","0",
                "--roll","0","--pitch","0","--yaw","0",
                "--frame-id","world_link", "--child-frame-id","odom"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use /clock from simulator'
        ),
        move_group_node,
        # moveit_py_node,
        rviz_node,
        tf_world_link_odom,
    ])