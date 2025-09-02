#!/usr/bin/env -S ros2 launch
"""Launch Husky with mounted Panda arm in Gazebo + RViz2 (without MoveIt)"""

import os
from typing import List
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction


def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Launch configs
    rviz_config = LaunchConfiguration("rviz_config")
    bridge_yaml = LaunchConfiguration("bridge_yaml")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    # Resolve package path
    pkg_husky_panda = get_package_share_directory("husky_panda_description")

    # Gazebo resource path
    model_search = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=f"{pkg_husky_panda}/husky_panda_model:{pkg_husky_panda}/husky_panda_model/panda_mounted_husky:{pkg_husky_panda}/husky_panda_model/panda_mounted_husky/meshes"
    )

    # Gazebo simulation
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-v", "3", world],
        output="screen",
    )

    # Spawn robot into Gazebo (SDF only)
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file", os.path.join(pkg_husky_panda, "husky_panda_model", "panda_mounted_husky","model.sdf"),
            # "-name", "husky_panda",
            "-x", "0.0", "-y", "0.0", "-z", "0.5",
        ],
    )

    # Robot State Publisher (URDF → RViz visualization)
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": ParameterValue(
                Command([
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution([
                        FindPackageShare("husky_panda_description"),
                        "husky_panda_model",
                        "panda_mounted_husky",
                        "panda_mounted_husky.urdf.xacro"
                    ])
                ]),
                value_type=str,
            )
        }],
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config, "--ros-args", "--log-level", log_level],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": use_sim_time}, {"config_file": bridge_yaml}],
        output="screen",
    )

    # # ros2_control node
    # ros2_control = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[{"use_sim_time": use_sim_time},
    #                 os.path.join(pkg_husky_panda, "config", "ros2_controllers.yaml")],
    #     output="screen",
    # )

    # Controller spawners

    twist_bridge = Node(
        package='husky_panda_description',     # your package name
        executable='twist_to_stamped.py',         # entry point from setup.py
        name='twist_to_stamped_bridge',
        output='screen',
        parameters=[{
            'in_topic': '/cmd_vel',
            'out_topic': '/husky_velocity_controller/cmd_vel',
            'frame_id': '',            # or 'base_link' if your controller expects it
            'use_sim_time': True,      # set False if you’re not using /clock
        }],
    )

    controllers = [
        ExecuteProcess(
            cmd=["ros2 run controller_manager spawner {}".format(ctrl)],
            shell=True,
            output="screen",
        )
        for ctrl in [
            "joint_state_broadcaster",
            "husky_velocity_controller",
            "panda_arm_controller",
            "panda_hand_controller",
            "gimbal_controller",
        ]
    ]

    # Delay controller spawning by 5 seconds
    delayed_controllers = TimerAction(
        period=10.0,
        actions=controllers,
    )

    tf_base_husky_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_gz_base',
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "panda_mounted_husky/base_link",
                ],
        output='screen'
    )

    tf_gz_base_planar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_gz_base_to_planar_laser",
        arguments=[
            "--x", "0.474", "--y", "0.0", "--z", "0.127",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "panda_mounted_husky/base_link",
            "--child-frame-id", "panda_mounted_husky/base_link/planar_laser",
        ],
        output="screen",
    )

    tf_gz_base_front_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_gz_base_to_front_laser",
        arguments=[
            "--x", "0.424", "--y", "0.0", "--z", "0.387",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "panda_mounted_husky/base_link",
            "--child-frame-id", "panda_mounted_husky/base_link/front_laser",
        ],
        output="screen",
    )

    tf_gz_base_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_gz_base_to_imu",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "panda_mounted_husky/base_link",
            "--child-frame-id", "panda_mounted_husky/base_link/imu_sensor",
        ],
        output="screen",
    )


    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            LaunchConfiguration("ekf_yaml")
        ],
        # remappings=[("/odometry/filtered", "/odom")]
    )

    return LaunchDescription(
        declared_arguments +
        [model_search, gazebo, spawn_entity, robot_state_pub, rviz, bridge, ekf_node, delayed_controllers,tf_base_husky_base,twist_bridge, tf_gz_base_planar, tf_gz_base_front_laser, tf_gz_base_imu]
    )


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """Generate list of all launch arguments"""
    pkg_husky_panda = get_package_share_directory("husky_panda_description")
    pkg_nav2 = get_package_share_directory("husky_nav2_slam")
    return [
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(pkg_husky_panda, "husky_panda_model", "husky_small_house_world.sdf"),
            description="World file to load in Gazebo.",
        ),
        DeclareLaunchArgument(
            "bridge_yaml",
            default_value=os.path.join(pkg_husky_panda, "config", "bridge_husky_panda.yaml"),
            description="YAML config for ros_gz_bridge.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(pkg_husky_panda, "rviz", "husky_panda_rviz.rviz"),
            description="RViz2 config file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated clock if true.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="Logging level for all launched nodes.",
        ),
        DeclareLaunchArgument(
            "ekf_yaml",
            default_value=os.path.join(pkg_nav2, "config", "ekf.yaml"),
            description="EKF parameters for robot_localization.",
        ),
    ]
