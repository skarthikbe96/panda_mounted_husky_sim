from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # sensors_yaml = os.path.join(
    #     get_package_share_directory("huskypanda_moveit_config"),
    #     "config",
    #     "sensors_3d.yaml",
    # )

    moveit_config = (
        MoveItConfigsBuilder("panda_mounted_husky", package_name="huskypanda_moveit_config")
        .robot_description(file_path=os.path.join(get_package_share_directory("husky_panda_description"),"husky_panda_model","panda_mounted_husky","panda_mounted_husky.urdf.xacro"))
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])  # this is enough
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()

    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}, {"publish_robot_description_semantic": True},
            os.path.join(
                get_package_share_directory("huskypanda_moveit_config"),
                "config",
                "ompl_planning.yaml"               
                ),
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )

    rviz_config = os.path.join(get_package_share_directory("huskypanda_moveit_config"), "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use /clock from simulator'
        ),
        move_group_node,
        rviz_node
           ])