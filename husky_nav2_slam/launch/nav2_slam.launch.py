# husky_nav2_slam/launch/nav2_slam.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share   = get_package_share_directory('husky_nav2_slam')
    nav2_share  = get_package_share_directory('nav2_bringup')
    nav2_nav_launch = os.path.join(nav2_share, 'launch', 'navigation_launch.py')

    # Launch args
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic   = LaunchConfiguration('scan_topic')
    params_file  = LaunchConfiguration('params_file')

    slam_params = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use /clock from simulator'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/world/default/model/husky/link/base_link/sensor/planar_laser/scan',
            description='2D lidar scan topic for SLAM'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Path to Nav2 parameters YAML'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_static_tf',
            arguments=['0.474', '0', '0.127', '0', '0', '0', 'base_link', 'husky/base_link/planar_laser'],
            output='screen'
        ),

        # SLAM Toolbox – builds /map and publishes map->odom
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('scan', scan_topic)],
        ),

        # SLAM Toolbox — publishes map->odom and /map
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}],
            remappings=[('scan', scan_topic)],
        ),

        # Nav2 servers (planner/controller/bt/etc), no map_server/amcl
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_nav_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file' : params_file,
                'autostart'   : 'true',
                "use_collision_monitor": "false"
            }.items(),
        ),
    ])
