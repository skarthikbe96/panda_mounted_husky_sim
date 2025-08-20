from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch.actions import TimerAction


def generate_launch_description():
    desc = get_package_share_directory('panda_description')

    # world_path   = PathJoinSubstitution([desc, 'panda_model', 'panda_empty_world.sdf'])
    world_path   = PathJoinSubstitution([desc, 'panda_model', 'husky_warehouse_world.sdf'])
    urdf_xacro   = PathJoinSubstitution([desc, 'panda_model', 'panda_mounted_husky', 'panda_mounted_husky.urdf.xacro'])
    bridge_yaml  = PathJoinSubstitution([desc, 'config', 'bridge_husky_panda.yaml'])   
    rviz_config  = PathJoinSubstitution([desc, 'panda_model', 'panda_rviz.rviz'])

    # Let Gazebo find local models (folders that contain your model resources)
    model_search = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{desc}:{os.path.join(desc, 'panda_model')}:{os.path.join(desc, 'panda_model', 'panda_mounted_husky')}"
    )

    # xacro -> robot_description for RViz/robot_state_publisher
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_xacro]),
        value_type=str
    )
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '3', world_path],
        output='screen'
    )

    # ROS <-> Gazebo bridge via YAML config (do NOT pass mappings as arguments)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_yaml}],
        output='screen'
    )

    # Robot State Publisher (URDF to TF for RViz)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': True, 
                     'publish_frequency': 50.0,
                      'robot_description': robot_description}],
        output='screen'
    )

    # Joint sliders -> /joint_states_cmd
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': True}],
        # remappings=[('/joint_states', '/joint_states_cmd')],
        output='screen'
    )

    # Your single Python relay that republishes /joint_states_cmd -> /model/panda/joint/*/cmd_pos
    js2gz = Node(
        package='panda_description',        # <-- this package
        executable='husky_panda_bridge.py',          # <-- the script’s console entry name
        name='husky_panda_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    tf_base_husky_base  = Node(
    package='tf2_ros', executable='static_transform_publisher',
    name='tf_base_to_gz_base',
    arguments=['0','0','0','0','0','0',
               'base_link',
               'panda_mounted_husky/husky/base_link'],
    output='screen'
    )

    tf_gz_base_planar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_gz_base_to_planar_laser',
        arguments=['0','0','0','0','0','0',
                'panda_mounted_husky/husky/base_link',
                'panda_mounted_husky/husky/base_link/planar_laser'],
        output='screen'
    )

    tf_gz_base_front_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_gz_base_to_front_laser',
        arguments=['0','0','0','0','0','0',
                'panda_mounted_husky/husky/base_link',
                'panda_mounted_husky/husky/base_link/front_laser'],
        output='screen'
    )

    # RViz (optional config file)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        model_search,
        gz,
        bridge,
        TimerAction(period=0.5, actions=[
            rsp, jsp, js2gz, rviz, tf_base_husky_base, tf_gz_base_planar, tf_gz_base_front_laser
        ]),

    ])
