from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('panda_description')

    # Your files
    urdf_xacro = os.path.join(pkg, 'panda_model', 'panda_model_description', 'panda.urdf.xacro')
    world_sdf  = os.path.join(pkg, 'panda_model', 'panda_empty_world.sdf')
    model_root = os.path.join(pkg, 'panda_model')
    model_dir  = os.path.join(model_root, 'panda_model_description')  # contains model.sdf (+ model.config)
    rviz_config = os.path.join(pkg, 'panda_model', 'panda_rviz.rviz')

    # Resource paths for Gazebo (preserve existing values)
    gz_res_paths = os.pathsep.join([p for p in [
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        pkg,            # for package://panda_description/…
        model_root,     # …/panda_model
        model_dir,      # …/panda_model_description (meshes, model.sdf)
    ] if p])

    ign_res_paths = os.pathsep.join([p for p in [
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        pkg, model_root, model_dir
    ] if p])

    set_gz_path  = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_res_paths)
    set_ign_path = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_res_paths)

    # URDF -> /robot_description (for RViz, TF)
    robot_description = Command([FindExecutable(name='xacro'), ' ', urdf_xacro])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str),
                     'use_sim_time': True}],
        remappings=[('joint_states', '/joint_states_cmd')],
        output='screen'
    )

    # Gazebo with your SDF world (Panda already included there)
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '3', world_sdf],
        output='screen'
    )

    # RViz shows the URDF
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True,
                     'robot_description': ParameterValue(robot_description, value_type=str)}],
        output='screen',
        arguments=['-d', rviz_config],
    )

    # Sliders publish /joint_states_cmd
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        remappings=[('/joint_states', '/joint_states_cmd')],
        output='screen'
    )

    # Convert /joint_states_cmd -> per-joint GZ command topics
    js_to_gz = Node(
        package='panda_description',
        executable='js_to_gz_cmd.py',
        output='screen'
    )

    # Joint / camera bridges (adjust if your SDF uses different topic names)
    bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    output='screen',
    arguments=[
        '/model/panda/joint/panda_joint1/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
        '/model/panda/joint/panda_joint2/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
        '/model/panda/joint/panda_joint3/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
        '/model/panda/joint/panda_joint4/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
        '/model/panda/joint/panda_joint5/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
        '/model/panda/joint/panda_joint6/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
        '/model/panda/joint/panda_joint7/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
        '/model/panda/joint/panda_finger_joint1/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
        '/model/panda/joint/panda_finger_joint2/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
        '/panda/wrist/rgbd/image@sensor_msgs/msg/Image@gz.msgs.Image',
        '/panda/wrist/rgbd/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        '/panda/wrist/rgbd/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
        # If your rgbd sensor publishes a point cloud (some builds do):
        '/panda/wrist/rgbd/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
    ],
    )


    return LaunchDescription([
        set_gz_path, set_ign_path,
        rsp, gz, bridge, rviz, jsp_gui, js_to_gz
    ])
