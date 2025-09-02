# moveit2_sim.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


# ---- helpers ---------------------------------------------------------------

TRUE_SET = {"1", "true", "yes", "on"}

def _to_bool(value: str) -> bool:
    return str(value).strip().lower() in TRUE_SET

def _find_param_type_issues(obj, path=""):
    """Return list of (key_path, typename, repr(value)) for illegal param leaf types."""
    allowed_scalars = (bool, int, float, str, bytes)
    issues = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            issues += _find_param_type_issues(v, f"{path}.{k}" if path else k)
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            issues += _find_param_type_issues(v, f"{path}[{i}]")
    else:
        if isinstance(obj, tuple):
            issues.append((path, "tuple", repr(obj)))
        elif not isinstance(obj, allowed_scalars) and obj is not None:
            issues.append((path, type(obj).__name__, repr(obj)))
    return issues


def _dump_param_type_issues(label, obj):
    issues = _find_param_type_issues(obj, path=label)
    if issues:
        print(f"\n[PARAM TYPE ERROR] Invalid parameter leaf types in {label}:")
        for p, t, v in issues:
            print(f"  - {p}: {t} -> {v}")
    else:
        print(f"[PARAM TYPE OK] {label}")


# ---- main node builder -----------------------------------------------------

def _make_nodes(context, *args, **kwargs):
    # Resolve CLI args to plain Python values
    use_sim_time = _to_bool(LaunchConfiguration("use_sim_time").perform(context))
    rviz_config_name = LaunchConfiguration("rviz_config").perform(context)

    # Package shares
    cfg_share = FindPackageShare("huskypanda_moveit_config").find("huskypanda_moveit_config")
    desc_share = None
    try:
        desc_share = FindPackageShare("husky_panda_description").find("husky_panda_description")
    except Exception:
        print("[WARN] husky_panda_description not found; will not set robot_description from file.")

    # Paths
    urdf_path = None
    if desc_share is not None:
        urdf_path_candidate = os.path.join(
            desc_share, "husky_panda_model", "panda_mounted_husky", "panda_mounted_husky.urdf.xacro"
        )
        if os.path.exists(urdf_path_candidate):
            urdf_path = urdf_path_candidate
        else:
            print(f"[WARN] URDF xacro missing: {urdf_path_candidate} (RViz may rely on /robot_description topic)")

    srdf_rel = "config/panda.srdf"
    ctrl_rel = "config/moveit_controllers.yaml"
    kin_rel  = "config/kinematics.yaml"
    jl_rel   = "config/joint_limits.yaml"
    controller_setting_path = os.path.join(cfg_share, "config", "controller_setting.yaml")
    if not os.path.exists(controller_setting_path):
        raise RuntimeError(f"Missing file: {controller_setting_path}")

    # Build MoveIt config (use package-relative paths where possible)
    builder = MoveItConfigsBuilder("panda_mounted_husky", package_name="huskypanda_moveit_config") \
        .robot_description_semantic(file_path=srdf_rel) \
        .robot_description_kinematics(file_path=kin_rel) \
        .joint_limits(file_path=jl_rel) \
        .trajectory_execution(file_path=ctrl_rel) \
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True) \
        .planning_pipelines(pipelines=["ompl"]) \
        .moveit_cpp(controller_setting_path)

    # Only set URDF if we actually have it on disk
    if urdf_path:
        builder = builder.robot_description(file_path=urdf_path)

    moveit_config = builder.to_moveit_configs()

    # Merge to a plain dict and add small overrides
    params = moveit_config.to_dict()

    # VERY IMPORTANT: remove embedded moveit_cpp from dict (it is a ParameterFile / tuple-like)
    params.pop("moveit_cpp", None)

    # Enforce correctly-typed values
    params["use_sim_time"] = bool(use_sim_time)
    params["planning_pipelines"] = ["ompl"]
    params["default_planning_pipeline"] = "ompl"

    ompl = params.setdefault("ompl", {})
    ompl["plugin"] = "ompl_interface/OMPLPlanner"
    # In MoveIt, adapters are commonly given as a single space-separated string, which is safest:
    ompl["request_adapters"] = (
        "default_planner_request_adapters/FixWorkspaceBounds "
        "default_planner_request_adapters/FixStartStateBounds "
        "default_planner_request_adapters/FixStartStateCollision "
        "default_planner_request_adapters/FixStartStatePathConstraints "
        "default_planner_request_adapters/AddTimeParameterization"
    )
    ompl.setdefault("start_state_max_bounds_error", 0.1)

    psm = params.setdefault("planning_scene_monitor", {})
    psm.setdefault("sensors", [])  # keep Octomap disabled for now

    # Guardrail: fail early with a readable message if anything illegal remains
    issues = _find_param_type_issues(params)
    if issues:
        print("\n[PARAM TYPE ERROR] Invalid parameter leaf types detected:")
        for p, t, v in issues:
            print(f"  - {p}: {t} -> {v}")
        raise RuntimeError("Aborting launch due to invalid parameter types (see list above).")
    
    # Right before you construct the nodes:
    _dump_param_type_issues("move_group.params", params)

    # RViz bundles:
    _dump_param_type_issues("rviz.robot_description", moveit_config.robot_description)
    _dump_param_type_issues("rviz.robot_description_semantic", moveit_config.robot_description_semantic)
    _dump_param_type_issues("rviz.planning_pipelines", moveit_config.planning_pipelines)
    _dump_param_type_issues("rviz.robot_description_kinematics", moveit_config.robot_description_kinematics)
    _dump_param_type_issues("rviz.joint_limits", moveit_config.joint_limits)

    # Nodes
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            params,
            # moveit_config.moveit_cpp,  # pass ParameterFile separately to avoid tuple embedding
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("huskypanda_moveit_config"),
        "launch",
        rviz_config_name,
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        # Supplying the core robot params helps RViz displays come up immediately
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return [rviz_node, move_group_node]


# ---- launch description ----------------------------------------------------

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulated time from /clock"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Start RViz"),
        DeclareLaunchArgument("rviz_config", default_value="moveit.rviz", description="RViz configuration file"),
        OpaqueFunction(function=_make_nodes),
    ])







# import os
# import yaml
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, OpaqueFunction
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch.conditions import IfCondition
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder
# from launch_ros.parameter_descriptions import ParameterFile


# def generate_launch_description():

#     use_sim_time_arg = DeclareLaunchArgument(
#         "use_sim_time",
#         default_value="true",
#         description="Use simulated time from /clock"
#     )

#     # Command-line arguments
#     rviz_config_arg = DeclareLaunchArgument(
#         "rviz_config",
#         default_value="moveit.rviz",
#         description="RViz configuration file",
#     )

#     arm_robot_sim_path = os.path.join(
#         get_package_share_directory('huskypanda_moveit_config'))
    
#     package_description = "husky_panda_description"
#     urdf_path = os.path.join(
#         FindPackageShare(package=package_description).find(package_description),
#         "husky_panda_model",
#         "panda_mounted_husky",
#         "panda_mounted_husky.urdf.xacro",
#     )

#     moveit_config = (
#         MoveItConfigsBuilder("panda_mounted_husky", package_name="huskypanda_moveit_config")
#         .robot_description(
#             file_path=urdf_path)
#         .robot_description_semantic(file_path="config/panda.srdf")
#         .planning_scene_monitor(
#             publish_robot_description=True, publish_robot_description_semantic=True
#         )
#         .trajectory_execution(file_path="config/moveit_controllers.yaml")
#         .moveit_cpp(arm_robot_sim_path + "/config/controller_setting.yaml")
#         .to_moveit_configs()
#     )
#     # Inject use_sim_time (and any small overrides) into a single params dict
#     params = moveit_config.to_dict()
#     params.pop("moveit_cpp", None)
#     # use sim time
#     params["use_sim_time"] = LaunchConfiguration("use_sim_time")

#     # --- Force correct planning-pipeline params (arrays, not strings) ---
#     params["planning_pipelines"] = ["ompl"]           # must be a string array
#     params["default_planning_pipeline"] = "ompl"      # single string

#     # Ensure per-pipeline sub-namespace exists
#     ompl = params.setdefault("ompl", {})
#     ompl["plugin"] = "ompl_interface/OMPLPlanner"

#     # In Jazzy, adapters are safest as a string array:
#     ompl["request_adapters"] = [
#         "default_planner_request_adapters/FixWorkspaceBounds",
#         "default_planner_request_adapters/FixStartStateBounds",
#         "default_planner_request_adapters/FixStartStateCollision",
#         "default_planner_request_adapters/FixStartStatePathConstraints",
#         "default_planner_request_adapters/AddTimeParameterization",
#     ]
#     # Optional but common:
#     ompl.setdefault("start_state_max_bounds_error", 0.1)

#     # If you don’t want Octomap right now, make 'sensors' an *empty list* (correct type):
#     psm = params.setdefault("planning_scene_monitor", {})
#     psm.setdefault("sensors", [])   # not a string
#     # --------------------------------------------------------------------

#     # Start the actual move_group node/action server
#     move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[params],
#         arguments=["--ros-args", "--log-level", "info"],
#     )

#     # RViz
#     rviz_base = LaunchConfiguration("rviz_config")
#     rviz_config = PathJoinSubstitution(
#         [FindPackageShare("huskypanda_moveit_config"), "launch", rviz_base]
#     )
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         arguments=["-d", rviz_config],
#         # parameters=[
#         #     # moveit_config.robot_description,
#         #     moveit_config.robot_description_semantic,
#         #     # moveit_config.planning_pipelines,
#         #     # moveit_config.robot_description_kinematics,
#         #     # moveit_config.joint_limits,
#         # ],
#     )

#     return LaunchDescription(
#         [
#             use_sim_time_arg,
#             rviz_config_arg,
#             rviz_node,
#             move_group_node,
#         ]
#     )