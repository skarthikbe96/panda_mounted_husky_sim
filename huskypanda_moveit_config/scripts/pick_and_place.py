#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown

ARM_GROUP = "panda_arm"       # SRDF group for the arm
HAND_GROUP = "panda_hand"     # SRDF group for the gripper
EE_LINK   = "panda_link8"     # tool link (flange)
FRAME     = "panda_link0"     # planning frame (fixed to Husky chassis via your mount)

def make_pose(x, y, z, roll=0.0, pitch=math.pi, yaw=0.0, frame=FRAME):
    # RPY->Quaternion (XYZ)
    cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy

    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    p.pose.orientation.x = qx
    p.pose.orientation.y = qy
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p

def wait_for_state_update(scene, name, is_known=None, is_attached=None, timeout=2.0):
    start = time.time()
    while time.time() - start < timeout:
        known = name in scene.get_known_object_names()
        attached = name in scene.get_attached_objects()
        if (is_known is None or known == is_known) and (is_attached is None or attached == is_attached):
            return True
        rclpy.spin_once(rclpy.get_running_loop(), timeout_sec=0.05)
    return False

def main():
    roscpp_initialize([])
    rclpy.init()
    node = rclpy.create_node("huskypanda_pick_place_demo")

    # MoveIt interfaces
    scene = PlanningSceneInterface(ns="")
    arm   = MoveGroupCommander(ARM_GROUP)
    hand  = MoveGroupCommander(HAND_GROUP)

    arm.set_end_effector_link(EE_LINK)
    arm.set_planning_time(5.0)
    arm.set_num_planning_attempts(3)
    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_max_acceleration_scaling_factor(0.3)
    arm.allow_replanning(True)

    # --- 1) Add a table and a box in front of the robot (static demo) ---
    table_name = "demo_table"
    obj_name   = "demo_box"
    scene.remove_world_object(table_name)
    scene.remove_world_object(obj_name)

    # Table: big thin box below the object
    table_pose = make_pose(x=0.6, y=0.0, z=0.30)     # center top @ z=0.30
    scene.add_box(table_name, table_pose, size=(0.8, 0.8, 0.02))

    # Object: small cube on the table (top @ 0.31 -> cube 0.04 => center z ~ 0.31+(0.04/2)=0.33)
    box_pose = make_pose(x=0.60, y=0.00, z=0.33)
    scene.add_box(obj_name, box_pose, size=(0.04, 0.04, 0.04))

    wait_for_state_update(scene, table_name, is_known=True, timeout=2.0)
    wait_for_state_update(scene, obj_name,   is_known=True, timeout=2.0)

    # --- 2) Open gripper ---
    def open_gripper():
        # Panda fingers typically open to ~0.04 m (each ~0.02 rad equivalent position).
        # If your SRDF has a "panda_hand" group, sending a joint value target works:
        hand.set_named_target("open") if "open" in hand.get_named_targets() else hand.set_joint_value_target([0.04, 0.04])
        hand.go(wait=True)
        hand.stop()

    def close_gripper():
        hand.set_named_target("close") if "close" in hand.get_named_targets() else hand.set_joint_value_target([0.0, 0.0])
        hand.go(wait=True)
        hand.stop()

    open_gripper()

    # --- 3) Move to a pre-grasp pose above the box ---
    pregrasp = make_pose(x=0.60, y=0.00, z=0.45)   # 12 cm above the box center
    arm.set_pose_target(pregrasp)
    arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()

    # --- 4) Cartesian approach down ---
    waypoints = [pregrasp.pose]
    approach  = make_pose(x=0.60, y=0.00, z=0.36).pose  # 3 cm above box top
    waypoints.append(approach)

    (plan, fraction) = arm.compute_cartesian_path(waypoints, eef_step=0.01, jump_threshold=0.0)
    arm.execute(plan, wait=True)

    # --- 5) Close gripper (grasp) ---
    close_gripper()
    time.sleep(0.3)

    # --- 6) Attach object to the gripper in the planning scene ---
    scene.attach_box(EE_LINK, obj_name)
    wait_for_state_update(scene, obj_name, is_attached=True, timeout=2.0)

    # --- 7) Retreat up ---
    waypoints = [make_pose(x=0.60, y=0.00, z=0.45).pose]
    (plan, fraction) = arm.compute_cartesian_path(waypoints, eef_step=0.01, jump_threshold=0.0, avoid_collisions=True)
    arm.execute(plan, wait=True)

    # --- 8) Move to a place pose (to the right) ---
    place_pre = make_pose(x=0.55, y=-0.20, z=0.45)
    arm.set_pose_target(place_pre)
    arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()

    # Lower to place
    place = make_pose(x=0.55, y=-0.20, z=0.34)  # just above table
    (plan, fraction) = arm.compute_cartesian_path([place.pose], 0.01, 0.0, avoid_collisions=True)
    arm.execute(plan, wait=True)

    # --- 9) Open gripper & detach ---
    open_gripper()
    scene.remove_attached_object(EE_LINK, name=obj_name)
    wait_for_state_update(scene, obj_name, is_attached=False, timeout=2.0)

    # --- 10) Retreat up and clear object from world (optional keep) ---
    (plan, fraction) = arm.compute_cartesian_path([place_pre.pose], 0.01, 0.0, avoid_collisions=True)
    arm.execute(plan, wait=True)

    # If you want to keep the placed object, comment the next line:
    # scene.remove_world_object(obj_name)

    node.get_logger().info("Pick & place demo DONE.")
    node.destroy_node()
    roscpp_shutdown()

if __name__ == "__main__":
    main()
