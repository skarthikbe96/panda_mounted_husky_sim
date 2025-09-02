#!/usr/bin/env python3
import math
import time
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

PANDA_JOINTS = [
    "panda_joint1","panda_joint2","panda_joint3","panda_joint4",
    "panda_joint5","panda_joint6","panda_joint7",
    "panda_finger_joint1","panda_finger_joint2",
]

class PandaTrajectoryBridge(Node):
    def __init__(self):
        super().__init__("panda_trajectory_bridge")
        # self.declare_parameter("use_sim_time", True)
        self.declare_parameter("servo_topic", "/panda_arm_controller/joint_trajectory")
        self.declare_parameter("cmd_prefix", "/model/panda/joint")  # matches your bridge yaml

        self.servo_topic = self.get_parameter("servo_topic").get_parameter_value().string_value
        cmd_prefix = self.get_parameter("cmd_prefix").get_parameter_value().string_value

        # Per-joint publishers -> std_msgs/Float64 (bridged to GZ Double)
        self.pub: Dict[str, rclpy.publisher.Publisher] = {}
        for j in PANDA_JOINTS:
            topic = f"{cmd_prefix}/{j}/cmd_pos"
            self.pub[j] = self.create_publisher(Float64, topic, 10)

        # (Optional) publish a mirrored joint_states of what we command
        self.js_pub = self.create_publisher(JointState, "/panda/commanded_joint_states", 10)

        # Servo input (topic-only)
        self.servo_sub = self.create_subscription(JointTrajectory, self.servo_topic, self.servo_cb, 10)

        # FollowJointTrajectory action (for Plan & Execute)
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/panda_controller/follow_joint_trajectory",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info("PandaTrajectoryBridge ready")

    # --- Helpers --------------------------------------------------------------

    def command_point(self, names: List[str], positions: List[float]):
        # publish per-joint Float64
        for j, p in zip(names, positions):
            if j in self.pub:
                self.pub[j].publish(Float64(data=float(p)))
        # mirror to /panda/commanded_joint_states
        msg = JointState()
        msg.name = list(names)
        msg.position = [float(p) for p in positions]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.js_pub.publish(msg)

    def playback(self, traj: JointTrajectory):
        # Simple time-based playback
        if not traj.points:
            return
        start = self.get_clock().now()
        for i, pt in enumerate(traj.points):
            t = Duration(seconds=float(pt.time_from_start.sec),
                         nanoseconds=float(pt.time_from_start.nanosec))
            # sleep until target time
            while (self.get_clock().now() - start) < t:
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.001)
            self.command_point(traj.joint_names, pt.positions)

    # --- Servo (topic) --------------------------------------------------------
    def servo_cb(self, msg: JointTrajectory):
        # take only the last point (typical Servo behavior), or stream all if present
        if not msg.points:
            return
        last = msg.points[-1]
        if len(last.positions) != len(msg.joint_names):
            self.get_logger().warn("Servo point has mismatched positions length; ignoring")
            return
        self.command_point(msg.joint_names, last.positions)

    # --- Action server (MoveIt Plan & Execute) --------------------------------
    def goal_cb(self, goal_request: FollowJointTrajectory.Goal):
        # basic validation
        if not goal_request.trajectory.joint_names:
            self.get_logger().warn("Goal rejected: empty joint list")
            return GoalResponse.REJECT
        # all joints must be in our published set
        for j in goal_request.trajectory.joint_names:
            if j not in PANDA_JOINTS:
                self.get_logger().warn(f"Goal rejected: unknown joint {j}")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, _goal_handle):
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        traj = goal_handle.request.trajectory
        goal_handle.publish_feedback(FollowJointTrajectory.Feedback(
            joint_names=traj.joint_names
        ))
        try:
            self.playback(traj)
        except Exception as e:
            self.get_logger().error(f"Playback error: {e}")
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = result.INVALID_GOAL
            result.error_string = str(e)
            return result

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = result.SUCCESSFUL
        return result


def main():
    rclpy.init()
    node = PandaTrajectoryBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
