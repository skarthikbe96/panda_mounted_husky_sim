#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from typing import Dict, List
import time

QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

PANDA_JOINTS = [
    "panda_joint1","panda_joint2","panda_joint3","panda_joint4",
    "panda_joint5","panda_joint6","panda_joint7",
    "panda_finger_joint1","panda_finger_joint2",
]
HUSKY_JOINTS = [
    "front_left_wheel_joint","front_right_wheel_joint",
    "rear_left_wheel_joint","rear_right_wheel_joint",
    "pan_gimbal_joint","tilt_gimbal_joint",
]
# Publish a stable joint ordering (helps RViz / debugging)
JOINT_ORDER: List[str] = HUSKY_JOINTS + PANDA_JOINTS

class HuskyPandaBridge(Node):
    def __init__(self):
        super().__init__('husky_panda_bridge')

        # --------------- TF: odom -> base_link from Gazebo odom ---------------
        self.declare_parameters('', [
            ('odom_topic', '/odom'),
            ('odom_frame', 'odom'),
            ('base_frame', 'base_link'),
        ])
        self.tf_broadcaster = TransformBroadcaster(self)
        odom_topic = self.get_parameter('odom_topic').value
        self.create_subscription(
            Odometry,
            odom_topic,
            self.on_odom,
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
        )
        self.get_logger().info(" processor + odom->tf is up.")

        # --------------- Panda command pubs to Gazebo ---------------
        prefix = '/model/panda/joint'
        self.topic_map = {
            'panda_joint1'        : f'{prefix}/panda_joint1/cmd_pos',
            'panda_joint2'        : f'{prefix}/panda_joint2/cmd_pos',
            'panda_joint3'        : f'{prefix}/panda_joint3/cmd_pos',
            'panda_joint4'        : f'{prefix}/panda_joint4/cmd_pos',
            'panda_joint5'        : f'{prefix}/panda_joint5/cmd_pos',
            'panda_joint6'        : f'{prefix}/panda_joint6/cmd_pos',
            'panda_joint7'        : f'{prefix}/panda_joint7/cmd_pos',
            'panda_finger_joint1' : f'{prefix}/panda_finger_joint1/cmd_pos',
            'panda_finger_joint2' : f'{prefix}/panda_finger_joint2/cmd_pos',
        }

        self.pubs = {j: self.create_publisher(Float64, t, QOS) for j, t in self.topic_map.items()}
        self.create_subscription(JointState, '/joint_states', self.on_js, QOS)

        # Remember latest commands and republish at 20 Hz (helps debugging)
        self.last_cmd = {j: 0.0 for j in self.pubs.keys()}
        self.last_recv = 0.0
        self.create_timer(0.05, self.republish)

        # Helpful startup logs
        for j, t in self.topic_map.items():
            self.get_logger().info(f'Publishing {j} -> {t}')
        self.get_logger().info('Relay ready: /joint_states_cmd -> /model/panda/joint/*/cmd_pos')


    # ----- callbacks -----
    def on_odom(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.get_parameter('odom_frame').value
        t.child_frame_id  = self.get_parameter('base_frame').value
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t) 

    def on_gui_js(self, msg: JointState):
        if msg.name and msg.position:
            for n, p in zip(msg.name, msg.position):
                self.gui_js[n] = float(p)
        # Forward GUI-set Panda joints to Gazebo controllers
        for j in PANDA_JOINTS:
            if j in self.gui_js:
                val = Float64(data=self.gui_js[j])
                pub0, pub1 = self.panda_pubs[j]
                pub0.publish(val)
                pub1.publish(val)

    def on_gz_js(self, msg: JointState):
        if msg.name and msg.position:
            for n, p in zip(msg.name, msg.position):
                self.gz_js[n] = float(p)

    def on_js(self, msg: JointState):
        if not msg.name or not msg.position:
            return
        pos = dict(zip(msg.name, msg.position))
        for j, pub in self.pubs.items():
            if j in pos:
                val = float(pos[j])
                self.last_cmd[j] = val
                pub.publish(Float64(data=val))
        self.last_recv = time.time()

    def republish(self):
        # Keep controllers “alive” and make it easy to see traffic with ros2 topic echo
        for j, pub in self.pubs.items():
            pub.publish(Float64(data=self.last_cmd[j]))

def main():
    rclpy.init()
    node = HuskyPandaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
