#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import time

QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

class JSToGZ(Node):
    def __init__(self):
        super().__init__('js_to_gz_cmd')

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
        self.create_subscription(JointState, '/joint_states_cmd', self.on_js, QOS)

        # Remember latest commands and republish at 20 Hz (helps debugging)
        self.last_cmd = {j: 0.0 for j in self.pubs.keys()}
        self.last_recv = 0.0
        self.create_timer(0.05, self.republish)

        # Helpful startup logs
        for j, t in self.topic_map.items():
            self.get_logger().info(f'Publishing {j} -> {t}')
        self.get_logger().info('Relay ready: /joint_states_cmd -> /model/panda/joint/*/cmd_pos')

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
    rclpy.spin(JSToGZ())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
