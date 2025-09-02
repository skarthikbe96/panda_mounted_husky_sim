#!/usr/bin/env python3
"""
Twist -> TwistStamped bridge for ROS 2 (Jazzy)

- Subscribes: /cmd_vel (geometry_msgs/msg/Twist)
- Publishes : /husky_velocity_controller/cmd_vel (geometry_msgs/msg/TwistStamped)

Notes:
- The header stamp uses node clock (respects /use_sim_time if set).
- Default frame_id is empty string "" to match many ros2_control configs.
- Change topics/frame_id with ROS parameters.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToStamped(Node):
    def __init__(self):
        super().__init__('husky_panda_description')

        # Parameters
        self.declare_parameter('in_topic', '/cmd_vel')
        self.declare_parameter('out_topic', '/husky_velocity_controller/cmd_vel')
        self.declare_parameter('frame_id', '')

        in_topic  = self.get_parameter('in_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('out_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # QoS: depth=10, reliable, volatile (typical for command topics)
        qos_profile = rclpy.qos.QoSProfile(depth=10)

        self.pub = self.create_publisher(TwistStamped, out_topic, qos_profile)
        self.sub = self.create_subscription(Twist, in_topic, self.cb, qos_profile)

        self.get_logger().info(
            f"Bridging {in_topic} (Twist) → {out_topic} (TwistStamped, frame_id='{self.frame_id}')"
        )

    def cb(self, msg: Twist):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()  # honors /use_sim_time
        ts.header.frame_id = self.frame_id
        ts.twist = msg
        self.pub.publish(ts)

def main():
    rclpy.init()
    node = TwistToStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
