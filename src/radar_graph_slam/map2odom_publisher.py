#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped


class Map2OdomPublisher(Node):
    def __init__(self):
        super().__init__('map2odom_publisher')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscriber = self.create_subscription(
            TransformStamped,
            '/radar_graph_slam/odom2base',
            self.callback,
            10
        )
        self.odom_msg = None
        self.timer = self.create_timer(0.1, self.spin)

    def callback(self, odom_msg):
        self.odom_msg = odom_msg

    def spin(self):
        if self.odom_msg is None:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(t)
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_msg.header.frame_id
        t.child_frame_id = self.odom_msg.child_frame_id
        t.transform = self.odom_msg.transform
        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Map2OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()