#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from asbir_navigation.classes import *
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.duration import Duration


class PointPub(Node):
    def __init__(self):
            super().__init__('PointPub')
            self.pointPub = self.create_publisher(PointStamped, 'targetPoint', 1)
            self.timer = self.create_timer(1, self.broadcast_timer_callback)     

    def broadcast_timer_callback(self):
        point = PointStamped(header=Header(stamp=Time(sec=0, nanosec=0), frame_id="structure"), point=Point(x=0.2, y=-0.0, z=1.317))
        self.pointPub.publish(point)



def main(args=None):
    rclpy.init(args=args)
    node = PointPub()
    rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()