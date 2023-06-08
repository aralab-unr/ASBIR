#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import PointStamped, Point
import os
from builtin_interfaces.msg import Time

class NodeController(Node):
    def __init__(self):
            super().__init__('NodeController')
            self.systemSub = self.create_subscription(Bool, 'ToggleSystem', self.toggleSystem, 1)
            self.pointPub = self.create_publisher(PointStamped, 'targetPoint', 1)

    def toggleSystem(self, msg):
        print("receive msg")
        if msg.data == True:
            os.system('ros2 launch ~/ASBIR/ASBIR-ROS2/src/asbir_navigation/launch/asbirLaunch.launch.py') 
            # Brock feels like this should be ros2 launch ~/ASBIR/ASBIR-ROS2/src/asbir_navigation/launch/asbirLaunch.launch.py
        else:
            point = PointStamped(header=Header(stamp=Time(sec=0, nanosec=0), frame_id="tag16h5:2"), point=Point(x=0.0, y=0.0, z=0.0))
            self.pointPub.publish(point)
             
def main(args=None):
    rclpy.init(args=args)
    node = NodeController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()