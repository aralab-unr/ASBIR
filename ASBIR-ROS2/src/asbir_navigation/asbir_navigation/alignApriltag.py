#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf2_ros

from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration



class AlignAprilTag(Node):
    def __init__(self):
            super().__init__('AlignAprilTag')
            self.tfBuffer = Buffer()    
            self.tfListener = TransformListener(self.tfBuffer, self)
            self.tfBroadcaster = StaticTransformBroadcaster(self)

            self.aligned = False
            self.structureFrame = TransformStamped()

            self.align_tag_timer = self.create_timer(0.1, self.align_tag)   
            self.broadcast_timer = self.create_timer(0.1, self.broadcast)

    def align_tag(self):
        if self.aligned:
            return
        try:
            self.structureFrame = self.tfBuffer.lookup_transform("odom_frame", "tag16h5:2", time=rclpy.time.Time())
            self.aligned = True
            self.structureFrame.header.frame_id = "odom_frame"
            self.structureFrame.child_frame_id = "tag_frame"
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
             i =0

    def broadcast(self):
        if not self.aligned:
            return
        self.tfBroadcaster.sendTransform(self.structureFrame)
         


def main(args=None):
    rclpy.init(args=args)
    node = AlignAprilTag()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()