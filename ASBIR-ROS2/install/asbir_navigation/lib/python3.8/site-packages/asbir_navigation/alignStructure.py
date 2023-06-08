#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf2_ros

from geometry_msgs.msg import TransformStamped, Vector3
from rclpy.duration import Duration
from .transformations import *



class AlignStructure(Node):
    def __init__(self):
            super().__init__('AlignStructure')
            self.tfBuffer = Buffer()    
            self.tfListener = TransformListener(self.tfBuffer, self)
            self.tfBroadcaster = StaticTransformBroadcaster(self)

            self.aligned = False
            self.structureFrame = TransformStamped()

            self.wait_for_apriltag = self.create_timer(0.5, self.wait) 
            self.broadcast_timer = self.create_timer(0.1, self.broadcast)

    def wait(self):
        if self.aligned:
            return
        try:
            self.tfBuffer.lookup_transform("odom_frame", "tag_frame", time=rclpy.time.Time())
            self.aligned = True
            self.structureFrame.header.frame_id = "tag_frame"
            self.structureFrame.child_frame_id = "structure"
            q = quaternion_from_euler(0,0.174533,np.pi)
            # self.structureFrame.transform.rotation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
            self.structureFrame.transform.rotation = Quaternion(x=-0.0958458,y=0.0,z=0.9953962,w=0.0)
            self.structureFrame.transform.translation.z = -0.05
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as exc:
            print(exc)

    def broadcast(self):
        if not self.aligned:
            return
        self.tfBroadcaster.sendTransform(self.structureFrame)
         


def main(args=None):
    rclpy.init(args=args)
    node = AlignStructure()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()