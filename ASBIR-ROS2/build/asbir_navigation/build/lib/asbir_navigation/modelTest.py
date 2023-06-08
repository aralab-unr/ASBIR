#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped, Vector3, Transform
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from asbir_navigation.classes import *
import numpy as np
import tf2_ros
from .transformations import *
import tf2_geometry_msgs.tf2_geometry_msgs 

class ModelTest(Node):
    def __init__(self):
        super().__init__('Model')
        self.markerPub = self.create_publisher(
            Marker, 
            "visualization_marker", 
            10)
        self.tfBroadcaster = TransformBroadcaster(self)
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        self.waiting = True
        self.S = Surfaces()
        self.structureModel=Marker()

        self.create_model_timer = self.create_timer(0.4, self.create_model)
        self.timer = self.create_timer(0.4, self.broadcast_timer_callback)   
  
    def create_model(self):
        if not self.waiting:
            return   
        try:
            self.tfBuffer.lookup_transform("structure", "tag_frame", time=rclpy.time.Time())
            self.waiting = False
            print("align")
        except tf2_ros.LookupException:
            return
        
        
        #====Build Structure Model===
        # Create base surface model
        
        self.structureModel.header.frame_id = "structure"
        self.structureModel.header.stamp = rclpy.time.Time().to_msg()
        self.structureModel.ns = "Namespace"
        self.structureModel.id = 0
        self.structureModel.type = self.structureModel.CUBE
        self.structureModel.action = self.structureModel.ADD
        self.structureModel.scale.x = self.S.xDim
        self.structureModel.scale.y = self.S.yDim
        self.structureModel.scale.z = self.S.zDim
        self.structureModel.pose.position.x = self.S.xDim/2 #+ S.xOffset)
        self.structureModel.pose.position.y = (self.S.yDim/2 + self.S.yOffset)
        self.structureModel.pose.position.z = self.S.zDim/2 #+ S.zOffset
        self.structureModel.pose.orientation.x = 0.0
        self.structureModel.pose.orientation.y = 0.0
        self.structureModel.pose.orientation.z = 0.0
        self.structureModel.pose.orientation.w = 1.0
        self.structureModel.color.r = 0.5
        self.structureModel.color.g = 0.5
        self.structureModel.color.b = 0.5
        self.structureModel.color.a = 1.0

    def broadcast_timer_callback(self):
        if self.waiting:
            return
        self.markerPub.publish(self.structureModel)
        self.tfBroadcaster.sendTransform([
                            self.S.surfaceA.frame, 
                            self.S.surfaceB.frame,
                            self.S.surfaceC.frame,
                            self.S.surfaceD.frame])

def main(args=None):
    rclpy.init(args=args)
    node = ModelTest()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
   