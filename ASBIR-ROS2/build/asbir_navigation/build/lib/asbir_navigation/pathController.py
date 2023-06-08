#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Pose, Quaternion
from std_msgs.msg import Bool
from nav_msgs.msg import Path

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from asbir_navigation.classes import *



class PathController(Node):
    def __init__(self):
        super().__init__('PathController')
        
        self.tfBroadcaster = StaticTransformBroadcaster(self)
        self.currentNode = 0
        self.path = Path()
        self.activePath = Bool()
        self.activePath.data=False
        self.activePub=self.create_publisher(Bool, 'activePath', 1)
        self.waypointPub=self.create_publisher(Pose,'currentWaypointPose', 1)

        self.pathSub=self.create_subscription(Path, 'path', self.loadPath, 1)
        self.reachedSub=self.create_subscription(Bool, 'waypointReached', self.nextWaypoint, 1)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)   

    def loadPath(self, msg):
        self.path = msg
        self.numNodes = len(msg.poses)
        self.currentNode = 0
        self.activePath.data = True

    def nextWaypoint(self, msg):
        if msg.data:
            if self.currentNode < self.numNodes + 1:
                self.currentNode += 1
            else:
                self.path = None
                self.activePath.data = False

    def broadcast_timer_callback(self):
        if self.activePath.data:
            waypoint = self.path.poses[self.currentNode]
            waypointFrame = TransformStamped()
            waypointFrame.header.frame_id='structure'
            # waypointFrame.header.stamp=rclpy.time.Time()
            
            waypointFrame.child_frame_id='waypoint'
            (waypointFrame.transform._translation._x,waypointFrame.transform._translation._y,waypointFrame.transform._translation._z)=(waypoint.pose.position.x,waypoint.pose.position.y,waypoint.pose.position.z)
            waypointFrame.transform.rotation=waypoint.pose.orientation

            self.activePub.publish(self.activePath)

            # time=self.get_clock().now().seconds_nanoseconds()
            # waypointFrame.header.stamp.sec=time[0]
            # waypointFrame.header.stamp.nanosec=time[1]

            self.tfBroadcaster.sendTransform(waypointFrame)
            self.waypointPub.publish(waypoint.pose)
            

def main(args=None):
    rclpy.init(args=args)
    node = PathController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
   