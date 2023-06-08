#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool, String
from geometry_msgs.msg import Pose, TransformStamped, Transform, Vector3
import numpy as np
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from .transformations import *

class PotentialField(Node):
    def __init__(self):
        super().__init__('PotentialField')
        self.servoControlPub = self.create_publisher(Int32MultiArray, 'servoControl', 1)
        self.waypointReachedPub = self.create_publisher(Bool, 'waypointReached', 1)
        self.transformPub = self.create_publisher(Transform, 'transformToWaypoint', 1)

        # self.staticTransformBroadCaster = StaticTransformBroadcaster()
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)

        self.vr_max = 0.25
        self.vr = 0
        self.wheel_rotation_max = np.pi/3
        # self.wheel_rotation_max = 900
        self.reachDist = 0.15
        self.reached = Bool()
        self.activePath=False
        self.last_pose = TransformStamped()
        self.next_pose = TransformStamped()

        self.activePathSub=self.create_subscription(Bool, 'activePath', self.activeCallback, 10)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)     

    def activeCallback(self, msg):
        self.activePath=msg.data

    def broadcast_timer_callback(self):
        if self.activePath:
            self.reached.data = False
            waypoint = TransformStamped()
            current_pose = TransformStamped()
            try:
                waypoint=self.tfBuffer.lookup_transform('waypoint', 'wheel_frame', rclpy.time.Time())
                # print(waypoint.transform.translation.x, waypoint.transform.translation.y, waypoint.transform.translation.z)
                current_pose = self.tfBuffer.lookup_transform('odom_frame', 'wheel_frame', rclpy.time.Time())
            except TransformException as ex:
                print('Could not transform T265_pose_frame to waypoint', ex)
                return
            
            print('%.5f %.5f %.5f' %(current_pose.transform.translation.x - self.next_pose.transform.translation.x,
                  current_pose.transform.translation.y - self.next_pose.transform.translation.y,
                  current_pose.transform.translation.z - self.next_pose.transform.translation.z))
            current_position = np.array((current_pose.transform.translation.x,current_pose.transform.translation.y,current_pose.transform.translation.z))
            last_position = np.array((self.last_pose.transform.translation.x,self.last_pose.transform.translation.y,self.last_pose.transform.translation.z))
            delta_pos=current_position - last_position
            current_time=current_pose.header.stamp.sec + current_pose.header.stamp.nanosec/1000000000
            last_time = self.last_pose.header.stamp.sec + self.last_pose.header.stamp.nanosec/1000000000
            delta_time=current_time - last_time
            current_velocity = delta_pos/delta_time
            # print(current_velocity)
            self.last_pose = current_pose
            
            self.next_pose.transform.translation.x = current_pose.transform.translation.x - current_velocity[0]
            self.next_pose.transform.translation.y = current_pose.transform.translation.y - current_velocity[1]
            self.next_pose.transform.translation.z = current_pose.transform.translation.z - current_velocity[2]

            # Find normal distance between robot and waypoint
            dist = np.linalg.norm([waypoint.transform.translation.x, waypoint.transform.translation.y, waypoint.transform.translation.z])
            # Convert quaternion to euler
            roll, pitch, yaw = euler_from_quaternion(waypoint.transform.rotation)

            # print("Transform Rotation: ", roll, pitch, yaw)
            # print("Transform Translation: ", waypoint.transform.translation.x, waypoint.transform.translation.y, waypoint.transform.translation.z)

            # # set velocity controller of robot
            vr = self.vr_max

            # limit maximum rotation (correcting for +/- angles)
            if yaw > 0:
                wheel_rotation = np.minimum(yaw, self.wheel_rotation_max)
            elif yaw < 0:
                wheel_rotation = np.maximum(yaw, -self.wheel_rotation_max)

            # Iterate to next waypoint if within reach radius of waypoint
            if(dist < self.reachDist):
                wheel_rotation = 0
                vr = 0
                self.reached.data = True


            # # convert control values to servo value range 
            #     # Steering 1500 = 0, 900 - 2100
            #     # Wheel rotation 1600 = 0, 800 - 2400
            fs = int(1500 + 600 * (wheel_rotation / self.wheel_rotation_max))
            bs = int(1500 - 600 * (wheel_rotation / self.wheel_rotation_max))
            fw = int(1600 + 400 * vr/self.vr_max)
            bw = int(1600 - 400 * vr/self.vr_max)
            # # fw = 1600
            # # bw = 1600

            # fs = 90 + 90 * (wheel_rotation / self.wheel_rotation_max)
            # bs = 90 - 90 * (wheel_rotation / self.wheel_rotation_max)
            # fw = vr/self.vr_max
            # bw = vr/self.vr_max

            # print(fs, bs, fw, bw)
            # print(yaw)

            servoControl = Int32MultiArray()
            servoControl.data = [fs,bs,fw,bw]
            self.waypointReachedPub.publish(self.reached)
            self.servoControlPub.publish(servoControl)
            self.transformPub.publish(waypoint.transform)
        
def main(args=None):
    rclpy.init(args=args)
    node = PotentialField()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()