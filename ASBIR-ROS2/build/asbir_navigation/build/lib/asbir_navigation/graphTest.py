#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from rclpy.duration import Duration

import tf2_geometry_msgs.tf2_geometry_msgs
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
import tf2_ros

from asbir_navigation.classes import *

class GraphTest(Node):
    def __init__(self):
            super().__init__('Graph')
            self.markerPub = self.create_publisher(Marker, "visualization_marker", 10)
            self.markerAPub = self.create_publisher(MarkerArray, "visualization_marker_array", 10)
            self.tfBuffer = Buffer()
            self.tfListener = TransformListener(self.tfBuffer, self)
            self.tfBroadcaster = TransformBroadcaster(self)

            self.graph = {}
            self.points = []
            self.id = 0

            self.waiting = True
            self.timer1 = self.create_timer(1, self.wait_for_transform)  
            self.timer2 = self.create_timer(1, self.broadcast_timer_callback)   

    def wait_for_transform(self):
        if not self.waiting:
            return
        while self.waiting:
            try:
                self.tfBuffer.lookup_transform("odom_frame", "structure", rclpy.time.Time())
                self.waiting = False
                self.createGraph()
            except tf2_ros.LookupException:
                continue
    
    def createGraph(self):
        F = FindEdge()
        S = Surfaces()
        self.verticeArray=MarkerArray()
        #===Create Vertices===
        row,col = (2, 3)
        
        for i in range(row+1):
            for j in range(col):
                p = Vertice(frame_pos=PointStamped(),surface=S.surfaceA)
                p.frame_pos.header.frame_id = 'surfaceA'
                p.frame_pos.header.stamp = rclpy.time.Time().to_msg()
                p.id = "%s_%d_%d" %('surfaceA', i, j)
                p.frame_pos.point.x = (S.xDim/row) * (i)
                p.frame_pos.point.y = (S.yDim/col) * (j)
                p.frame_pos.point.z = 0.0
                # p.frame_pos.child_frame_id = p.frame_pos.header.frame_id+str(p.frame_pos.point)
                if (p.frame_pos.point.x == S.xDim or p.frame_pos.point.x == 0) and not (p.frame_pos.point.y == S.yDim or p.frame_pos.point.y == 0):
                    p.edge = True
                elif (p.frame_pos.point.y == S.yDim or p.frame_pos.point.y == 0) and not (p.frame_pos.point.x == S.xDim or p.frame_pos.point.x == 0):
                    p.edge = True
                if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
                    self.points.append(p)

        row,col = (6, 3)
        for i in range(row):
            for j in range(1,col):
                p = Vertice(frame_pos=PointStamped(),surface=S.surfaceB)
                p.frame_pos.header.frame_id = 'surfaceB'
                p.frame_pos.header.stamp = rclpy.time.Time().to_msg()
                p.id = "%s_%d_%d" %('surfaceB', i, j)
                p.frame_pos.point.x = (S.zDim/row) * (i)
                p.frame_pos.point.y = (S.yDim/col) * (j)
                p.frame_pos.point.z = 0.0
                # p.frame_pos.child_frame_id = p.frame_pos.header.frame_id+str(p.frame_pos.point)
                if (p.frame_pos.point.x == S.xDim or p.frame_pos.point.x == 0) != (p.frame_pos.point.y == S.yDim or p.frame_pos.point.y == 0):
                    p.edge = True
                if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
                    self.points.append(p)

                if i == 0 and not (j == 0 or j == col):
                    gp = Vertice(PointStamped(),surface=S.surfaceB)
                    gp.frame_pos.header.frame_id = 'surfaceB'
                    gp.frame_pos.header.stamp = rclpy.time.Time().to_msg()
                    gp.id = "%s_%d_%d" %('surfaceBF', i, j)
                    gp.ground = True
                    gp.frame_pos.point.x = 0.0
                    gp.frame_pos.point.y = (S.yDim/col) * (j)
                    gp.frame_pos.point.z = (S.zDim/row)
                    # gp.frame_pos.child_frame_id = p.frame_pos.header.frame_id+str(p.frame_pos.point)
                    self.points.append(gp)

        for i in range(row):
            for j in range(1,col):
                p = Vertice(frame_pos=PointStamped(),surface=S.surfaceC)
                
                p.frame_pos.header.frame_id = 'surfaceC'
                p.frame_pos.header.stamp = rclpy.time.Time().to_msg()
                p.id = "%s_%d_%d" %('surfaceC', i, j)
                p.frame_pos.point.x = (S.zDim/row) * (i)
                p.frame_pos.point.y = (S.yDim/col) * (j)
                p.frame_pos.point.z = 0.0
                # p.frame_pos.child_frame_id = p.frame_pos.header.frame_id+str(p.frame_pos.point)
                if (p.frame_pos.point.x == S.xDim or p.frame_pos.point.x == 0) != (p.frame_pos.point.y == S.yDim or p.frame_pos.point.y == 0):
                    p.edge = True
                if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
                    self.points.append(p)

                if i == 0 and not (j == 0 or j == col):
                    gp = Vertice(frame_pos=PointStamped(),surface=S.surfaceC)
                    gp.frame_pos.header.frame_id = 'surfaceC'
                    gp.frame_pos.header.stamp = rclpy.time.Time().to_msg()
                    gp.id = "%s_%d_%d" %('surfaceCF', i, j)
                    gp.ground = True
                    gp.frame_pos.point.x = 0.0
                    gp.frame_pos.point.y = (S.yDim/col) * (j)
                    gp.frame_pos.point.z = (S.zDim/row)
                    # gp.frame_pos.child_frame_id = p.frame_pos.header.frame_id+str(p.frame_pos.point)
                    self.points.append(gp)

        row,col = (6, 2)
        for i in range(row+1):
            for j in range(col+1):
                p = Vertice(frame_pos=PointStamped(),surface=S.surfaceD)
                p.frame_pos.header.frame_id = 'surfaceD'
                p.frame_pos.header.stamp = rclpy.time.Time().to_msg()
                p.id = "%s_%d_%d" %('surfaceD', i, j)
                p.frame_pos.point.x = (S.zDim/row) * (i)
                p.frame_pos.point.y = (S.xDim/col) * (j)
                p.frame_pos.point.z = 0.0
                # p.frame_pos.child_frame_id = p.frame_pos.header.frame_id+str(p.frame_pos.point)
                if (p.frame_pos.point.x == S.zDim or p.frame_pos.point.x == 0) != (p.frame_pos.point.y == S.xDim or p.frame_pos.point.y == 0):
                    p.edge = True
                if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
                    self.points.append(p)

                if i == 0 and not (j == 0 or j == col):
                    gp = Vertice(frame_pos=PointStamped(),surface=S.surfaceD)
                    gp.frame_pos.header.frame_id = 'surfaceD'
                    gp.frame_pos.header.stamp = rclpy.time.Time().to_msg()
                    gp.id = "%s_%d_%d" %('surfaceDF', i, j)
                    gp.ground = True
                    gp.frame_pos.point.x = 0.0
                    gp.frame_pos.point.y = (S.xDim/col) * (j)
                    gp.frame_pos.point.z = (S.zDim/row)
                    # gp.frame_pos.child_frame_id = p.frame_pos.header.frame_id+str(p.frame_pos.point)
                    self.points.append(gp)

        for j in self.points:
            j.pos = tf2_geometry_msgs.tf2_geometry_msgs.do_transform_point(j.frame_pos, j.surface.frame)

        #create graph	
        for j in self.points:     
            vert=j.id
            edges = []
            for k in self.points:
                a = np.array((j.pos.point.x, j.pos.point.y, j.pos.point.z))
                b = np.array((k.pos.point.x, k.pos.point.y, k.pos.point.z))
                dist = np.linalg.norm(a-b)
                if (not (j.edge and k.edge) and not(j.ground and k.ground) and # ensure edge points cannot connect to edge points and ground points cannont connect to ground points
                ((dist < 0.33 and not j.pos == k.pos and j.surface == k.surface and not (j.edge or k.edge or j.ground or k.ground)) or # establish connections on face of surface, excluding edge points and ground points
                (dist < 0.245 and j.ground != k.ground and j.surface == k.surface) or # establish connections to ground points
                (dist < 0.245 and j.edge != k.edge and j.surface == k.surface) or # establish connections to edge points
                (dist < 0.245 and j.edge != k.edge and j.surface != k.surface and (j.surface in S.surfaces[k.surface.id])))): # establish connections between surfaces
                    edge = F.getEdge(j,k,dist, self.tfBuffer)
                    edges.append(edge)

            self.graph[vert] = edges

        # Transform point from surface frame to base frame and create marker
        for p in self.points:
            vertice = Marker()
            vertice.header.frame_id = p.pos.header.frame_id
            vertice.header.stamp = rclpy.time.Time().to_msg()
            vertice.type = vertice.SPHERE
            vertice.action = vertice.ADD
            vertice.id = self.id
            self.id += 1
            vertice.scale.x = 0.01
            vertice.scale.y = 0.01
            vertice.scale.z = 0.01
            vertice.pose.position.x = p.pos.point.x
            vertice.pose.position.y = p.pos.point.y
            vertice.pose.position.z = p.pos.point.z
            vertice.pose.orientation.x = 0.0
            vertice.pose.orientation.y = 0.0
            vertice.pose.orientation.z = 0.0
            vertice.pose.orientation.w = 1.0
            vertice.color.r = 0.0
            vertice.color.g = 0.0
            vertice.color.b = 0.0
            vertice.color.a = 1.0
            # if p.edge:
            # 	vertice.color.r = 0.5
            # 	vertice.color.g = 0
            # 	vertice.color.b = 1
            # 	vertice.color.a = 1
            self.verticeArray.markers.append(vertice)
            
        # visualize connections between vertices	
        self.line = Marker()
        self.line.header.frame_id = "structure"
        self.line.header.stamp = rclpy.time.Time().to_msg()
        self.line.type = vertice.LINE_LIST
        self.line.action = vertice.ADD
        self.line.id = self.id
        self.line.scale.x = 0.005
        self.line.pose.orientation.x = 0.0
        self.line.pose.orientation.y = 0.0
        self.line.pose.orientation.z = 0.0
        self.line.pose.orientation.w = 1.0
        self.line.color.r = 0.0
        self.line.color.g = 0.0
        self.line.color.b = 0.0
        self.line.color.a = 1.0
            
        for vert in self.graph:
            for nxtvert in self.graph[vert]:
                self.line.points.append(nxtvert.target.pos.point)
                self.line.points.append(nxtvert.source.pos.point)

        # save graph to file
        with open("/home/aralab/ASBIR/ASBIR-ROS2/src/asbir_navigation/graphs/mapGraph.txt", "w") as f:
            for key, value in self.graph.items():
                f.write('%s:\n' % key)
                # print(key,len(value))
                for edge in value:
                    f.write('-source=%s ,target=%s ,distance=%s,rotation=%s\n' 
                    % (edge.source,edge.target,edge.distance,edge.rotation))
        
    def broadcast_timer_callback(self):
        if self.waiting:
            return
        # publish markers
        self.markerAPub.publish(self.verticeArray)
        self.markerPub.publish(self.line)


def main(args=None):
    rclpy.init(args=args)
    graphTest = GraphTest()
    rclpy.spin(graphTest)

    graphTest.destroy_node()
    rclpy.shutdown()
		
if __name__ == '__main__':
    main()