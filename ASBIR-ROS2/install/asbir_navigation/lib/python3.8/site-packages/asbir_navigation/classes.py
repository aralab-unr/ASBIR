#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Point, Quaternion, Pose, PointStamped, PoseStamped, Vector3, TransformStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import tf2_geometry_msgs.tf2_geometry_msgs 
import numpy as np

class Surface:
    def __init__(self, id='structure', xMin=0, xMax=0, yMin=0, yMax=0, zMin=0, zMax=0, xDim=0, yDim=0, rotation=None):
        self.id = id
        self.xMin = xMin
        self.xMax = xMax
        self.yMin = yMin
        self.yMax = yMax
        self.zMin = zMin
        self.zMax = zMax
        self.xDim = xDim
        self.yDim = yDim
        if not rotation == None:
            self.frame=self.getFrame(rotation)
        else:
            self.frame=TransformStamped()
            self.frame.child_frame_id = 'structure'
	
    def getFrame(self, frame_rotation):
        frame = TransformStamped()
        frame.header.frame_id = 'structure'
        frame.child_frame_id = self.id
        frame.transform.translation = Vector3(x=self.xMin, y=self.yMin, z=self.zMin)
        frame.transform.rotation = frame_rotation
        return frame

    def __str__(self):
        return self.id

class Surfaces:
    xDim = 0.381
    yDim = 0.722
    zDim = 1.317
    xOffset = 0.5
    yOffset = -0.361
    zOffset = -0.19
    
    surfaceA = Surface('surfaceA', 0.0, xDim, yOffset, (yOffset + yDim), zDim, zDim, xDim, yDim, Quaternion(x=0.0,y=0.0,z=0.0,w=1.0))
    surfaceB = Surface('surfaceB', 0.0, 0.0, yOffset, (yOffset + yDim), 0.0, zDim, zDim, yDim, Quaternion(x=0.0, y=-0.707, z=0.0, w=0.707))
    surfaceC = Surface('surfaceC', xDim, xDim, (yOffset + yDim), yOffset, 0.0, zDim, zDim, yDim, Quaternion(x=0.707, y=0.0, z=0.707, w=0.0))
    surfaceD = Surface('surfaceD', xDim, 0.0, yOffset, yOffset, 0.0, zDim, zDim, xDim, Quaternion(x=0.5, y=-0.5, z=0.5, w=0.5))
    surfaceF = Surface('surfaceF', 0.0, 5.0, 0.0, 5.0, 0.0, 5.0, 10.0, 10.0, Quaternion(x=0.0,y=0.0,z=0.0,w=1.0))
    
    surface = {}
    surface[surfaceA.id] = surfaceA
    surface[surfaceB.id] = surfaceB
    surface[surfaceC.id] = surfaceC
    surface[surfaceD.id] = surfaceD
    surface[surfaceF.id] = surfaceF

    surfaces = {}
    surfaces[surfaceA.id] = [surfaceB, surfaceC, surfaceD]
    surfaces[surfaceB.id] = [surfaceA, surfaceD, surfaceF]
    surfaces[surfaceC.id] = [surfaceA, surfaceD, surfaceF]
    surfaces[surfaceD.id] = [surfaceA, surfaceB, surfaceC, surfaceF]
    surfaces[surfaceF.id] = [surfaceB, surfaceC, surfaceD]

class Vertice:
	def __init__(self,id='',frame_pos=PointStamped(),surface=Surface(),edge=False,ground=False,pos=PointStamped()):
		self.id = id
		self.frame_pos = frame_pos
		self.surface = surface
		self.edge = edge
		self.ground = ground
		self.pos = pos

	def __str__(self):
		return "(id={0}, frame_pos={1}, surface={2}, edge={3}, ground={4}, pos={5})".format(self.id, self.frame_pos, self.surface, self.edge, self.ground, self.pos)

class Edge:
    def __init__(self,source,target,distance,rotation):
        self.source = source
        self.target = target
        self.distance = distance
        self.rotation = rotation
    def __str__(self):
        return "(source:{0} {1} {2}\n\ttarget:{3} {4} {5})".format(self.source.pos.point.x,self.source.pos.point.y,self.source.pos.point.z,self.target.pos.point.x,self.target.pos.point.y,self.target.pos.point.z)


class FindEdge:
											      # Euler Angles (roll,pitch,yaw) (degrees)
    QUATERNIONS = [ Quaternion(x=0.0,y=0.0,z=0.0,w=1.0), 						    #(0,0,0)			
                    Quaternion(x=0.0,y=0.0,z=0.382,w=0.924),					#(0,0,45)
                    Quaternion(x=0.0,y=0.0,z=0.707,w=0.707),					#(0,0,90)
                    Quaternion(x=0.0,y=0.0,z=0.924,w=0.382),					#(0,0,135)	
                    Quaternion(x=0.0,y=0.0,z=1.0,w=0.0),							#(0,0,180)
                    Quaternion(x=0.0,y=0.0,z=-0.924,w=0.382),				    #(0,0,-135)
                    Quaternion(x=0.0,y=0.0,z=-0.707,w=0.707),				    #(0,0,-90)
                    Quaternion(x=0.0,y=0.0,z=-0.382,w=0.924),				    #(0,0,-45)

                    Quaternion(x=0.0,y=0.382,z=0.0,w=0.924),					#(0,45,0)
                    Quaternion(x=0.271,y=0.271,z=0.653,w=0.653),			#(0,45,90)
                    Quaternion(x=0.382,y=0.0,z=0.924,w=0.0),					#(0,45,180)
                    Quaternion(x=-0.271,y=0.271,z=-0.635,w=0.635)]		    #(0,45,-90)		

    def getEdge(self,source,target,dist,tfBuffer):
        source_frame = source.surface.id
        target_frame = target.surface.id
        source_pos = source.frame_pos
        target_pos = target.frame_pos
        if target_frame != source_frame:
            target_pos = tf2_geometry_msgs.tf2_geometry_msgs.do_transform_point(target.frame_pos, target.surface.frame)
            # target_pos = tfBuffer.transform(target_pos, source_frame)
            if np.linalg.norm(source_pos.point.x - target_pos.point.x) < 0.1:
                target_pos.point.x = source_pos.point.x
            if np.linalg.norm(source_pos.point.y - target_pos.point.y) < 0.1:
                target_pos.point.y = source_pos.point.y

        q = PoseStamped()
        q.header.frame_id = source_frame

        if source.ground:
            q.pose = Pose(position=target_pos.point,orientation=self.QUATERNIONS[8])
        elif target.edge:
            if np.linalg.norm(target_pos.point.x - source.surface.xDim) < 0.1:
                q.pose = Pose(position=target_pos.point, orientation=self.QUATERNIONS[8])
            elif np.linalg.norm(target_pos.point.y - source.surface.yDim) < 0.1:
                q.pose = Pose(position=target_pos.point, orientation=self.QUATERNIONS[9])
            elif np.linalg.norm(target_pos.point.x - 0) < 0.1:
                q.pose = Pose(position=target_pos.point, orientation=self.QUATERNIONS[10])
            elif np.linalg.norm(target_pos.point.y - 0) < 0.1:
                q.pose = Pose(position=target_pos.point, orientation=self.QUATERNIONS[11])
        else:
            if source_pos.point.x < target_pos.point.x and source_pos.point.y == target_pos.point.y:
                q.pose = Pose(position=target_pos.point, orientation=self.QUATERNIONS[0])
            elif source_pos.point.x > target_pos.point.x and source_pos.point.y < target_pos.point.y:
                q.pose = Pose(position=target_pos.point,orientation=self.QUATERNIONS[1])
            elif source_pos.point.x == target_pos.point.x and source_pos.point.y < target_pos.point.y:
                q.pose = Pose(position=target_pos.point, orientation=self.QUATERNIONS[2])
            elif source_pos.point.x > target_pos.point.x and source_pos.point.y < target_pos.point.y:
                q.pose = Pose(position=target_pos.point, orientation=self.QUATERNIONS[3])
            elif source_pos.point.x > target_pos.point.x and source_pos.point.y == target_pos.point.y:
                q.pose = Pose(position=target_pos.point, orientation=self.QUATERNIONS[4])
            elif source_pos.point.x > target_pos.point.x and source_pos.point.y > target_pos.point.y:
                q.pose = Pose(position=target_pos.point, orientation=self.QUATERNIONS[5])
            elif source_pos.point.x == target_pos.point.x and source_pos.point.y > target_pos.point.y:
                q.pose = Pose(position=target_pos.point, orientation=self.QUATERNIONS[6])
            elif source_pos.point.x < target_pos.point.x and source_pos.point.y > target_pos.point.y:
                q.pose = Pose(position=target_pos.point, orientation=self.QUATERNIONS[7])

        # q = tfBuffer.transform(q, 'odom_frame')
        q=tf2_geometry_msgs.tf2_geometry_msgs.do_transform_pose_stamped(q,source.surface.frame)
        return Edge(source,target,dist,q.pose.orientation)