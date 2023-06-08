#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros import TransformException

from geometry_msgs.msg import Point, Quaternion, PointStamped, Transform
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from std_msgs.msg import Header, Bool
from builtin_interfaces.msg import Time

from parse import *
from asbir_navigation.classes import *
from asbir_navigation.aStar import AStar
import numpy as np



class BestPath(Node):
    def __init__(self):
            super().__init__('BuildBestPath')
            self.pathPub = self.create_publisher(Path, 'path', 10)
            self.visPub = self.create_publisher(Marker, "visualization_marker", 10)
            self.activePathPub = self.create_publisher(Bool, 'activePath', 10)

            self.tfBuffer = Buffer()
            self.tfListener = TransformListener(self.tfBuffer, self)
            self.tfBroadcaster = TransformBroadcaster(self)
            self.graph = {}
            self.astar = AStar()

            self.loadGraph()
            self.targetSub = self.create_subscription(PointStamped, 'targetPoint', self.buildPath, 10)

    def loadGraph(self):
        S=Surfaces()
        lines = None
        edges = []
        sourceid=''
        with open("/home/aralab/ASBIR/ASBIR-ROS2/src/asbir_navigation/graphs/mapGraph.txt", "r") as f: 
            lines = f.readlines()
        for line in lines:
            if ':' in line:
                sourceid = search("{}:", line).fixed[0]
                self.graph[sourceid]=[]
                
            else:
                parsed=search('-source=(id={}, frame_pos=geometry_msgs.msg.PointStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=\'{}\'), point=geometry_msgs.msg.Point(x={}, y={}, z={})), surface={}, edge={}, ground={}, pos=geometry_msgs.msg.PointStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=\'{}\'), point=geometry_msgs.msg.Point(x={}, y={}, z={}))) ,target=(id={}, frame_pos=geometry_msgs.msg.PointStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=\'{}\'), point=geometry_msgs.msg.Point(x={}, y={}, z={})), surface={}, edge={}, ground={}, pos=geometry_msgs.msg.PointStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=\'{}\'), point=geometry_msgs.msg.Point(x={}, y={}, z={}))) ,distance={},rotation=geometry_msgs.msg.Quaternion(x={}, y={}, z={}, w={})',line).fixed

                self.graph[sourceid].append(Edge(source=Vertice(
                        id=parsed[0], 
                        frame_pos=PointStamped(header=Header(stamp=Time(sec=0, nanosec=0), frame_id=parsed[1]), point=Point(x=float(parsed[2]),y=float(parsed[3]),z=float(parsed[4]))),
                        surface=S.surface[parsed[5]],
                        edge=bool(parsed[6]),
                        ground=bool(parsed[7]),
                        pos=PointStamped(header=Header(stamp=Time(sec=0, nanosec=0), frame_id=parsed[8]), point=Point(x=float(parsed[9]),y=float(parsed[10]),z=float(parsed[11])))
                        ),
                    target=Vertice(
                        id=parsed[12], 
                        frame_pos=PointStamped(header=Header(stamp=Time(sec=0, nanosec=0), frame_id=parsed[13]), point=Point(x=float(parsed[14]),y=float(parsed[15]),z=float(parsed[16]))),
                        surface=S.surface[parsed[17]],
                        edge=bool(parsed[18]),
                        ground=bool(parsed[19]),
                        pos=PointStamped(header=Header(stamp=Time(sec=0, nanosec=0), frame_id=parsed[20]), point=Point(x=float(parsed[21]),y=float(parsed[22]),z=float(parsed[23])))
                        ),
                    distance=float(parsed[24]),
                    rotation=Quaternion(x=float(parsed[25]),y=float(parsed[26]),z=float(parsed[27]),w=float(parsed[28]))
                ))
        print('graph loaded')

    def findEndTransform(self,end, targetNode, frame_id):
        # if target node is not on the ground use the frame of its surface, otherwise use base frame
        if not targetNode.ground:
            # end_frame_pos = self.tfBuffer.transform(end.pos, targetNode.surface.id)
            tf2_geometry_msgs.tf2_geometry_msgs.do_transform_point(end.pos,targetNode.surface.frame)
            target_frame_pos = targetNode.frame_pos
        else:
            end_frame_pos = end.pos
            target_frame_pos = targetNode.pos

        endA = np.array((end_frame_pos.point.x, end_frame_pos.point.y, end_frame_pos.point.z))
        targetNodeA = np.array((target_frame_pos.point.x, target_frame_pos.point.y, target_frame_pos.point.z))
        # find distance vector between start and node
        ed = endA - targetNodeA
        # find angle between target node and end, relative to the set frame
        e_yaw = np.arctan2(ed[1],ed[0])
        # create z and w values for rotation quaternion
        ez = np.sin(e_yaw/2)
        ew = np.cos(e_yaw/2)
        e = PoseStamped()
        e.pose = Pose(position=end.pos.point, orientation=Quaternion(x=0.0,y=0.0,z=ez,w=ew))
        e.header.frame_id = targetNode.surface.id
        # transform pose into the base frame
        e=tf2_geometry_msgs.tf2_geometry_msgs.do_transform_pose_stamped(e,targetNode.surface.frame)
        return e

    def findStartTransform(self,start, minS):
        # if starting node is not on the ground use the frame of its surface, otherwise use base frame
        s = PoseStamped()
        if not minS.ground:
            # start_frame_pos = self.tfBuffer.transform(start.pos, minS.surface.id)
            start_frame_pos=tf2_geometry_msgs.tf2_geometry_msgs.do_transform_point(start.pos,minS.surface.frame)
            minS_frame_pos = minS.frame_pos
            
        else:
            start_frame_pos = start.pos
            minS_frame_pos = minS.pos 
            # s.header.frame_id = 'structure'

        startA = np.array((start_frame_pos.point.x, start_frame_pos.point.y, start_frame_pos.point.z))
        minSA = np.array((minS_frame_pos.point.x, minS_frame_pos.point.y, minS_frame_pos.point.z))
        # find distance vector between start and node
        sd = minSA-startA 
        # find angle between start and node, relative to the set frame
        s_yaw = np.arctan2(-sd[1],sd[0])
        # create z and w values for rotation quaternion
        sz = np.sin(s_yaw/2)
        sw = np.cos(s_yaw/2)
        
        s.pose = Pose(position=start.pos.point, orientation=Quaternion(x=0.0,y=0.0,z=sz,w=sw))
        s.header.frame_id = minS.surface.id
        
        s=tf2_geometry_msgs.tf2_geometry_msgs.do_transform_pose_stamped(s, minS.surface.frame)
        return s
    
    def getRobotPose(self):
        connecting = True
        while connecting:
            try:
                transform=self.tfBuffer.lookup_transform('structure', 'T265_pose_frame',rclpy.time.Time())
                connecting=False
            except TransformException:
                continue
        return transform

    def buildPath(self, msg):
        print('building path')
        if self.graph == None:
            self.get_logger().info('Abort BuildPath: graph == None')
            return
        graphCopy = self.graph.copy()
        print('graph copied')
        currentPose = self.getRobotPose()
        findPath = AStar()
        print('created astar object')
        print(currentPose)

        # set starting vertice to current position of robot
        start = Vertice()
        start.pos.point = Point(x=currentPose.transform.translation.x, y=currentPose.transform.translation.y, z=currentPose.transform.translation.z-0.05)
        start.id = 'start'
        start.pos.header.frame_id = 'structure'
        

    	# set goal vertice to Target message
        end = Vertice()
        end.pos = msg
        end.pos.point.z = end.pos.point.z
        end.pos.header.frame_id = 'structure'
        end.id = 'end'

        
        startA = np.array((start.pos.point.x, start.pos.point.y, start.pos.point.z))
        endA = np.array((end.pos.point.x, end.pos.point.y, end.pos.point.z))
        mindS = 500
        mindE = 500
        minS = Vertice(pos=PointStamped(header=Header(stamp=Time(sec=0, nanosec=0), frame_id='structure'),point=Point(x=500.0,y=500.0,z=500.0)))
        minE = Vertice(pos=PointStamped(header=Header(stamp=Time(sec=0, nanosec=0), frame_id='structure'),point=Point(x=500.0,y=500.0,z=500.0)))
    
        # find nodes closest to start and end
        print('find nodes closest to start and end')
        for i in graphCopy:
            try:
                a = np.array((graphCopy[i][0].source.pos.point.x,graphCopy[i][0].source.pos.point.y,graphCopy[i][0].source.pos.point.z))
                distS = np.linalg.norm(startA - a)
                distE = np.linalg.norm(endA - a)
                if distS < mindS:
                    mindS = distS
                    minS = graphCopy[i][0].source
                if distE < mindE:
                    mindE = distE
                    minE = graphCopy[i][0].source
            except IndexError:
                continue
        
        # find neighbor of node closest to goal that is closest to the starting position
        targetNode = minE
        for node in graphCopy[minE.id]:
            targetNodeA = np.array((targetNode.pos.point.x, targetNode.pos.point.y, targetNode.pos.point.z))
            targetNodeD = np.linalg.norm(startA - targetNodeA)
            nodeA = np.array((node.target.pos.point.x, node.target.pos.point.y, node.target.pos.point.z))
            nodeD = np.linalg.norm(startA - nodeA)
            if nodeD < targetNodeD and node.target.surface == start.surface:
                targetNode = node.target
        
        sEdge = Edge(source=start,target=minS,distance=mindS,rotation=currentPose.transform.rotation)
        graphCopy[start.id] = [sEdge]

        # find path from start to end, return list of node ids and dictionary of path edges using node ids as keys
        print('running astar')
        pathNodes, pathEdges = self.astar.aStar(graphCopy, start.id, targetNode.id)

        # visualize path
        path = Marker()
        # path.header.frame_id = "robot_odom_frame"
        path.header.frame_id = "structure"
        path.header.stamp = self.get_clock().now().to_msg()
        path.type = path.LINE_LIST
        path.action = path.ADD
        path.id = 1000
        path.scale.x = 0.05
        path.scale.y = 0.05
        path.scale.z = 0.05
        path.pose.orientation.x = 0.0
        path.pose.orientation.y = 0.0
        path.pose.orientation.z = 0.0
        path.pose.orientation.w = 1.0
        path.color.r = 1.0
        path.color.g = 1.0
        path.color.b = 0.0
        path.color.a = 1.0

        pathPoints = Path()

        # find transformation from starting pose to the first waypoint
        st = self.findStartTransform(minS,start)
        pathPoints.poses.append(st)
        path.points.append(start.pos.point)
        path.points.append(minS.pos.point)


        end_frame_id = 0
        # create transforms from waypoint to waypoint
        for i in range(2,len(pathNodes)):
            path.points.append(pathEdges[pathNodes[i]].source.pos.point)
            path.points.append(pathEdges[pathNodes[i]].target.pos.point)
            t = PoseStamped()
            t.pose = Pose(position=Point(x=pathEdges[pathNodes[i]].target.pos.point.x, y=pathEdges[pathNodes[i]].target.pos.point.y, z=pathEdges[pathNodes[i]].target.pos.point.z), 
                                            orientation=pathEdges[pathNodes[i]].rotation)
            # t.header.frame_id = 'robot_odom_frame'		
            t.header.frame_id = 'structure'
            pathPoints.poses.append(t)
            end_frame_id = i+1

        pathTransforms = []
        i=0
        for p in pathPoints.poses:
            a = TransformStamped()
            a.header.frame_id = 'structure'
            a.transform.translation = Vector3(x=p.pose.position.x,y=p.pose.position.y,z=p.pose.position.z)
            a.transform.rotation = p.pose.orientation
            a.child_frame_id = str(i)
            i+=1
            pathTransforms.append(a)

        
        # find transform from final node to the end goal
        et = self.findEndTransform(end, targetNode, end_frame_id)
        pathPoints.poses.append(et)
        path.points.append(targetNode.pos.point)
        path.points.append(end.pos.point)
        
        # publish transform array and visualizations
        # self.tfBroadcaster.sendTransform(pathTransforms)
        print('publish')
        self.pathPub.publish(pathPoints)
        self.visPub.publish(path)
        # self.activePathPub.publish(Bool(data=True))

def main(args=None):
    rclpy.init(args=args)
    node = BestPath()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()