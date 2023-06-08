#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusListener(Node):
    def __init__(self):
            super().__init__('StatusListener')
            self.timer = self.create_timer(1, self.broadcast_timer_callback)     
            self.nodePub = self.create_publisher(String, 'activeNodes', 10)
            self.topicPub = self.create_publisher(String, 'activeTopics', 10)

    def broadcast_timer_callback(self):
        nodes = String(data=str(self.get_node_names()))
        topics= String(data=str(self.get_topic_names_and_types()))

        self.nodePub.publish(nodes)
        self.topicPub.publish(topics)

def main(args=None):
    rclpy.init(args=args)
    node = StatusListener()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()