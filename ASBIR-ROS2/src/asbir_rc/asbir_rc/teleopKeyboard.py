#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from pynput import keyboard

class Teleop(Node):
    def __init__(self):
            super().__init__('teleopKeyboard')
            self.ctrlPub = self.create_publisher(Int32MultiArray, 'servoControl', 1)
            self.controller = Int32MultiArray()
            self.controller.data = [1500, 1500, 1600, 1600]

            self.listener = keyboard.Listener(on_press=self.on_press,on_release=self.on_release)
            self.listener.start()

            self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
    
    def broadcast_timer_callback(self):
        self.ctrlPub.publish(self.controller)

    def on_press(self,key):
        try:
            if key.char == "w":
                self.controller.data[2]=2000
                self.controller.data[3]=1200
            elif key.char == "s":
                self.controller.data[2]=1200
                self.controller.data[3]=2000
            elif key.char == "a":
                self.controller.data[0]=900
                self.controller.data[1]=2100
            elif key.char == "d":
                self.controller.data[0]=2100
                self.controller.data[1]=900
        except AttributeError:
            # print(key)
            print(self.controller.data)

        self.ctrlPub.publish(self.controller)

    def on_release(self,key):
        try:
            if key.char == "w":
                self.controller.data[2]=1600
                self.controller.data[3]=1600
            elif key.char == "s":
                self.controller.data[2]=1600
                self.controller.data[3]=1600
            elif key.char == "a":
                self.controller.data[0]=1500
                self.controller.data[1]=1500
            elif key.char == "d":
                self.controller.data[0]=1500
                self.controller.data[1]=1500
        except AttributeError:
            # print(key)
            print(self.controller.data)
    
        self.ctrlPub.publish(self.controller)

def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
