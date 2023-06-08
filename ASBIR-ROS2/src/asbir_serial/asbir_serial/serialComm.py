#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import serial


class SerialComm(Node):
    def __init__(self):
        super().__init__('serialComm')
        self.ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600, write_timeout=1)
        # self.control = "1500 1500 1600 1600"
        self.control = "1500 1500 1600 1600\n"
        
        self.timer = self.create_timer(0.2, self.broadcast_timer_callback)
        self.servoSub = self.create_subscription(Int32MultiArray, 'servoControl', self.servoSubCallback, 1)
        
    def broadcast_timer_callback(self):
        try:
            self.ser.write(self.control.encode())
            print(self.control)
        except serial.serialutil.SerialTimeoutException:
            print("timeout")


    def servoSubCallback(self, msg):
        (fs,bs,fw,bw) = (msg.data[0],msg.data[1],msg.data[2],msg.data[3])
        if fs < 1000:
            fsS = ("0%d" %fs)
        else:
            fsS = ("%d" %fs)

        if bs < 1000:
            bsS = ("0%d" %bs)
        else:
            bsS = ("%d" %bs)

        if fw < 1000:
            fwS = ("0%d" %fw)
        else:
            fwS = ("%d" %fw)
        
        if bw < 1000:
            bwS = ("0%d" %bw)
        else:
            bwS = ("%d" %bw)

        self.control = ("%s %s %s %s\n" %(fsS,bsS,fwS,bwS))
        

def main(args=None):
    rclpy.init(args=args)
    node = SerialComm()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
