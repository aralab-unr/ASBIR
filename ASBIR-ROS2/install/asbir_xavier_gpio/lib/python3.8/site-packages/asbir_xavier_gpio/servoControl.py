import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import busio
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class XavierServoControl(Node):
    def __init__(self):
            super().__init__('xavierServo')
            self.ctrlSub = self.create_subscription(Float32MultiArray, 'servoControl', self.subCallback, 1)
            self.controller = Float32MultiArray(data=[90, 90, 0, 0])

            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(self.i2c)
            self.pca.frequency = 50

            self.fs = servo.Servo(self.pca.channels[12])
            self.bs = servo.Servo(self.pca.channels[13])
            self.fw = servo.ContinuousServo(self.pca.channels[14])
            self.bw = servo.ContinuousServo(self.pca.channels[15])

            self.fs.set_pulse_width_range(1000,2000)
            self.bs.set_pulse_width_range(1000,2000)
            self.fw.set_pulse_width_range(1460,1760)
            self.bw.set_pulse_width_range(1460,1760)

            self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

    def subCallback(self, msg):
        self.controller.data = msg.data

    def broadcast_timer_callback(self):
         self.fs.angle = self.controller.data[0]
         self.bs.angle = self.controller.data[1]
         self.fw.throttle = self.controller.data[2]
         self.bw.throttle = self.controller.data[3]

def main(args=None):
    rclpy.init(args=args)
    node = XavierServoControl()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()