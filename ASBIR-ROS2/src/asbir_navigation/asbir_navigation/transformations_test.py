import unittest
from transformations import *
from geometry_msgs.msg import Quaternion

class SimpleTestCase(unittest.TestCase):
    def setUp(self):
        self.quaternion_from_euler = quaternion_from_euler(0,0,0)
        self.euler_from_quaternion = euler_from_quaternion(Quaternion(x=0.0,y=0.0,z=0.0,w=1.0))

    def test_quaternion_from_euler(self):
        self.assertEqual(self.quaternion_from_euler, [1.0, 0.0, 0.0, 0.0])

    def test_euler_from_quaternion(self):
        self.assertEqual(self.euler_from_quaternion, (0,0,0))

if __name__ == "main":
    unittest.main()