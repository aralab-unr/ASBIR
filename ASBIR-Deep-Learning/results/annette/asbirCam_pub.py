# Basic ROS 2 program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from tensorflow.keras.models import load_model
import math
import six
import numpy as np

IMAGE_ORDERING = "channels_last"
 
class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_publisher')
    self.rgb_received = False
    self.current_frame_header = None
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'output', 10)
    self.subscription = self.create_subscription(
      Image, 
      'input', 
      self.listener_callback, 
      10)
    self.current_frame = none
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    # load model path
    self.model = load_model("/home/aralab/catkin_ws/src/segmentation/eNet/checkpoints/checkpoint-202-0.9826-.hdf5")
    
    # wait for message b4 moving on
    self.seg_pub()
    
  def listener_callback(self, data): 
    # Convert ROS Image message to OpenCV image
    self.current_frame = self.br.imgmsg_to_cv2(data)
    self.rgb_received = True
    self.current_frame_header = data.header
   
  def get_image_array(self, image_input,
                        width, height,
                        imgNorm="sub_mean", ordering='channels_first'):
        """ Load image array from input """
        #print("get", type(image_input))
        if type(image_input) is np.ndarray:
            # It is already an array, use it as it is
            img = image_input
        elif isinstance(image_input, six.string_types):
            if not os.path.isfile(image_input):
                raise DataLoaderError("get_image_array: path {0} doesn't exist"
                                      .format(image_input))
            img = cv2.imread(image_input, 1)
        else:
            raise DataLoaderError("get_image_array: Can't process input type {0}"
                                  .format(str(type(image_input))))

        if imgNorm == "sub_and_divide":
            img = np.float32(cv2.resize(img, (width, height))) / 127.5 - 1
        elif imgNorm == "sub_mean":
            img = cv2.resize(img, (width, height))
            img = img.astype(np.float32)
            img[:, :, 0] -= 103.939
            img[:, :, 1] -= 116.779
            img[:, :, 2] -= 123.68
            img = img[:, :, ::-1]
        elif imgNorm == "divide":
            img = cv2.resize(img, (width, height))
            img = img.astype(np.float32)
            img = img / 255.0

        if ordering == 'channels_first':
            img = np.rollaxis(img, 2, 0)
        return img

    def segmentation(self, cv2_input_img):
        x = self.get_image_array(cv2_input_img, 512, 512, ordering=IMAGE_ORDERING)
        pr = self.model.predict(np.array([x]))[0]
        #(512,512)
        pr = pr.reshape((512, 512, 2)).argmax(axis=2)
        seg_img = cv2.resize(cv2_input_img, (pr.shape[0], pr.shape[1]))
        idx = (pr == 1)
        seg_img[idx] = (0, 0, 255.0)
        seg_img = cv2.resize(seg_img, (self.output_width, self.output_height))
        return seg_img
        
    def seg_pub(self):
        while not rospy.is_shutdown():
            if self.rgb_received:
                try:
                    cv2_output = self.segmentation(self.current_frame)
                    output = self.br.cv2_to_imgmsg(cv2_output)
                    output.header = self.current_frame_header
                    self.publisher_.publish(output)
                except CvBridgeError as e:
                    print(e)
            self.rate.sleep()


  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = ImagePublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
