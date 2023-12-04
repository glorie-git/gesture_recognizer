import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
# import rospkg
import os
from ament_index_python.packages import get_package_share_directory

my_package_name='gesture_recognizer'
# Construct the full path to 'gesture_recognizer.task'
model_asset_path = os.path.join(get_package_share_directory(my_package_name), 'gesture_recognizer.task')

class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(String, 'robot_status', 1)
        # self.subscription  # prevent unused variable warning

        self.image_sub = self.create_subscription(Image,"/image_in",self.image_callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)

        # enable drive
        self.drive_enabled_pub = self.create_publisher(Bool,"/drive_enabled", 1)
        # self.drive_enabled
        self.setup_mediapipe()

        

    def setup_mediapipe(self):
        self.base_options = python.BaseOptions(model_asset_path=model_asset_path)
        self.options = vision.GestureRecognizerOptions(
            base_options=self.base_options,
            # running_mode=mp.tasks.vision.RunningMode.LIVE_STREAM,
            # result_callback=self.print_result
            running_mode=mp.tasks.vision.RunningMode.IMAGE)
        self.recognizer = vision.GestureRecognizer.create_from_options(self.options)

    def print_result(self, result):
        msg = String()
        bool_msg = Bool()
        for gesture_list in result.gestures:
            for gesture in gesture_list:
                category_name = gesture.category_name
                score = gesture.score
                if category_name == "Thumb_Up" and score > 0.60:
                    msg.data = "thumbs_up" # SOLID ORANGE
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Detected Gesture: {category_name}! With score: {score}")
                    
                    bool_msg.data = True
                    self.drive_enabled_pub.publish(bool_msg)

                elif category_name == "Open_Palm" and score > 0.60:
                    msg.data = "idle" # FLASHING ORANGE
                    self.publisher_.publish(msg)

                    bool_msg.data = False
                    self.drive_enabled_pub.publish(bool_msg)

                    self.get_logger().info(f"Detected Gesture: {category_name}! With score: {score}")


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            numpy_frame_from_opencv = np.array(rgb_image)

            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=numpy_frame_from_opencv)

            # Process the image with Mediapipe Hands
            gesture_recognition_result = self.recognizer.recognize(mp_image)

            self.print_result(gesture_recognition_result)

            gesture_recognition_result = self.recognizer.recognize(mp_image)

        except CvBridgeError as e:
            print(e)

def main(args=None):

    rclpy.init(args=args)
    node = GestureRecognitionNode()

    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
