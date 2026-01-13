# Service node to perform face detection on received images and handle parameter updates
import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory # get package absolute path
import os
from cv_bridge import CvBridge
import time
from rcl_interfaces.msg import SetParametersResult

class FaceDetectNode(Node):
    """
    Service node to perform face detection on received images and handle parameter updates
    """
    def __init__(self):
        super().__init__('face_detect_node')
        self.service_ = self.create_service(FaceDetector, 'face_detect', self.detect_face_callback)
        self.bridge = CvBridge()

        self.declare_parameter('number_of_times_to_upsample', 1)
        self.declare_parameter('model', 'hog')
        self.number_of_times_to_upsample = self.get_parameter('number_of_times_to_upsample').value
        self.model = self.get_parameter('model').value

        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource', 'default.jpg')
        self.get_logger().info('Face Detect Node is ready to receive requests.')

        self.add_on_set_parameters_callback(self.parameter_callback)

        # set its own parameters
        self.set_parameters([rclpy.Parameter('model', rclpy.Parameter.Type.STRING, 'cnn')])

    def parameter_callback(self, parameters):
        for parameter in parameters:
            self.get_logger().info(f'Parameter {parameter.name} changed to {parameter.value}')
            if parameter.name == 'number_of_times_to_upsample':
                self.number_of_times_to_upsample = parameter.value
            elif parameter.name == 'model':
                self.model = parameter.value
        return SetParametersResult(successful=True)


    def detect_face_callback(self, request, response):
        # 1. Convert ROS Image message to OpenCV image
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.default_image_path)
            self.get_logger().info('No image data received. Using default image.')
    
        # cv_image is now a valid OpenCV image
        start_time = time.time()
        self.get_logger().info('Starting face detection...')

        # 2. Detect faces in the image using face_recognition
        face_locations = face_recognition.face_locations(
            cv_image, 
            number_of_times_to_upsample=self.number_of_times_to_upsample, 
            model=self.model
        )
        response.use_time = time.time() - start_time
        response.number = len(face_locations)

        # 3. Populate response with face locations
        for (top, right, bottom, left) in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)

        return response


def main():
    rclpy.init()
    node = FaceDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()