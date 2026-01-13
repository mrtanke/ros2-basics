# Client node to call face detection service and update its parameters
import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
import cv2
from ament_index_python.packages import get_package_share_directory # get package absolute path
import os
from cv_bridge import CvBridge
import time
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class FaceDetectClientNode(Node):
    """
    Client node to call face detection service and update its parameters
    """
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.bridge = CvBridge()
        self.default_image_path = os.path.join(
            get_package_share_directory('demo_python_service'), 
            'resource', 'test1.jpg')
        self.get_logger().info('Face Detect Client Node is ready to send requests.')
        self.client = self.create_client(FaceDetector, 'face_detect')
        self.image = cv2.imread(self.default_image_path)
    
    def call_set_parameters(self, parameters):
        """
        Call the service and modify parameters on the server node.
        """
        # 1. create a client waiting for the service to be available
        update_param_client = self.create_client(SetParameters, '/face_detect_node/set_parameters')
        while update_param_client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info('Service not available, waiting again...')

        # 2. create request
        request = SetParameters.Request()
        request.parameters = parameters

        # 3. call service and update parameters
        future = update_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response
    
    def update_detect_model(self, model='hog'):
        """
        Construct parameters based on input model, 
        and call call_set_parameters to update server node parameters.
        """
        # 1. create parameter object
        param = Parameter()
        param.name = 'model'

        # 2. assign value based on input
        param_value = ParameterValue()
        param_value.string_value = model
        param_value.type = ParameterType.PARAMETER_STRING
        param.value = param_value

        # 3. request parameter update
        response = self.call_set_parameters([param])
        for result in response.results:
            if result.successful:
                self.get_logger().info(f'Parameter updated successfully: {result.successful} {result.reason}')
            else:
                self.get_logger().error('Failed to update parameter.')


    def send_request(self):
        # 1. check if inline image is available
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info('Service not available, waiting again...')
        
        # 2. create request
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)

        # 3. send request and wait for response
        def result_callback(result_future):
            response = result_future.result()
            self.get_logger().info('Received response: {} faces detected in {:.4f} seconds.'.format(
                response.number, response.use_time))
            # self.show_response(response)

        
        future = self.client.call_async(request)
        future.add_done_callback(result_callback)

    def show_response(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.imshow('Detected Faces', self.image)
        cv2.waitKey(0) # Wait for a key press to close the window


def main():
    rclpy.init()
    node = FaceDetectClientNode()
    
    # initially use hog model
    node.update_detect_model('hog')
    node.send_request()

    # wait for a while and switch to cnn model
    time.sleep(5)

    # switch to cnn model
    node.update_detect_model('cnn')
    node.send_request()

    rclpy.spin(node)
    rclpy.shutdown()