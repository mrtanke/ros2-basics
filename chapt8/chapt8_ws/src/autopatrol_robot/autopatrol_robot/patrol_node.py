import rclpy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from autopatrol_interfaces.srv import SpeechText
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 

np.float = float
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)

        # Declare relevant parameters
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0]) 
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.declare_parameter('img_save_path', '')

        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value 
        self.img_save_path_ = self.get_parameter('img_save_path').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cv_bridge = CvBridge()
        self.speech_client_ = self.create_client(SpeechText, 'speech_text')
        self.latest_img_ = None
        self.img_sub_ = self.create_subscription(Image, '/camera_sensor/image_raw', self.img_callback, 1)

    def img_callback(self, msg):
        """
        Callback function for image subscription
        """
        self.latest_img_ = msg
    
    def record_img(self):
        if self.latest_img_ is not None:
            pose = self.get_current_pose()
            cv_img = self.cv_bridge.imgmsg_to_cv2(self.latest_img_, desired_encoding='bgr8')
            cv2.imwrite(
                f'{self.img_save_path_}img_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png',
                cv_img
            )


    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        return a PoseStamped object given x, y, yaw
        """
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y

        quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def init_robot_pose(self):
        """
        Initialize robot pose in the simulation environment
        """
        self.initial_point_ = self.get_parameter('initial_point').value
        init_pose = self.get_pose_by_xyyaw(self.initial_point_[0], self.initial_point_[1], self.initial_point_[2])
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active() # wait for navigation2 to become active
    
    def get_target_points(self):
        """
        Get target point arrays from parameters
        """
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for i in range(0, int(len(self.target_points_)), 3):
            x = self.target_points_[i]
            y = self.target_points_[i+1]
            yaw = self.target_points_[i+2]
            
            points.append([x, y, yaw])
            self.get_logger().info(f'Target point added {i}: x={x}, y={y}, yaw={yaw}')
        return points
    
    def nav_to_pose(self, target_point):
        """
        Navigate to the target point
        """
        self.goToPose(target_point)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            self.get_logger().info('Distance remaining: ' + str(feedback.distance_remaining))
        result = self.getResult()
        self.get_logger().info('Navigation result: ' + str(result))
    
    def get_current_pose(self):
        """
        Get the current pose of the robot
        """
        while rclpy.ok():
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
                transform = tf.transform
                self.get_logger().info(f'Translation: {transform.translation}')
                return transform
            except Exception as e:
                self.get_logger().warn(f'Unable to lookup transform: {str(e)}')

    def speech_text(self, text):
        """
        Call the speech service to speak the given text
        """
        while not self.speech_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        request = SpeechText.Request()
        request.text = text

        future = self.speech_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            if response.result == True:
                self.get_logger().info(f'Successfully spoke text: "{text}"')
            else:
                self.get_logger().warn(f'Failed to speak text: "{text}"')
        else:
            self.get_logger().warn('Service call failed')

def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.speech_text("Initializing localization.")
    patrol.init_robot_pose()
    patrol.speech_text('Initializing localization complete.')

    while rclpy.ok():
        target_points = patrol.get_target_points()
        for point in target_points:
            x, y, yaw = point[0], point[1], point[2]
            target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
            patrol.speech_text(f'Navigating to point x={x}, y={y}, yaw={yaw}')
            patrol.nav_to_pose(target_pose)
            patrol.speech_text(f'Arrived at point x={x}, y={y}, yaw={yaw}, prepare to record image.')
            patrol.record_img()
            patrol.speech_text(f'Image recorded successfully.')
    
    rclpy.shutdown()