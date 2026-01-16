import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster # coordinate broadcaster
from geometry_msgs.msg import TransformStamped # message interface 
import math # angular to radian conversion


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Convert roll, pitch, yaw (radians) to quaternion (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.broadcaster_ = TransformBroadcaster(self)
        self.timer_ = self.create_timer(1.0, self.publish_tf)  # publish every second

    def publish_tf(self):
        """
        Publish tf: coordinate relationship from camera_link to bottle_link 
        """
        transform = TransformStamped()
        transform.header.frame_id = 'camera_link'  # parent frame
        transform.child_frame_id = 'bottle_link'  # child frame
        transform.header.stamp = self.get_clock().now().to_msg()

        transform.transform.translation.x = 0.2  # x-axis translation (m)
        transform.transform.translation.y = 0.3  # y-axis translation
        transform.transform.translation.z = 0.5  # z-axis translation

        quaternion = quaternion_from_euler(math.radians(180), 0, 0)  # convert degrees to radians
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        # Publish the transform
        self.broadcaster_.sendTransform(transform)
        self.get_logger().info('Published transform from camera_link to bottle_link {}'.format(transform))

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()