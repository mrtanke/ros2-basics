import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster # static coordinate broadcaster
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

class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.static_broadcaster_ = StaticTransformBroadcaster(self)
        self.publish_static_tf()

    def publish_static_tf(self):
        """
        Publish static tf: coordinate relationship from base_link to camera_link 
        """
        transform = TransformStamped()
        transform.header.frame_id = 'base_link'  # parent frame
        transform.child_frame_id = 'camera_link'  # child frame
        transform.header.stamp = self.get_clock().now().to_msg()

        transform.transform.translation.x = 0.5  # x-axis translation (m)
        transform.transform.translation.y = 0.3  # y-axis translation
        transform.transform.translation.z = 0.6  # z-axis translation

        quaternion = quaternion_from_euler(math.radians(180), 0, 0)  # convert degrees to radians
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        # Publish the static transform
        self.static_broadcaster_.sendTransform(transform)
        self.get_logger().info('Published static transform from base_link to camera_link {}'.format(transform))

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()