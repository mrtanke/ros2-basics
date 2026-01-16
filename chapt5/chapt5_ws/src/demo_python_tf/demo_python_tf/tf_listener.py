import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer # coordinate broadcaster
import math


def euler_from_quaternion(x: float, y: float, z: float, w: float):
    """Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).

    Returns:
        (roll, pitch, yaw) in radians
    """
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        self.timer_ = self.create_timer(1.0, self.get_transform)  # publish every second

    def get_transform(self):
        """
        Real-time query the coordinate relationship in buffer_
        """
        try:
            result = self.buffer_.lookup_transform('base_link', 'bottle_link', 
                                                   rclpy.time.Time(seconds=0), rclpy.duration.Duration(seconds=1.0))
            transform = result.transform
            self.get_logger().info(f'Translation: {transform.translation}')
            self.get_logger().info(f'Rotation: {transform.rotation}')
            
            rotation_euler = euler_from_quaternion(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            )
            self.get_logger().info(f'Rotation in Euler angles (radians): {rotation_euler}')
        except Exception as e:
            self.get_logger().warning(f'Could not transform: {e}')

def main():
    rclpy.init()
    node = TFListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()