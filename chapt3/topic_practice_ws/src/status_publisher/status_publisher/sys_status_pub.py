import rclpy
from status_interfaces.msg import SystemStatus
from rclpy.node import Node
import psutil
import platform

class SysStatusPub(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.status_publisher_ = self.create_publisher(
            SystemStatus, 'sys_status', 10
        )
        self.timer_ = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        """
        builtin_interfaces/Time stamp # Record time stamp
        string host_name # host name 
        float32 cpu_percent # cpu usage rate
        float32 memory_percent # memory usage rate
        float32 memory_total # total memory size
        float32 memory_available # total memory available size
        float64 net_sent # total amount of data sent over network
        float64 net_recv # total amount of data received over network
        """
        cpu_percent = psutil.cpu_percent()
        memory_info = psutil.virtual_memory()
        net_io_counters= psutil.net_io_counters()

        msg = SystemStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.host_name = platform.node()
        msg.cpu_percent = cpu_percent
        msg.memory_percent = memory_info.percent
        msg.memory_total = float(memory_info.total)
        msg.memory_available = float(memory_info.available)
        msg.net_sent = net_io_counters.bytes_sent / 1024 / 1024
        msg.net_recv = net_io_counters.bytes_recv / 1024 / 1024

        self.get_logger().info(f'Publishing System Status: {str(msg)}')
        self.status_publisher_.publish(msg)

def main():
    rclpy.init()
    node = SysStatusPub('sys_status_pub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()