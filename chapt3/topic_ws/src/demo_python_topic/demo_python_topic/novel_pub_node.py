import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue

class NovelPubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'Node {node_name} has been started.')
        self.novels_queue_ = Queue() # Create a queue to store novels
        self.novel_publisher_ = self.create_publisher(String, 'novel', 10)
        self.create_timer(5, self.timer_callback)

    def timer_callback(self):
        # self.novel_publisher_.publish()
        if self.novels_queue_.qsize() > 0:
            line = self.novels_queue_.get()
            msg = String()
            msg.data = line
            self.novel_publisher_.publish(msg)
            self.get_logger().info(f'Published novel line: {msg}')

    def download(self, url):
        response = requests.get(url)
        response.encoding = 'utf-8'
        text = response.text
        self.get_logger().info(f'Downloaded content from {url},{len(text)} characters.')

        for line in text.splitlines():
            self.novels_queue_.put(line)


def main():
    rclpy.init()
    node = NovelPubNode('novel_pub_node')
    node.download('http://localhost:8000/novel/novel1.txt')

    rclpy.spin(node)
    rclpy.shutdown()