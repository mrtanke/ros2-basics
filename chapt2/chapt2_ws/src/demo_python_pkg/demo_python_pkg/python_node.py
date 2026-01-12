import rclpy
from rclpy.node import Node

def main():
    rclpy.init() # initialize, allocate resources
    node = Node('python_node') # create node
    node.get_logger().info('Hello, Python node!') # log message
    node.get_logger().warn('Hello, Python node!') # log message
    rclpy.spin(node) # keep node alive to process callbacks
    rclpy.shutdown() # cleanup and free resources
