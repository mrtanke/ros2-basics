import espeakng
import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue
import threading
import time


class NovelSubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'Node {node_name} has been started.')
        self.novels_queue_ = Queue()
        self.novel_subscriber_ = self.create_subscription(String, 'novel', self.novel_callback, 10) # novel_callback will be called whenever a message is received
        self.speech_thread_ = threading.Thread(target=self.speak_thread)
        self.speech_thread_.start()

    def novel_callback(self, msg):
        self.novels_queue_.put(msg.data)

    def speak_thread(self):
        speaker = espeakng.Speaker()
        speaker.voice = 'en'
        while rclpy.ok(): # check if ROS is still running
            if self.novels_queue_.qsize() > 0:
                text = self.novels_queue_.get()
                self.get_logger().info(f'Speaking: {text}...')
                speaker.say(text)
                speaker.wait()
            else:
                time.sleep(1)


def main():
    rclpy.init()
    node = NovelSubNode('novel_sub_node')

    rclpy.spin(node)
    rclpy.shutdown()
