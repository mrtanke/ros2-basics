import rclpy
from rclpy.node import Node

class PersonNode(Node):
    def __init__(self, node_name: str, name: str, age: int):
        super().__init__(node_name)
        self.name = name
        self.age = age

    def eat(self, food: str) -> str:
        self.get_logger().info(f"{self.name} is eating {food}.")
    

def main():
    rclpy.init()
    node = PersonNode("person_node_1", "Alice", 30)
    node.eat("apple")

    rclpy.spin(node)
    rclpy.shutdown()