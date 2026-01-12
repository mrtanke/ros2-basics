import rclpy

from demo_python_pkg.person_node import PersonNode

class WriterNode(PersonNode):
    def __init__(self, node_name: str, name: str, age: int, book:str):
        super().__init__(node_name=node_name, name=name, age=age)
        self.book = book
    

def main():
    rclpy.init()
    node = WriterNode("writer_node_1", "Writer Name", 40, "The Great Novel")
    node.eat("sandwich")
    node.destroy_node()
    rclpy.shutdown()