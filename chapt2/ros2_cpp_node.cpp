#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create a node named "cpp_node"
    auto node = rclcpp::Node::make_shared("cpp_node");

    // Log an informational message
    RCLCPP_INFO(node->get_logger(), "Hello from my C++ ROS 2 node!");

    // Keep the node alive until it is shut down
    rclcpp::spin(node);

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();
    return 0;
}