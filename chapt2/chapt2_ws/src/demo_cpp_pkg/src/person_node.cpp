#include "rclcpp/rclcpp.hpp"

class PersonNode : public rclcpp::Node
{
private:
    std::string name_;
    int age_;

public:
    PersonNode(const std::string & node_name, const std::string & name, const int & age)
    :Node(node_name) // python: super().__init__(node_name)
    {
        this -> name_ = name;
        this -> age_ = age;
    }


    void eat(const std::string & food)
    {
        RCLCPP_INFO(this->get_logger(), "%s %d is eating %s.", this->name_.c_str(), this->age_, food.c_str());
    }   
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PersonNode>("person_node_1", "Alice", 30);
    RCLCPP_INFO(node->get_logger(), "PersonNode has been started.");
    node->eat("apple");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}