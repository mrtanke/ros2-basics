// Service node to control the turtle to move to a target position
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "chapt4_interfaces/srv/partol.hpp"
#include <chrono>
#include <functional>
#include <cmath>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using Partol = chapt4_interfaces::srv::Partol;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;
using namespace std::chrono_literals;

class TurtleController : public rclcpp::Node
{
private:
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rclcpp::Service<Partol>::SharedPtr partol_service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
    double target_x_ {1.0};
    double target_y_ {1.0};
    double k_ {1.0}; // Proportional gain for control
    double max_speed_ {3.0}; // Maximum linear speed

public:
    explicit TurtleController(): Node("turtle_control") // constructor
    {
        this -> declare_parameter("k", 1.0);
        this -> declare_parameter("max_speed", 1.0);
        this -> get_parameter("k", k_);
        this -> get_parameter("max_speed", max_speed_);
        this -> set_parameter(rclcpp::Parameter("k", 2.0));

        parameter_callback_handle_ = this-> add_on_set_parameters_callback([&](const std::vector<rclcpp::Parameter> 
            & parameters) -> rcl_interfaces::msg::SetParametersResult {
                SetParametersResult result;
                result.successful = true;

                for (const auto & parameter : parameters) {
                    if (parameter.get_name() == "k") {
                        k_ = parameter.as_double();
                        RCLCPP_INFO(this->get_logger(), "Parameter 'k' updated to: %.2f", k_);
                    } else if (parameter.get_name() == "max_speed") {
                        max_speed_ = parameter.as_double();
                        RCLCPP_INFO(this->get_logger(), "Parameter 'max_speed' updated to: %.2f", max_speed_);
                    }
                }
                return result;
        });
        partol_service_ = this->create_service<Partol>("partol", 
            [&](const Partol::Request::SharedPtr request, Partol::Response::SharedPtr response) -> void {
                if((0<request->target_x && request->target_x<12.0f)&&
                   (0<request->target_y && request->target_y<12.0f)){
                    this->target_x_ = request->target_x;
                    this->target_y_ = request->target_y;
                    response->result = Partol::Response::SUCCESS;
                } else {
                    response->result = Partol::Response::FAIL;
                }
        });
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose",
            10,
            std::bind(&TurtleController::on_pose_received_, this, std::placeholders::_1)
        );
    }

    void on_pose_received_(const turtlesim::msg::Pose::SharedPtr pose)
    {
        // 1. Get current position
        auto current_x = pose->x;
        auto current_y = pose->y;
        RCLCPP_INFO(this->get_logger(), "Current Position: (%.2f, %.2f)", current_x, current_y);

        // 2. Compute the distance and angular difference between turtle and target
        auto distance = std::sqrt(
            (target_x_ - current_x) * (target_x_ - current_x) +
            (target_y_ - current_y) * (target_y_ - current_y)
        );
        auto angle = std::atan2(target_y_ - current_y, target_x_ - current_x) - pose->theta;

        // 3. Control policy
        auto msg = geometry_msgs::msg::Twist();
        if(distance > 0.1){
            if(fabs(angle) > 0.2){
                msg.angular.z = fabs(angle);
            }else{
                msg.linear.x = k_ * distance;
            }
        }

        // 4. Limit the maximum speed
        if(msg.linear.x > max_speed_){
            msg.linear.x = max_speed_;
        }

        // 5. Publish the velocity command
        publisher_->publish(msg);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}