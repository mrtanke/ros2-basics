// Client node that requests a turtle to move to a specified location
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <ctime>
#include "chapt4_interfaces/srv/partol.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

using SetP = rcl_interfaces::srv::SetParameters;
using Partol = chapt4_interfaces::srv::Partol;
using namespace std::chrono_literals;

class PartolClient : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Partol>::SharedPtr partol_client_;

public:
    explicit PartolClient(): Node("partol_client") // constructor
    {
        srand(time(NULL)); // initialize random seed
        partol_client_ = this->create_client<Partol>("partol");
        timer_ = this->create_wall_timer(
            10s, [this]()->void {
                // 1. check if service is available
                while (!this->partol_client_->wait_for_service(1s)) {
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                        return;
                    }
                    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
                }

                // 2. create a request
                auto request = std::make_shared<Partol::Request>();
                request -> target_x = rand() % 15;
                request -> target_y = rand() % 15;
                RCLCPP_INFO(this->get_logger(), "Prepared target point (%.2f, %.2f)", request->target_x, request->target_y);

                // 3. send the request
                this -> partol_client_ -> async_send_request(
                    request, [this](rclcpp::Client<Partol>::SharedFuture result_future)->void {
                        auto response = result_future.get();
                        if(response->result == Partol::Response::SUCCESS){
                            RCLCPP_INFO(this->get_logger(), "Turtle is moving to the target point successfully.");
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Turtle failed to move to the target point.");
                        }
                });
            });
    }

    SetP::Response::SharedPtr call_set_parameters(const rcl_interfaces::msg::Parameter &param){
        /**
         * Create a client to sent requests and receive responses.
         */
        auto param_client = this->create_client<SetP>("/turtle_control/set_parameters");

        // 1. check if service is available
        while (!param_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return nullptr;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // 2. create a request
        auto request = std::make_shared<SetP::Request>();
        request -> parameters.push_back(param);

        // 3. send the request
        auto future = param_client -> async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        auto response = future.get();
        
        return response;

    }

    void update_server_param_k(double k){
        /**
         * Update the server parameter 'k' to the given value.
         **/
        // 1. create parameter object
        auto param = rcl_interfaces::msg::Parameter();
        param.name = "k";
        // 2. set parameter value
        auto param_value = rcl_interfaces::msg::ParameterValue();
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_value.double_value = k;
        param.value = param_value;
        // 3. request to set parameter
        auto response = this -> call_set_parameters(param);
        if(response==nullptr){
            RCLCPP_INFO(this->get_logger(), "Failed to call set_parameters service.");
            return;
        }

        for(auto result:response->results){
            if(result.successful==false){
                RCLCPP_WARN(this->get_logger(), "Failed to set parameter 'k'. reason: %s", result.reason.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Parameter 'k' set successfully.");
            }
        }
    }

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PartolClient>();
    node -> update_server_param_k(4.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}