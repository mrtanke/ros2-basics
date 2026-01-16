#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "chrono"
using namespace std::chrono_literals;

class TFListener : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;

public:
    TFListener() : Node("tf_listener")
    {
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        this->listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this);
        this->timer_ = this->create_wall_timer(
            6s,
            std::bind(&TFListener::getTransform, this));
    }

    void getTransform()
    {
        try
        {
            // query the coordinates relationship in buffer_
            const auto tranform = buffer_->lookupTransform(
                "base_link",
                "target_point",
                this->get_clock()->now(),
                rclcpp::Duration::from_seconds(6.0f));
            
            //get query result
            auto translation = tranform.transform.translation;
            auto rotation = tranform.transform.rotation;
            double y, p, r;
            tf2::getEulerYPR(rotation, y, p, r);
            RCLCPP_INFO(this->get_logger(), "Translation: [%.2f, %.2f, %.2f]", 
                translation.x, translation.y, translation.z);
            RCLCPP_INFO(this->get_logger(), "Rotation (YPR): [%.2f, %.2f, %.2f]", 
                y, p, r);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", e.what());  
        }
    }

    // void publish_tf()
    // {
    //     geometry_msgs::msg::TransformStamped transform;
    //     transform.header.stamp = this -> get_clock() -> now();
    //     transform.header.frame_id = "map";
    //     transform.child_frame_id = "base_link";
    //     transform.transform.translation.x = 2.0;
    //     transform.transform.translation.y = 3.0;
    //     transform.transform.translation.z = 0.0;

    //     tf2::Quaternion q;
    //     q.setRPY(0.0, 0.0, 30 * M_PI / 180.0);  // Roll
    //     transform.transform.rotation = tf2::toMsg(q);
    //     this -> listener_ -> sendTransform(transform);
    // }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}