#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <memory>
#include <string>

#include "nav2_core/exceptions.hpp"
#include "nav2_custom_planner/nav2_custom_planner.hpp"

namespace nav2_custom_planner
{

    void CustomPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        tf_ = tf;
        node_ = parent.lock();
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
        // Parameter initialization
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        node_->get_parameter(name_ + ".interpolation_resolution",
                             interpolation_resolution_);
    }

    void CustomPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin of type CustomPlanner %s",
                    name_.c_str());
    }

    void CustomPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "Activating plugin of type CustomPlanner %s",
                    name_.c_str());
    }

    void CustomPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "Deactivating plugin of type CustomPlanner %s",
                    name_.c_str());
    }

    nav_msgs::msg::Path
    CustomPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                              const geometry_msgs::msg::PoseStamped &goal)
    {
        // 1. declare and initialize global_path
        nav_msgs::msg::Path global_path;
        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        // 2. check target and intial state if in the global coordinates system
        if (start.header.frame_id != global_frame_)
        {
            RCLCPP_ERROR(node_->get_logger(), "Planner only accepts start positions from the %s frame",
                         global_frame_.c_str());
            return global_path;
        }

        if (goal.header.frame_id != global_frame_)
        {
            RCLCPP_ERROR(node_->get_logger(), "Planner only accepts goal positions from the %s frame",
                         global_frame_.c_str());
            return global_path;
        }

        // 3. calculate the number of loops and increments based on the current interpolation resolution
        int total_number_of_loop =
            std::hypot(goal.pose.position.x - start.pose.position.x,
                       goal.pose.position.y - start.pose.position.y) /
            interpolation_resolution_;
        double x_increment =
            (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
        double y_increment =
            (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

        // 4. generate the path
        for (int i = 0; i < total_number_of_loop; ++i)
        {
            geometry_msgs::msg::PoseStamped pose; // generate a point
            pose.pose.position.x = start.pose.position.x + x_increment * i;
            pose.pose.position.y = start.pose.position.y + y_increment * i;
            pose.pose.position.z = 0.0;
            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            // add the point to the path
            global_path.poses.push_back(pose);
        }

        // 5. use costmap to check if the path passes through any obstacles
        for (geometry_msgs::msg::PoseStamped pose : global_path.poses)
        {
            unsigned int mx, my; // convert the point's coordinates to grid coordinates
            if (costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
            {
                unsigned char cost = costmap_->getCost(mx, my); // get the cost value of the corresponding grid
                // if there is a lethal obstacle, throw an exception
                if (cost == nav2_costmap_2d::LETHAL_OBSTACLE)
                {
                    RCLCPP_WARN(node_->get_logger(), "Lethal obstacle detected at (%f,%f), planning failed.",
                                pose.pose.position.x, pose.pose.position.y);
                    throw nav2_core::PlannerException(
                        "Unable to create goal plan: " + std::to_string(goal.pose.position.x) + "," +
                        std::to_string(goal.pose.position.y));
                }
            }
        }

        // 6. wrap up, add the goal point as the last point of the path and return the path
        geometry_msgs::msg::PoseStamped goal_pose = goal;
        goal_pose.header.stamp = node_->now();
        goal_pose.header.frame_id = global_frame_;
        global_path.poses.push_back(goal_pose);
        return global_path;
    }

} // namespace nav2_custom_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner,
                       nav2_core::GlobalPlanner)