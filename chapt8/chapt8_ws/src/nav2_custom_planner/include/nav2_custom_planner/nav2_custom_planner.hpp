#ifndef NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_
#define NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_
#include <memory>
#include <string>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_custom_planner
{
  // Self define CustomPlanner class
  class CustomPlanner : public nav2_core::GlobalPlanner
  {
  public:
    CustomPlanner() = default;
    ~CustomPlanner() = default;
    // plugin configure method
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    // plugin cleanup method
    void cleanup() override;
    // plugin activate method
    void activate() override;
    // plugin deactivate method
    void deactivate() override;
    // method to create a plan for given start and goal poses
    nav_msgs::msg::Path
    createPlan(const geometry_msgs::msg::PoseStamped &start,
               const geometry_msgs::msg::PoseStamped &goal) override;

  private:
    // Coordinate transform buffer pointer, used to query coordinate relationships
    std::shared_ptr<tf2_ros::Buffer> tf_;
    // Node pointer
    nav2_util::LifecycleNode::SharedPtr node_;
    // Global costmap
    nav2_costmap_2d::Costmap2D *costmap_;
    // Global costmap frame
    std::string global_frame_, name_;
    // Interpolation resolution
    double interpolation_resolution_;
  };

} // namespace nav2_custom_planner

#endif // NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_