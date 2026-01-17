#include "nav2_custom_controller/custom_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace nav2_custom_controller {
void CustomController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;

  // declare and get the parameters, set max linear and angular speeds
  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(0.1));
  node_->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(1.0));
  node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
}

void CustomController::cleanup() {
  RCLCPP_INFO(node_->get_logger(),
              "Cleaning up controller: %s of type nav2_custom_controller::CustomController",
              plugin_name_.c_str());
}

void CustomController::activate() {
  RCLCPP_INFO(node_->get_logger(),
              "Activating controller: %s of type nav2_custom_controller::CustomController",
              plugin_name_.c_str());
}

void CustomController::deactivate() {
  RCLCPP_INFO(node_->get_logger(),
              "Deactivating controller: %s of type nav2_custom_controller::CustomController",
              plugin_name_.c_str());
}

geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *) {
    // 1. check the path validity
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received a path with zero length");
  }

  // 2. Transform the robot's current pose to the global plan frame
  geometry_msgs::msg::PoseStamped pose_in_globalframe;
  if (!nav2_util::transformPoseInTargetFrame(
          pose, pose_in_globalframe, *tf_, global_plan_.header.frame_id, 0.1)) {
    throw nav2_core::PlannerException("Failed to transform robot pose to global plan frame");
  }

  // 3. Get the nearest target pose and calculate the angle difference
  auto target_pose = getNearestTargetPose(pose_in_globalframe);
  auto angle_diff = calculateAngleDifference(pose_in_globalframe, target_pose);

  // 4. Calculate linear and angular velocities based on the angle difference
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose_in_globalframe.header.frame_id;
  cmd_vel.header.stamp = node_->get_clock()->now();
  // Calculate velocity based on angle difference, rotate in place if angle difference is greater than 0.3, otherwise move straight
  if (fabs(angle_diff) > M_PI/10.0) {
    cmd_vel.twist.linear.x = .0;
    cmd_vel.twist.angular.z = fabs(angle_diff) / angle_diff * max_angular_speed_;
  } else {
    cmd_vel.twist.linear.x = max_linear_speed_;
    cmd_vel.twist.angular.z = .0;
  }
  RCLCPP_INFO(node_->get_logger(), "Controller: %s sending velocity (%f,%f)",
              plugin_name_.c_str(), cmd_vel.twist.linear.x,
              cmd_vel.twist.angular.z);
  return cmd_vel;
}

void CustomController::setSpeedLimit(const double &speed_limit,
                                     const bool &percentage) {
  (void)percentage;
  (void)speed_limit;
}

void CustomController::setPlan(const nav_msgs::msg::Path &path) {
  global_plan_ = path;
}

geometry_msgs::msg::PoseStamped CustomController::getNearestTargetPose(
    const geometry_msgs::msg::PoseStamped &current_pose) {
   // 1. Iterate through the path to find the index of the point closest to the current pose, store it in nearest_pose_index
  using nav2_util::geometry_utils::euclidean_distance;
  int nearest_pose_index = 0;
  double min_dist = euclidean_distance(current_pose, global_plan_.poses.at(0));
  for (unsigned int i = 1; i < global_plan_.poses.size(); i++) {
    double dist = euclidean_distance(current_pose, global_plan_.poses.at(i));
    if (dist < min_dist) {
      nearest_pose_index = i;
      min_dist = dist;
    }
  }
  // 2. Erase the path from the beginning to the nearest point
  global_plan_.poses.erase(std::begin(global_plan_.poses),
                           std::begin(global_plan_.poses) + nearest_pose_index);
  // 3. If there is only one point, return it directly; otherwise, return the next point after the nearest point
  if (global_plan_.poses.size() == 1) {
    return global_plan_.poses.at(0);
  }
  return global_plan_.poses.at(1);
}

double CustomController::calculateAngleDifference(
    const geometry_msgs::msg::PoseStamped &current_pose,
    const geometry_msgs::msg::PoseStamped &target_pose) {
 // Calculate the angle difference between the current pose and the target pose
  // 1. Get the current yaw angle
  float current_robot_yaw = tf2::getYaw(current_pose.pose.orientation);
  // 2. Get the target angle
  float target_angle =
      std::atan2(target_pose.pose.position.y - current_pose.pose.position.y,
                 target_pose.pose.position.x - current_pose.pose.position.x);
  // 3. Calculate the angle difference and normalize it to the range -M_PI to M_PI
  double angle_diff = target_angle - current_robot_yaw;
  if (angle_diff < -M_PI) {
    angle_diff += 2.0 * M_PI;
  } else if (angle_diff > M_PI) {
    angle_diff -= 2.0 * M_PI;
  }
  return angle_diff;
}
} // namespace nav2_custom_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController,nav2_core::Controller)