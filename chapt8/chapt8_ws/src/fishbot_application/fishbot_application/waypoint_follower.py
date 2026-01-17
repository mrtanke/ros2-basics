from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()
    goal_poses = []

    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = "map"
    goal_pose1.header.stamp = nav.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 2.0
    goal_pose1.pose.position.y = 1.0
    goal_pose1.pose.orientation.w = 1.0
    goal_poses.append(goal_pose1)

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = "map"
    goal_pose2.header.stamp = nav.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 1.0
    goal_pose2.pose.position.y = 2.0
    goal_pose2.pose.orientation.w = 1.0
    goal_poses.append(goal_pose2)

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = "map"
    goal_pose3.header.stamp = nav.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 0.0
    goal_pose3.pose.position.y = 0.0
    goal_pose3.pose.orientation.w = 1.0
    goal_poses.append(goal_pose3)

    nav.follow_waypoints(goal_poses)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        nav.get_logger().info(f'Current waypoint: {feedback.current_waypoint}')

    result = nav.getResult()
    nav.get_logger().info('Navigation result: ' + str(result))

    # rclpy.spin(nav)
    # rclpy.shutdown()