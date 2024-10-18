#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "hubert_grp");

    // Set a target Pose
    auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.000000;
    msg.position.x = 1e-6;
    msg.position.y = -0.442426;
    msg.position.z = 0.1996769;

    return msg;
    }();
    move_group_interface.setGoalPositionTolerance(0.5);
    move_group_interface.setGoalOrientationTolerance(0.5);
    move_group_interface.setPoseTarget(target_pose, "end_effector");
    move_group_interface.setPlanningTime(30);
    move_group_interface.getCurrentPose();
    move_group_interface.setPlannerId("PRMstarkConfigDefault");
    move_group_interface.setNumPlanningAttempts(50);
    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
    move_group_interface.execute(plan);
    } else {
    RCLCPP_ERROR(logger, "Planning failed!");
    }
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}