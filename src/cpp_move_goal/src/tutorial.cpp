#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "hubert_arm");
  move_group_interface.setGoalPositionTolerance(0.5);
  move_group_interface.setGoalOrientationTolerance(0.5);
  move_group_interface.setPlanningTime(30);
  // Set a target Pose
// pose_goal.position.x = 0.29591830149428244
// pose_goal.position.y = -0.09801094170013577
// pose_goal.position.z = 0.07800292822954993
// pose_goal.orientation.x = 0.6205310755024788
// pose_goal.orientation.y = -0.33899560607520735
// pose_goal.orientation.z = 0.3390209866150409
// pose_goal.orientation.w = 0.6205545375162478
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.6205310755024788;// 0.660831;
    msg.orientation.y = -0.33899560607520735;// -0.257464;
    msg.orientation.z = 0.3390209866150409;// 0.255172;
    msg.orientation.w = 0.6205545375162478;// 1.0;
    msg.position.x =  0.29591830149428244; //0.093624;
    msg.position.y =  -0.09801094170013577; //-0.096507;
    msg.position.z =  0.07800292822954993; //0.310392;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}