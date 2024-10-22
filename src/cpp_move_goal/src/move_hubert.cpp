#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

auto message = geometry_msgs::msg::Pose();


/*
Subscribes to cv_coord topic and updates the goal positions sent to moveit_group_interface accordingly.
*/
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    auto topic_callback =
      [this](geometry_msgs::msg::Pose msg) -> void {
        message = msg;
      };
    subscription_ =
      this->create_subscription<geometry_msgs::msg::Pose>("topic", 10, topic_callback);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

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
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = message.orientation.x;
    msg.orientation.y = message.orientation.y;
    msg.orientation.z = message.orientation.z;
    msg.orientation.w = message.orientation.w;
    msg.position.x = message.position.x; 
    msg.position.y = message.position.y;
    msg.position.z = message.position.z;
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