#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

/*
Subscribes to cv_coord topic and updates the goal positions sent to moveit_group_interface accordingly.
*/
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("move_hubert"), message_(geometry_msgs::msg::Pose())
  {
    auto topic_callback =
      [this](geometry_msgs::msg::Pose msg) -> void {
        message_ = msg;
      };
    subscription_ =
      this->create_subscription<geometry_msgs::msg::Pose>("cv_coord", 10, topic_callback);
      
      timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&MinimalSubscriber::timer_callback, this));
  }
  
private:
  geometry_msgs::msg::Pose message_; 
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;  

  void timer_callback()
  {
    auto const logger = this->get_logger();

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(shared_from_this(), "hubert_arm");
  move_group_interface.setGoalPositionTolerance(0.3); //current tested tolerance 23 Oct
  move_group_interface.setGoalOrientationTolerance(0.25); //current tested tolerance 23 Oct
  move_group_interface.setPlanningTime(30);

  //Set a target pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = 0.6205310755024788;  // ros2 topic pub /cv_coord geometry_msgs/msg/Pose "{position: {x: 0.29591830149428244, y: -0.09801094170013577, z: 0.07800292822954993}, orientation: {x: 0.6205310755024788, y: -0.33899560607520735, z: 0.3390209866150409, w: 0.6205545375162478}}"
  target_pose.orientation.y = -0.33899560607520735; //working quaternion values
  target_pose.orientation.z = 0.3390209866150409;
  target_pose.orientation.w = 0.6205545375162478;
  target_pose.position = message_.position;
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

  } 
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0; 
}