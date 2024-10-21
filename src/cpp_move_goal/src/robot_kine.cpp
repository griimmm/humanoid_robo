/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Michael Lautman*/

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("robot_model_and_state_tutorial", node_options);
  const auto& LOGGER = node->get_logger();

  // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using the RobotModel class is very easy. In
  // general, you will find that most higher-level components will
  // return a shared pointer to the RobotModel. You should always use
  // that when possible. In this example, we will start with such a
  // shared pointer and discuss only the basic API. You can have a
  // look at the actual code API for these classes to get more
  // information about how to use more features provided by these
  // classes.
  //
  // We will start by instantiating a
  // `RobotModelLoader`_
  // object, which will look up
  // the robot description on the ROS parameter server and construct a
  // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>` for us to use.
  //
  // .. _RobotModelLoader:
  //     https://github.com/ros-planning/moveit2/blob/main/moveit_ros/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.h
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "hubert_arm");

  // Using the :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`, we can
  // construct a :moveit_codedir:`RobotState<moveit_core/robot_state/include/moveit/robot_state/robot_state.h>` that
  // maintains the configuration of the robot. We will set all joints in the state to their default values. We can then
  // get a :moveit_codedir:`JointModelGroup<moveit_core/robot_model/include/moveit/robot_model/joint_model_group.h>`,
  // which represents the robot model for a particular group, e.g. the "panda_arm" of the Panda robot.
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("hubert_arm");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retrieve the current set of joint values stored in the state for the Panda arm.
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // Joint Limits
  // ^^^^^^^^^^^^
  // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  /* Set one joint in the Panda arm outside its joint limit */
  joint_values[0] = 5.57;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  /* Check whether any joint is outside its joint limits */
  RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  /* Enforce the joint limits for this state and check again*/
  kinematic_state->enforceBounds();
  RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  // Forward Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // Now, we can compute forward kinematics for a set of random joint
  // values. Note that we would like to find the pose of the
  // "panda_link8" which is the most distal link in the
  // "panda_arm" group of the robot.
  kinematic_state->setToRandomPositions(joint_model_group); //setToRandomPositions joint_model_group
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("robot_elbow");
  
  // /* Print end-effector pose. Remember that this is in the model frame */
  RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");
  RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");
  RCLCPP_INFO_STREAM(LOGGER, "Full: \n" << end_effector_state.matrix() << "\n");
  //  RCLCPP_INFO(logger, "Publishing: '%s'", std::to_string(gpt).c_str())

  // Inverse Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // We can now solve inverse kinematics (IK) for the Panda robot.
  // To solve IK, we will need the following:
  //
  //  * The desired pose of the end-effector (by default, this is the last link in the "panda_arm" chain):
  //    end_effector_state that we computed in the step above.
  //  * The timeout: 0.1 s
  double timeout = 5;
  // auto const target_pose = [] {
  //   geometry_msgs::msg::Pose msg;
  //   msg.orientation.x = 0.59816; //0.65952;//relative robot_elbow 0.0;//-0.659801;   //0.0;//0.660831;
  //   msg.orientation.y = -0.37708;//-0.25498;//relative robot_elbow0.0;//-0.255526;   //0.0;//-0.257464;
  //   msg.orientation.z = 0.5337;//0.25503;//relative robot_elbow0.13689;//0.255057;   //0.0;//0.255172;
  //   msg.orientation.w = -0.46388;//0.65953;//relative robot_elbow 0.99059;//0.659024;   //1.0;//0.657191;
  //   msg.position.x = 0.081335;//0.086748;//relative robot_elbow 0.015; //0.0882441;  //-0.093624;
  //   msg.position.y = 0.11549;//-0.103;//relative robot_elbow -0.088;//-0.101586;  //0.096507;
  //   msg.position.z = 0.59834;//0.29798;//relative robot_elbow -0.005;//0.300683;  //0.310392;
  //   return msg;
  // }();
  // const Eigen::Translation3d translation(0.0, 0.0, 0.0);

  // Define the quaternion
  // const Eigen::Quaterniond quat(0.0, 0.0, 0.0, 1.0);
  // Eigen::Matrix3d rotation;
  // rotation = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  // quat.normalize();
  const Eigen::Isometry3d& pose = Eigen::Isometry3d::Identity();
  RCLCPP_INFO_STREAM(LOGGER, "Full: \n" << pose.matrix() << "\n");
 //translation * quat;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);
  
  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    std::vector<double> goal = {joint_values[0], joint_values[1], joint_values[2]};
    move_group_interface.setJointValueTarget(goal);

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
      RCLCPP_ERROR(LOGGER, "Planning failed!");
    }
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Did not find IK solution");
  }

  // Get the Jacobian
  // ^^^^^^^^^^^^^^^^
  // We can also get the Jacobian from the :moveit_codedir:`RobotState<moveit_core/robot_state/include/moveit/robot_state/robot_state.h>`.
  // Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  // Eigen::MatrixXd jacobian;
  // kinematic_state->getJacobian(joint_model_group,
  //                              kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
  //                              reference_point_position, jacobian);
  // RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");
  // auto const target_pose = [] {
  //   Eigen::Affine3d msg;
  //   // msg.orientation.x = 0.660831;
  //   // msg.orientation.y = -0.257464;
  //   // msg.orientation.z = 0.255172;
  //   msg.orientation.w = 1.0;
  //   msg.position.x = 0.0; //0.093624;
  //   msg.position.y = 0.0; //-0.096507;
  //   msg.position.z = 0.0; //0.310392;
  //   return msg;
  // }();
  

  // END_TUTORIAL

  rclcpp::shutdown();
  return 0;
}