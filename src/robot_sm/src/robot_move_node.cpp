/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  static const std::string PLANNING_GROUP = "g1";

  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.hpp>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);


  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Group: %s", move_group.getName().c_str());


  for (const auto& joint : move_group.getActiveJoints()) {
    RCLCPP_INFO(LOGGER, "Active joint: %s", joint.c_str());
}
  // Planning to a Pose goal

  // geometry_msgs::msg::Pose goal_pose;
  // goal_pose.orientation.w = 1.0;
  // goal_pose.position.x = 0.3;
  // goal_pose.position.y = 0.0;
  // goal_pose.position.z = 0.0;
  // move_group.setPoseTarget(goal_pose);

  geometry_msgs::msg::Pose curr_pose = move_group.getCurrentPose().pose;
  geometry_msgs::msg::Pose goal_pose = curr_pose;

  goal_pose.position.x = 0.0; 
  goal_pose.position.y = 0.0;
  goal_pose.position.z = 0.87;
  // goal_pose.pose.orientation.w = 1.0
  // moveit::core::RobotStatePtr kinematic_state = move_group.getCurrentState();
  // const moveit::core::JointModelGroup* joint_model_group =
  //     kinematic_state->getJointModelGroup(move_group.getName());

  RCLCPP_INFO(LOGGER, "Current Pose: x=%.3f y=%.3f z=%.3f",
    curr_pose.position.x, curr_pose.position.y, curr_pose.position.z);

  RCLCPP_INFO(LOGGER, "Goal Pose: x=%.3f y=%.3f z=%.3f",
    goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);


  // bool found_ik = kinematic_state->setFromIK(joint_model_group, goal_pose);

  // if (!found_ik) {
  //     RCLCPP_WARN(LOGGER, "IK solution not found for goal pose!");
  // }

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group.setPoseTarget(goal_pose);
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "planning results: %s", success ? "SUCCESS" : "FAILED");

  // Move the robot based on the plan
  if (success) {
    move_group.execute(my_plan);
  } else {
    RCLCPP_WARN(LOGGER, "Failed to execute the plan");
  }
  rclcpp::shutdown();
  return 0;
}