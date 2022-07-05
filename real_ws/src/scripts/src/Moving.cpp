#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur5");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "manipulator";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);



  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("ee_tool");
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  visual_tools.trigger();

  ROS_INFO_NAMED("tutorial", "Planning frame: %s", group.getPlanningFrame().c_str());

  ROS_INFO_NAMED("tutorial", "End effector link: %s", group.getEndEffectorLink().c_str());

  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(group.getJointModelGroupNames().begin(),
            group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));


  moveit::core::RobotStatePtr current_state = group.getCurrentState();
  moveit::core::RobotState start_state(*group.getCurrentState());


  group.clearPathConstraints();


  //creating a pose
  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.3;
  start_pose.position.y = -0.3;
  start_pose.position.z = 1.2;
  group.setStartState(start_state);
  group.move();

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose target_pose = start_pose;
  
  for(int i = -3; i>3; i++){
    std::string str = std::to_string(i);
    ROS_INFO_NAMED("yas", "test");
    double ii = i/10;
    target_pose.position.x = 0.3;
    target_pose.position.y = -ii;
    target_pose.position.z = 1.2;
    waypoints.push_back(target_pose);
  }
  


  group.setMaxVelocityScalingFactor(0.01);
  group.setMaxAccelerationScalingFactor(0.01);
  group.setGoalTolerance(0.01);


  moveit_msgs::RobotTrajectory trajectory;
  group.setPlanningTime(10.0);

  const double jump_threshold = 0.01;
  const double eef_step = 0.1;
  double fraction = group.computeCartesianPath(waypoints,
                                          eef_step,
                                          jump_threshold, 
                                          trajectory, 
                                          true);

  ROS_INFO_NAMED("completed", "visualizing cartesian path(%.2f%% achieved)",
              fraction * 100.0);


  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
  
  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory);

  my_plan.trajectory_ = trajectory;
  sleep(5.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  visual_tools.publishText(text_pose, "El plan", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  group.setStartState(start_state);
  group.execute(my_plan);

  ROS_INFO_NAMED("info", "executing trajectory");
  ros::shutdown();
  return 0;
}