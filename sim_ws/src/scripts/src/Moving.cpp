#include <ros/ros.h>
#include <cmath>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


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
  static const std::string PLANNING_GROUP = "arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  // Visualization

  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("ee_tool");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "COBOT photo slave", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());


  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

/*


  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a po
  // se goal for our group.
  // First define the path constraint.
  /*
  moveit_msgs::VisibilityConstraint vcm;
  vcm.target_radius = 10;
  vcm.target_pose = "object";
  vcm.cone_sides = 1.0;
  vcm.absolute_x_axis_tolerance = 0.3;
  vcm.absolute_y_axis_tolerance = 0.3;
  vcm.absolute_z_axis_tolerance = 0.3;
  vcm.weight = 0.5;
  vcm.
  */

  // Now, set it as the path constraint for the group.
  //moveit_msgs::Constraints test_constraints;
  //test_constraints.VisibilityConstraint.push_back(vcm);
  //move_group_interface.setPathConstraints(test_constraints);

  // Enforce Planning in Joint Space
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Depending on the planning problem MoveIt chooses between
  // ``joint space`` and ``cartesian space`` for problem representation.
  // Setting the group parameter ``enforce_joint_model_state_space:true`` in
  // the ompl_planning.yaml file enforces the use of ``joint space`` for all plans.
  //
  // By default planning requests with orientation path constraints
  // are sampled in ``cartesian space`` so that invoking IK serves as a
  // generative sampler.
  //
  // By enforcing ``joint space`` the planning process will use rejection
  // sampling to find valid requests. Please note that this might
  // increase planning time considerably.
  //
  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already
  // satisfies the path constraints. So we need to set the start
  // state to a new pose.


  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;


  start_pose2.position.x = ((cos(0/100)*sin(0/100))/5.0) + 0.5;
  start_pose2.position.y = (sin((0/100) + (3.14/2.0)))/5;
  start_pose2.position.z = ((sin(0/100)*(sin(0/100)))/5.0) + 0.2;
  move_group_interface.setStartState(start_state);


  move_group_interface.setPoseTarget(start_pose2);

  move_group_interface.move();

  //moveit_msgs::VisibilityConstraint VCM;
  //VCM.max_view_angle = 1.1;
  //VCM.max_range_angle = 0.55;

  //kinematic_constraints::VisibilityConstraint 
  

  //move_group_interface.setPathConstraints(VCM);

  std::vector<geometry_msgs::Pose> waypoints;
  
  double first = 0;
  double prev = 1;
  double count;
  for (int e = 0; e <= 314; e = e + 31){

    double y = sin((e/100) + (3.14/2.0));
    double r = sin(e/100);

    for(int i = 0; i <= 314; i = i + 3){
      first = ((cos(i/100)*r)/5.0) + 0.5;
      if(first != prev){
        
        double x = ((cos(i/100)*r)/5.0) + 0.5;
        double y = y/5.0;
        double z = ((sin(i/100)*r)/5.0) + 0.3;

        geometry_msgs::Pose target_pose3 = start_pose2;
        target_pose3.position.x = x;
        target_pose3.position.y = y;
        target_pose3.position.z = z;

        waypoints.push_back(target_pose3);
      }
      prev = ((cos(i/100)*r)/5.0) + 0.5;
      count = count++;
    }
  }
  
    ROS_INFO_NAMED("tutorial", "(%.2f%% points)", count);


  move_group_interface.setMaxVelocityScalingFactor(0.02);
  move_group_interface.setMaxAccelerationScalingFactor(0.02);
  move_group_interface.setGoalTolerance(0.01);
  move_group_interface.setPlanningTime(20.0);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double eef_step = 0.01;
  const double jump_threshold = 0.00;
  double fraction = move_group_interface.computeCartesianPath(waypoints, 
                                          eef_step, 
                                          jump_threshold, 
                                          trajectory,
                                          true);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
  // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
  // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
  // Pull requests are welcome.
  //
  // You can execute a trajectory like this.
  sleep(5.0);
  move_group_interface.setStartState(start_state);
  move_group_interface.execute(trajectory);
  ROS_INFO_NAMED("info", "executing trajectory");
  ros::shutdown();
  return 0;
}