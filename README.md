# BSC workspaces
Simulation:

The realsense packages are used to get the camera working in the simulation.

The "scripts" folder have been used to test functionality as getting goal tolerances, tf-data, pointcloud data and setting new trajectories. This is where we will be creating our scripts throughout the project.

The "ur_description" folder is a source for the urdfs for both the ur5 & the camera (d435 for sim purposes as we did not find the required files for d455), launch files for the ur5 hw_interface and other files for the simulation.

The "ursim" folder is the result of the moveit setup assistant. This contains the config files for the movement of the robot. It is slightly modified, it now also launches the camera, spawns the box that the robot is placed on and it also launches a pcl nodelet aswell as the octomap server. We also added a static_transform_publisher to make sure the camera was appropriately placed according to the robot (modified in demo.launch).

How to use:
1. catkin_make
2. source devel/setup.bash
3. roslaunch ursim demo_gazebo.launch

This will launch everything necessary for the simulation, we either modify the launch file in scripts for testing or we just run the scripts directly from visual studio code.
