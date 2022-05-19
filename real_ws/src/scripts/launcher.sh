#!/bin/bash
gnome-terminal \
  --tab-with-profile=TEST --title=HW_Driver --command="roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch" 
sleep 2
gnome-terminal \
  --tab-with-profile=TEST --title=Moveit --command="roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=128.39.53.161" 
sleep 2
gnome-terminal \
  --tab-with-profile=TEST --title=RViz --command="roslaunch ur5_moveit_config moveit_rviz.launch config:=true"