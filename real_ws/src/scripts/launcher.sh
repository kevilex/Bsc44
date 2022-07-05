#!/bin/bash
gnome-terminal \
  --tab-with-profile=Profile --title=HW_Driver --command="roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch" 
sleep 2
gnome-terminal \
  --tab-with-profile=Profile --title=Moveit --command="roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=128.39.53.159" 
sleep 2
gnome-terminal \
  --tab-with-profile=Profile --title=RViz --command="roslaunch ur5_moveit_config moveit_rviz.launch config:=true"
sleep 2
gnome-terminal \
  --tab-with-profile=Profile --title=RViz --command="roslaunch scripts camera.launch"
