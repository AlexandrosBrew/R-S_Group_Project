#!/bin/bash

cd ~/R-S_Group_Project/ros2_ws || exit
colcon build --packages-select GazeboRobotSim
source install/setup.bash

echo "GazeboRobotSim rebuilt"
