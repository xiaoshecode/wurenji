cmake_minimum_required(VERSION 3.0.2)
project(wurenji_workspace)

# This file is the top-level CMake entry when this checkout is catkin_ws/src.
# Source your ROS installation's setup.bash before configuring the workspace.
set(CATKIN_TOPLEVEL TRUE)
find_package(catkin REQUIRED NO_POLICY_SCOPE)
catkin_workspace()
