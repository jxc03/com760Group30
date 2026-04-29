cmake_minimum_required(VERSION 3.0.2)
project(com760cw2_group30)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  message_generation
)

## Custom service files
## Reference: Assignment Brief (custom services requirement)
## W3 Lecture (Custom ROS Services)
add_service_files(
  FILES
  MineRescueSetBugStatus.srv
  MineRescueSetQLStatus.srv
)

## Custom message files
## Reference: Assignment Brief (custom messages requirement)
## W3 Lecture (Custom ROS Messages)
add_message_files(
  FILES
  RLAction.msg
  RLReward.msg
  SurvivorDetected.msg
)

## Generate messages and services with dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts so they are executable via rosrun/roslaunch
catkin_install_python(PROGRAMS
  scripts/GoToPoint.py
  scripts/FollowWall.py
  scripts/Bug2.py
  scripts/QLearning.py
  scripts/SurvivorDetector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
