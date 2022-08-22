cmake_minimum_required(VERSION 2.8.3)
project(franka_gazebo_moveit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  #PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
