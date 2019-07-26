cmake_minimum_required(VERSION 2.8.3)
project(rosbot_patrol_simulation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  frontier_exploration
  geometry_msgs
  gmapping
  move_base
  roscpp
  rospy
  tf
  darknet_ros
)

find_package( OpenCV REQUIRED )


add_message_files(FILES
  EspTrigger.msg
)


generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(patrol_robot_simulation src/patrol_robot_simulation.cpp src/PatrolManager.cpp)
add_dependencies(patrol_robot_simulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(patrol_robot_simulation
  ${catkin_LIBRARIES} yaml-cpp
)
