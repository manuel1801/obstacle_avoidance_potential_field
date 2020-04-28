cmake_minimum_required(VERSION 2.8.3)

project(obstacle_avoidance_potential_field)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  rtt_ros
 )

catkin_package(
  CATKIN_DEPENDS rtt_ros 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DRTT_COMPONENT)

orocos_component(PotentialFielPlanning src/potentialFieldPlanning.cpp)
target_link_libraries(PotentialFielPlanning ${catkin_LIBRARIES})

orocos_generate_package()

install(FILES 
  start.launch 
  start.ops 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(path_visualization_rviz src/path_visualization_rviz.cpp)
target_link_libraries(path_visualization_rviz ${catkin_LIBRARIES})
