cmake_minimum_required(VERSION 2.8.3)

project(obstacle_avoidance_potential_field)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  rtt_ros
  rtt_roscomm
  interactive_markers
  visualization_msgs
  tf
 )

catkin_package(
  CATKIN_DEPENDS rtt_ros rtt_roscomm interactive_markers visualization_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(potentialfield
  src/potentialfield.cpp
)


add_definitions(-DRTT_COMPONENT)

orocos_component(PotentialFielPlanning src/potentialFieldPlanning.cpp)
target_link_libraries(PotentialFielPlanning
  ${catkin_LIBRARIES}
  potentialfield
)


orocos_generate_package()


catkin_package(INCLUDE_DIRS include
                LIBRARIES potentialfield)


install(TARGETS potentialfield
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


install(FILES 
  start.launch 
  start.ops 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(path_visualization_rviz src/path_visualization_rviz.cpp)
target_link_libraries(path_visualization_rviz ${catkin_LIBRARIES})
