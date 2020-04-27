cmake_minimum_required(VERSION 2.8.3)

project(obstacle_avoidance_potential_field)

find_package(catkin REQUIRED COMPONENTS rtt_ros)

catkin_package(
  CATKIN_DEPENDS rtt_ros 
)

add_definitions(-DRTT_COMPONENT)
# orocos_component(PotentialField src/potentialField.cpp)
# target_link_libraries(PotentialField ${catkin_LIBRARIES})

orocos_component(PotentialFielPlanning src/potentialFieldPlanning.cpp)
target_link_libraries(PotentialFielPlanning ${catkin_LIBRARIES})


orocos_generate_package()

install(FILES 
  start.launch 
  start.ops 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


