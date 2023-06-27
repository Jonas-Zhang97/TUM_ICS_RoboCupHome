cmake_minimum_required(VERSION 3.1.3)
project(pick_place)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs

)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    moveit_core
    moveit_ros_planning_interface
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pick_node src/applications/pick_node.cpp)
add_executable(place_node src/applications/place_node.cpp)

add_library(pick src/pick.cpp)
add_library(place src/place.cpp)


target_link_libraries(pick_node pick)
target_link_libraries(place_node place)

target_link_libraries(pick ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(place ${catkin_LIBRARIES} ${Boost_LIBRARIES})