cmake_minimum_required(VERSION 2.8.3)
project(lidar)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS message_runtime
   )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rplidar_ros
  sensor_msgs
  std_msgs
  pcl_ros
  genmsg
  message_generation

)

find_package(PCL REQUIRED COMPONENTS common io visualization filters)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen 3 QUIET)

add_message_files(
  FILES
  avoid.msg
)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable (obstacleAvoidance src/obstacleAvoidance.cpp)
add_dependencies(obstacleAvoidance ${catkin_EXPORTED_TARGETS})
add_dependencies(obstacleAvoidance ${PROJECT_NAME}_gencpp)
target_link_libraries(obstacleAvoidance ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PCL_FILTERS_LIBRARIES})