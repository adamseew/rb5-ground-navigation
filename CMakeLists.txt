
cmake_minimum_required(VERSION 2.8)
project(ytcg_ground-nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  message_generation
  std_msgs
  cv_bridge
)

find_package(Boost COMPONENTS filesystem regex REQUIRED)

find_package(OpenCV REQUIRED)

# define catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs geometry_msgs message_generation std_msgs cv_bridge ${MSG_DEPS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(LIBS
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
)

add_executable(pose_publisher_node src/pose_pub.cpp)
add_executable(pointcloud_depth_wrapper src/pcd_wrapper.cpp)
add_dependencies(pose_publisher_node ${catkin_EXPORTED_TARGETS})
add_dependencies(pointcloud_depth_wrapper ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_publisher_node ${LIBS})
target_link_libraries(pointcloud_depth_wrapper ${LIBS})


