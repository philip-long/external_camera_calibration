cmake_minimum_required(VERSION 2.8.3)
project(depth_camera_pose_calibration)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometric_shapes
  kdl_parser
  pcl_ros
  roscpp
  rospy
)



catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES camera_calibration
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
