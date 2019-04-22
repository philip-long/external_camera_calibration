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

add_compile_options(-std=c++11)
find_package(PCL 1.7 REQUIRED COMPONENTS common io)

catkin_package(
INCLUDE_DIRS include
   LIBRARIES depth_camera_pose_calibration
   CATKIN_DEPENDS eigen_conversions roscpp geometric_shapes
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

add_library(depth_camera_pose_calibration
            src/depth_camera_pose_calibration.cpp
            include/${PROJECT_NAME}/depth_camera_pose_calibration.h)
target_link_libraries(depth_camera_pose_calibration ${catkin_LIBRARIES})


add_executable(depth_camera_pose_calibration_node
                      src/depth_camera_pose_calibration.cpp)

target_link_libraries(depth_camera_pose_calibration_node ${catkin_LIBRARIES}
                      depth_camera_pose_calibration)
