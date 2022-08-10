cmake_minimum_required(VERSION 3.5.1)
project(icp_localization)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
    pcl_conversions
    pcl_ros
    sensor_msgs
    std_msgs
    visualization_msgs
    geometry_msgs
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    roslib
    pointmatcher_ros
)

set(SRC_FILES
  src/helpers.cpp
  src/RangeDataAccumulator.cpp
  src/ICPlocalization.cpp
  src/transform/RigidTransform.cpp
  src/transform/Twist.cpp
  src/transform/TwistIntegrationBuffer.cpp
  src/transform/TwistIntegrationBufferRos.cpp
  src/transform/TransformInterpolationBuffer.cpp
  src/transform/TfPublisher.cpp
  src/transform/FrameTracker.cpp
  src/transform/ImuReading.cpp
  src/transform/ImuTracker.cpp
  src/transform/ImuInterpolationBuffer.cpp
  src/common/time.cpp
  src/common/math.cpp
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(libpointmatcher REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    PCL
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
    ${libpointmatcher_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

link_directories( #only needed for pcl
  ${PCL_LIBRARY_DIRS}
)

add_definitions( #only needed for pcl
  ${PCL_DEFINITIONS}
)

add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${LIBPOINTMATCHER_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

add_executable(localizer_node
  src/localizer_node.cpp
)

target_link_libraries(localizer_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${libpointmatcher_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

#############
## Install ##
#############
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#)
#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node point_cloud_stitcher_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install(DIRECTORY
#  launch
#  param
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
