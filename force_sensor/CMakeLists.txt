cmake_minimum_required(VERSION 2.8.3)
project(force_sensor)
 
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  serial
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
)

# ulimit -c unlimited
add_compile_options(-g -std=c++11)    #-g -> core dumped

add_message_files(
  FILES
  Six_Axis_Force_sensor.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES force_sensor
#  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs nav_msgs roscpp roslib rospy sensor_msgs std_msgs tf
  CATKIN_DEPENDS message_runtime
  
  DEPENDS system_lib
)

# defind robot status massage
# command atkin_make -DCATKIN_WHITELIST_PACKAGES="new_msg"
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(
    include
    include/${PROJECT_NAME}
)

add_executable(M8128_data_acquire_node src/M8128_data_acquire.cpp)
target_link_libraries(M8128_data_acquire_node ${catkin_LIBRARIES})
add_dependencies(M8128_data_acquire_node ${PROJECT_NAME}_gencpp)

# cmake_minimum_required(VERSION 2.4.6)
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# # Set the build type.  Options are:
# #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
# #  Debug          : w/ debug symbols, w/o optimization
# #  Release        : w/o debug symbols, w/ optimization
# #  RelWithDebInfo : w/ debug symbols, w/ optimization
# #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# #set(ROS_BUILD_TYPE RelWithDebInfo)

# rosbuild_init()

# #set the default path for built executables to the "bin" directory
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# #set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# #uncomment if you have defined messages
# #rosbuild_genmsg()
# #uncomment if you have defined services
# #rosbuild_gensrv()

# #common commands for building c++ executables and libraries
# #rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
# #target_link_libraries(${PROJECT_NAME} another_library)
# #rosbuild_add_boost_directories()
# #rosbuild_link_boost(${PROJECT_NAME} thread)
# #rosbuild_add_executable(example examples/example.cpp)
# #target_link_libraries(example ${PROJECT_NAME})
