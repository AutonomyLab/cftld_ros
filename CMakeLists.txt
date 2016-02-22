cmake_minimum_required(VERSION 2.8.3)
project(cftld_ros)

# CMAKE_PREFIX_PATH has higher priority than $ENV{CMAKE_PREFIX_PATH}
# catkin workspace's setup file sets the latter, so messing with
# the former will break catkin

set(CUSTOM_OPENCV_DIR "/home/autolab/opencv/opencv-3.1.0/install")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  cv_bridge
  image_transport
)

# Forcing the custom version of OpenCV
find_package(OpenCV 3 REQUIRED PATHS ${CUSTOM_OPENCV_DIR} NO_DEFAULT_PATH)
message(STATUS "OpenCV Includes: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV Libraries: ${OpenCV_LIBRARIES}")
message(STATUS "OpenCV Libs: ${OpenCV_LIBS}")
message(STATUS "OpenCV Lib dir: ${OpenCV_LIB_DIR}")

# catkin_python_setup()

#Generate messages in the 'msg' folder
add_message_files(
  FILES
  Track.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs std_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cftld_ros
   CATKIN_DEPENDS sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include(ExternalProject)
ExternalProject_Add(CFtld
  #GIT_REPOSITORY https://github.com/AutonomyLab/CFtld.git
  #GIT_TAG master
  URL https://github.com/AutonomyLab/CFtld/archive/master.zip
  #GIT_REPOSITORY /home/autolab/Dev/bebop_ws/external-src/CFtld
  #GIT_TAG dev
  PREFIX ${CATKIN_DEVEL_PREFIX}
  CMAKE_ARGS
    -DWITH_TESTS=0
    -DCMAKE_PREFIX_PATH=${CUSTOM_OPENCV_DIR}
    -DCMAKE_BUILD_TYPE=RelWithDebInfo
    #-DEXTERNAL_DEVEL_SPACE=${CATKIN_DEVEL_PREFIX}
    -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  BUILD_COMMAND make -j4
  # INSTALL_COMMAND make install
  BUILD_IN_SOURCE 1
)

set(CFTLD_PATH "${CATKIN_DEVEL_PREFIX}/src/CFtld")
#set(CFTLD_PATH "/home/autolab/Dev/bebop_ws/external-src/CFtld")

# The underlying source dependency (CFtld) is badly packaged
# and does not provide an easy way to be used as a library.
# I have to manually add include directories to avoid patches.
include_directories(
  include
  ${CFTLD_PATH}/src/opentld
  ${CFTLD_PATH}/src/libopentld
  ${CFTLD_PATH}/src/libopentld/imacq
  ${CFTLD_PATH}/src/libopentld/tld
  ${CFTLD_PATH}/src/3rdparty/cf_tracking/src/cf_libs/common
  ${CFTLD_PATH}/src/3rdparty/libconfig
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${CFTLD_PATH}/lib
#  ${OpenCV_LIB_DIR}
)

## Declare a C++ library
add_library(cftld_benchmarker
  src/benchmarker.cpp
)

add_library(cftld_nodelet_class
  src/cftld_nodelet_class.cpp
)

add_dependencies(cftld_nodelet_class
    ${PROJECT_NAME}_generate_messages_cpp
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    CFtld
)

target_link_libraries(cftld_nodelet_class
  opentld
  config++
  main
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  cftld_benchmarker
)

## Declare a C++ executable
add_executable(cftld_ros_node src/cftld_ros_node.cpp)

add_dependencies(cftld_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cftld_ros_node
  ${catkin_LIBRARIES}
)

add_executable(fake_tracker_node src/fake_tracker.cpp)
add_dependencies(fake_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fake_tracker_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS cftld_ros cftld_ros_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cftld_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
