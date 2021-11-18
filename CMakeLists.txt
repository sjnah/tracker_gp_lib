cmake_minimum_required(VERSION 2.8.3)
project(tracker_gp_lib)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    pcl_ros
    roscpp
    sensor_msgs
    costmap_converter
    visualization_msgs
)

## Declare a catkin package
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include )

## Declare a C++ library
add_library(${PROJECT_NAME} 
  src/tracker.cpp 
  src/Matern32model.cpp 
  src/InfiniteHorizonGP.cpp 
)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries (${PROJECT_NAME} 
    ${catkin_LIBRARIES} )