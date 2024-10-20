cmake_minimum_required(VERSION 3.0.2)
project(pkg_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(publisher src/publisher.cpp)

target_link_libraries(publisher 
      ${catkin_LIBRARIES})

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
