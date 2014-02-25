cmake_minimum_required(VERSION 2.8.3)
project(receive_xsens)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for the xsens library
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Declare executables
add_executable(receive_xsens src/main.cpp src/console.cpp src/conio.c src/myxda.cpp src/serialkey.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(receive_xsens
  ${catkin_LIBRARIES}
  xsensdeviceapi
  xstypes
)
