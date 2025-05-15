cmake_minimum_required(VERSION 2.8.3)
project(reassemble_haptic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  eigen_conversions
  rospy
  std_msgs
  tf
)
find_package(Eigen3 REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES haptic_ros
  CATKIN_DEPENDS
    eigen_conversions
    roscpp
    franka_gripper
    haptic_ros_driver
)

catkin_install_python(
  PROGRAMS
  scripts/teleop_franka.py
  scripts/saveBag2NPZ.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

find_library(USB1LIB  libusb-1.0.so)

add_library(dhd SHARED IMPORTED)
set_target_properties(dhd PROPERTIES IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/haptic_ros/lib/libdhd.so")

add_executable(HapticDevice src/HapticDevice.cpp src/OneEuroFilter.cpp)
add_dependencies(HapticDevice ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(HapticDevice
  dhd
  ${USB1LIB}
  ${catkin_LIBRARIES}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
