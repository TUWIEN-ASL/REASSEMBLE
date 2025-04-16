cmake_minimum_required(VERSION 2.8.3)
project(haptic_ros)

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
  LIBRARIES haptic_ros
  CATKIN_DEPENDS
    eigen_conversions
    roscpp
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

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()



include_directories(
  include ${catkin_INCLUDE_DIRS}
)

find_library(USB1LIB  libusb-1.0.so)

add_library(dhd SHARED IMPORTED)
set_target_properties(dhd PROPERTIES IMPORTED_LOCATION /asl_libs/haptic/lib/libdhd.so.3)

add_executable(HapticDevice src/HapticDevice.cpp src/OneEuroFilter.cpp)
add_dependencies(HapticDevice ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(HapticDevice
  dhd
  ${USB1LIB}
  ${catkin_LIBRARIES}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# add_executable(haptic_ros
#   src/OneEuroFilter.cpp
# )
# add_dependencies(haptic_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(haptic_ros
#   ${catkin_LIBRARIES}
#   ${EIGEN3_INCLUDE_DIRS}
# )