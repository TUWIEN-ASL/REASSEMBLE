cmake_minimum_required(VERSION 3.0.2)
project(aidin_ros)

# set(iir_DIR /usr/local/lib/cmake/iir)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  # iir
  # soem
  xmlrpcpp
)

find_package(iir REQUIRED)
find_package(soem REQUIRED)

# find_library(SOEM_LIBRARIES NAMES soem PATHS /usr/local/lib REQUIRED)
# find_path(SOEM_INCLUDE_DIRS "ethercat.h" PATHS /usr/local/include/soem REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${iir_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
  # DEPENDS SOEM
)

message(WARNING ${catkin_INCLUDE_DIRS})
message(WARNING ${SOEM_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # /usr/include/iir
  ${iir_INCLUDE_DIRS}
  # ${SOEM_INCLUDE_DIRS}
  ${soem_INCLUDE_DIRS}
  /asl_libs/EciLinux_amd64/inc
  # /home/dsliwowski/Projects/EciLinux_amd64/inc
)

add_library(${PROJECT_NAME}
  src/aidin_can_wrapper.cpp
  src/can_utils.c
  src/aidin_ec_wrapper.cpp
)

# Set linker flags
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")

# add_executable(${PROJECT_NAME}_node src/aidin_can_ros_node.cpp src/aidin_ec_ros_node.cpp)

add_library(eci113DriverLinux-usb-1.0 SHARED IMPORTED)
set_property(TARGET eci113DriverLinux-usb-1.0 PROPERTY IMPORTED_LOCATION "/asl_libs/EciLinux_amd64/bin/release/libeci113DriverLinux-usb-1.0.so.1.13.3877.0")


# target_link_libraries(${PROJECT_NAME}_node LINK_PUBLIC
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}
#   eci113DriverLinux-usb-1.0
#   ${iir_LIBRARIES}
#   ${SOEM_LIBRARIES}
# )

# For aidin_ec_ros_node
add_executable(aidin_ec_ros_node src/aidin_ec_ros_node.cpp)
target_link_libraries(aidin_ec_ros_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  eci113DriverLinux-usb-1.0
  # ${iir_LIBRARIES}
  iir::iir
  ${soem_LIBRARIES}
  # ${SOEM_LIBRARIES}
  # pthread        # Link POSIX thread library
  # rt             # Link real-time extensions library
  # m              # Math library for any math functions used
  # c
)

# For aidin_can_ros_node
add_executable(aidin_can_ros_node src/aidin_can_ros_node.cpp)
target_link_libraries(aidin_can_ros_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  eci113DriverLinux-usb-1.0
  # ${iir_LIBRARIES}
  iir::iir
  ${soem_LIBRARIES}
  # ${SOEM_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/pub_fake_imu.py
  scripts/force_2_base.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# message(STATUS "SOEM Libraries: ${SOEM_LIBRARIES}")
# message(STATUS "SOEM Include Directories: ${SOEM_INCLUDE_DIRS}")

# find_library(SOEM_FOUND NAMES soem PATHS /usr/local/lib)
# if(SOEM_FOUND)
#   message(STATUS "SOEM library found at ${SOEM_FOUND}")
# else()
#   message(WARNING "SOEM library not found")
# endif()