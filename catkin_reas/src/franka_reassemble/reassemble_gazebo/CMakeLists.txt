cmake_minimum_required(VERSION 3.4)
project(reassemble_gazebo)

execute_process(COMMAND uname -m
  OUTPUT_VARIABLE ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  std_msgs
  gazebo_ros_control
  kdl_parser
  boost_sml
  control_toolbox
  controller_manager
  controller_interface
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  eigen_conversions
  urdf
  angles
  franka_hw
  franka_msgs
  franka_gripper
)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

# merge Franka + catkin INCLUDE_DIRS in topological order
list_insert_in_workspace_order(catkin_INCLUDE_DIRS ${Franka_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    gazebo_ros_control
    kdl_parser
    boost_sml
    controller_manager
    controller_interface
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    eigen_conversions
    urdf
    angles
    franka_hw
    franka_msgs
    franka_gripper
    franka_example_controllers
    franka_gazebo
    reassemble_controllers
    reassemble_control
  DEPENDS Franka orocos_kdl
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

# # Modified
# install(PROGRAMS 
#   scripts/sim_time_cleanup.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
  )
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
    )
  add_format_target(reassemble_gazebo FILES ${SOURCES} ${HEADERS})
  add_tidy_target(reassemble_gazebo
    FILES ${SOURCES}
    DEPENDS franka_hw_sim
    )
endif()

# Modified
# Automatically make all scripts in the 'scripts' directory executable
file(GLOB SCRIPTS ${CMAKE_SOURCE_DIR}/scripts/*)
foreach(SCRIPT ${SCRIPTS})
  execute_process(COMMAND chmod +x ${SCRIPT})
endforeach()

# Install rules for all scripts
install(PROGRAMS ${SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)