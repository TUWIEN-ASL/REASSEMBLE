cmake_minimum_required(VERSION 3.4)
project(reassemble_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  joint_limits_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  urdf
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

find_package(catkin REQUIRED COMPONENTS
franka_example_controllers
)

# merge Franka + INCLUDE_DIRS in topological order
list_insert_in_workspace_order(catkin_INCLUDE_DIRS ${Franka_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add_message_files(FILES
#   JointTorqueComparison.msg
# )
#
# generate_messages()

generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
  cfg/desired_mass_param.cfg
  cfg/dual_arm_compliance_param.cfg
  cfg/teleop_param.cfg
  cfg/teleop_gripper_param.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES reassemble_controllers
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    hardware_interface
    joint_limits_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    urdf
    visualization_msgs
  DEPENDS Franka
)

add_library(reassemble_controllers
  src/cartesian_impedance_controller_damping_ratio.cpp
  src/cartesian_impedance_example_controller.cpp
  # src/cartesian_variable_impedance_controller_passive.cpp
  # src/cartesian_velocity_example_controller.cpp
  # src/cartesian_VIC_teleop.cpp

  # src/elbow_example_controller.cpp
  # src/cartesian_pose_example_controller.cpp
  # src/cartesian_velocity_example_controller.cpp
  # src/joint_position_example_controller.cpp
  # src/joint_velocity_example_controller.cpp
  # src/model_example_controller.cpp
  # src/joint_impedance_example_controller.cpp
  # src/cartesian_impedance_example_controller.cpp
  # src/cartesian_impedance_controller_damping_ratio.cpp
  # src/cartesian_variable_impedance_controller_damping_ratio.cpp
  # src/cartesian_variable_impedance_controller.cpp
  # src/cartesian_variable_impedance_controller_passive.cpp
  # src/cartesian_variable_impedance_controller_passive_multidim.cpp
  # src/cartesian_variable_impedance_controller_passive_dampingratio_multidim.cpp
  # src/force_example_controller.cpp
  # src/dual_arm_cartesian_impedance_example_controller.cpp
  # src/teleop_joint_pd_example_controller.cpp
  # src/joint_wall.cpp
  # src/cartesian_VIC_teleop.cpp
)

add_dependencies(reassemble_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  # ${PROJECT_NAME}_generate_messages_cpp
  # ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(reassemble_controllers PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

include_directories(SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
include_directories(PUBLIC
  include
)
catkin_package(
  CATKIN_DEPENDS franka_example_controllers
  DEPENDS dynamic_reconfigure
)

target_include_directories(reassemble_controllers SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(reassemble_controllers PUBLIC
  include
)

add_executable(reas_teleop_gripper_node
  src/teleop_gripper_node.cpp
)
target_include_directories(reas_teleop_gripper_node PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(reas_teleop_gripper_node PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(reas_teleop_gripper_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  # ${PROJECT_NAME}_generate_messages_cpp
  # ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

## Installation
install(TARGETS reassemble_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES reassemble_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# catkin_install_python(
#   PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py scripts/dual_arm_interactive_marker.py scripts/recordMotion.py scripts/move_to_traj.py #Modified
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(reassemble_controllers FILES ${SOURCES} ${HEADERS})
  add_tidy_target(reassemble_controllers
    FILES ${SOURCES}
    DEPENDS reassemble_controllers
  )
endif()

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/PepTools.cmake OPTIONAL
  RESULT_VARIABLE PEP_TOOLS
)
if(PEP_TOOLS)
  file(GLOB_RECURSE PYSOURCES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
  add_pyformat_target(franka_control FILES ${PYSOURCES})
endif()
