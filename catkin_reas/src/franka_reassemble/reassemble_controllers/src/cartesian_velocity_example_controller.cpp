// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_velocity_example_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <iostream>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool CartesianVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  linear_d_ = {0.0, 0.0, 0.0};
  angular_d_ = {0.0, 0.0, 0.0};

  sub_twist_ = node_handle.subscribe(
    "commendedTwist", 20, &CartesianVelocityExampleController::twistCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  return true;
}

void CartesianVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

void CartesianVelocityExampleController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  double v_x = linear_d_[0];
  double v_y = linear_d_[1];
  double v_z = linear_d_[2];
  double w_x = angular_d_[0];
  double w_y = angular_d_[1];
  double w_z = angular_d_[2];
  std::array<double, 6> command = {{v_x, v_y, v_z, w_x, w_y, w_z}};

  // std::cout << command[0] << " " << command[1] << " " << command[2] << " " << command[3] << " " << command[4] << " " << command[5] << std::endl;

  velocity_cartesian_handle_->setCommand(command);

  std::lock_guard<std::mutex> twist_target_mutex_lock(
        twist_target_mutex_lock_);
    linear_d_ = filter_params_ * linear_d_target_ + (1.0 - filter_params_) * linear_d_;
    angular_d_ = filter_params_ * angular_d_target_ + (1.0 - filter_params_) * angular_d_;
}

void CartesianVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void CartesianVelocityExampleController::twistCallback(
  const geometry_msgs::TwistStampedConstPtr& msg) {
    std::lock_guard<std::mutex> twist_target_mutex_lock(
      twist_target_mutex_lock_);
  this->linear_d_target_ = {msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};
  this->angular_d_target_ = {msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z};
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerBase)
