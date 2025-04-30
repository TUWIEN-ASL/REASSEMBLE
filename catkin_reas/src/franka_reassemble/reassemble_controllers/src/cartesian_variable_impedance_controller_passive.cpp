// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_variable_impedance_controller_passive.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool CartesianVariableImpedanceControllerPassive::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {

  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &CartesianVariableImpedanceControllerPassive::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  sub_alpha_h_ = node_handle.subscribe(
      "/alpha_h", 20, &CartesianVariableImpedanceControllerPassive::alphaHCallback, this);
  sub_vel_d_ = node_handle.subscribe(
      "desired_velocity", 20, &CartesianVariableImpedanceControllerPassive::velDCallback, this);
  sub_wrench_d_ = node_handle.subscribe(
      "desired_wrench", 20, &CartesianVariableImpedanceControllerPassive::wrenchDCallback, this);
  sub_wrench_meas_ = node_handle.subscribe(
      "measured_wrench", 20, &CartesianVariableImpedanceControllerPassive::wrenchMeasCallback, this);
  tank_energy_pub_ = node_handle.advertise<std_msgs::Float64>("tank_energy", 1);

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianVariableImpedanceControllerPassive: Could not read parameter arm_id");
    return false;
  }
  // Retrieve impedance parameters from the parameter server
  if (!node_handle.getParam("K_R_trans", K_R_trans)) {
    ROS_ERROR("Failed to get parameter K_R_trans");
    return false;
  }
  if (!node_handle.getParam("K_H_trans", K_H_trans)) {
    ROS_ERROR("Failed to get parameter K_H_trans");
    return false;
  }
  if (!node_handle.getParam("K_R_rot", K_R_rot)) {
    ROS_ERROR("Failed to get parameter K_R_rot");
    return false;
  }
  if (!node_handle.getParam("K_H_rot", K_H_rot)) {
    ROS_ERROR("Failed to get parameter K_H_rot");
    return false;
  }
  if (!node_handle.getParam("filter_params_", filter_params_)) {
    ROS_ERROR("Failed to get parameter filter_params_");
    return false;
  }
  if (!node_handle.getParam("nullspace_stiffness_", nullspace_stiffness_)) {
    ROS_ERROR("Failed to get parameter nullspace_stiffness_");
    return false;
  }
  if (!node_handle.getParam("max_energy_", max_energy_)) {
    ROS_ERROR("Failed to get parameter max_energy_");
    return false;
  }
  if (!node_handle.getParam("min_energy_", min_energy_)) {
    ROS_ERROR("Failed to get parameter min_energy_");
    return false;
  }
  if (!node_handle.getParam("tank_state_", tank_state_)) {
    ROS_ERROR("Failed to get parameter tank_state_");
    return false;
  }
  if (!node_handle.getParam("K_f_", K_f_)) {
    ROS_ERROR("Failed to get parameter K_f_");
    return false;
  }
  if (!node_handle.getParam("K_I_f_", K_I_f_)) {
    ROS_ERROR("Failed to get parameter K_I_f_");
    return false;
  }
  
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianVariableImpedanceControllerPassive: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianVariableImpedanceControllerPassive: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianVariableImpedanceControllerPassive: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianVariableImpedanceControllerPassive: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianVariableImpedanceControllerPassive: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianVariableImpedanceControllerPassive: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianVariableImpedanceControllerPassive: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  linear_velocity_d_.setZero();
  angular_velocity_d_.setZero();
  linear_velocity_target_.setZero();
  angular_velocity_target_.setZero();

  force_d_.setZero();
  torque_d_.setZero();
  force_target_.setZero();
  torque_target_.setZero();

  force_meas_f_.setZero();
  torque_meas_f_.setZero();
  force_meas_.setZero();
  torque_meas_.setZero();

  cartesian_stiffness_H_.setZero();
  cartesian_damping_H_.setZero();
  cartesian_stiffness_H_.topLeftCorner(3, 3)
          << K_H_trans * Eigen::Matrix3d::Identity();
  cartesian_stiffness_H_.bottomRightCorner(3, 3)
          << K_H_rot * Eigen::Matrix3d::Identity();
  cartesian_damping_H_.topLeftCorner(3, 3)
          << 2*sqrt(K_H_trans) * Eigen::Matrix3d::Identity();
  cartesian_damping_H_.bottomRightCorner(3, 3)
          << 2*sqrt(K_H_rot) * Eigen::Matrix3d::Identity();

  K_R = Eigen::MatrixXd::Identity(6, 6);
  K_R.topLeftCorner(3, 3) *= K_R_trans;
  K_R.bottomRightCorner(3, 3) *= K_R_rot;

  K_H = Eigen::MatrixXd::Identity(6, 6);
  K_H.topLeftCorner(3, 3) *= K_H_trans;
  K_H.bottomRightCorner(3, 3) *= K_H_rot;

  K_F = Eigen::MatrixXd::Identity(6, 6)*K_f_;
  // K_I_F = Eigen::MatrixXd::Identity(6, 6)*K_I_f_;
  Eigen::VectorXd diagonal_values(6);
  diagonal_values << K_I_f_*0, K_I_f_*0, K_I_f_*1, K_I_f_*0, K_I_f_*0, K_I_f_*0;  // Replace with your desired values
  K_I_F = diagonal_values.asDiagonal();

  return true;
}

void CartesianVariableImpedanceControllerPassive::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void CartesianVariableImpedanceControllerPassive::update(const ros::Time& /*time*/,
                                                 const ros::Duration& period) {
  // ros::Time current_time = ros::Time::now();
  // ROS_INFO("Current time: %f", current_time.toSec());
  delta_t = period.toSec();

  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // compute error to desired position
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute error to desired velocity
  Eigen::Matrix<double, 6, 1> velocity = (jacobian * dq);
  Eigen::Matrix<double, 6, 1> error_vel;
  error_vel.head(3) << velocity.head(3) - linear_velocity_d_;
  error_vel.tail(3) << velocity.tail(3) - angular_velocity_d_;

  Eigen::Matrix<double, 6, 1> wrench_d;
  wrench_d.head(3) << force_d_;
  wrench_d.tail(3) << torque_d_;

  Eigen::Matrix<double, 6, 1> wrench_m;
  wrench_m.head(3) << force_meas_f_;
  wrench_m.tail(3) << torque_meas_f_;

  // energy-tank based passive control
  Eigen::VectorXd wrench_cmd = updateEnergyTank(error, error_vel, wrench_d, wrench_m, alpha_h);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_FC(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  // tau_task << jacobian.transpose() *
  //                 (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_H_ * error - cartesian_damping_H_ * error_vel);
  tau_FC << jacobian.transpose() * (wrench_cmd);
  // nullspace PD control with damping ratio = 1
  // tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
  //                   jacobian.transpose() * jacobian_transpose_pinv) *
  //                      (nullspace_stiffness_ * (q_d_nullspace_ - q) -
  //                       (2.0 * sqrt(nullspace_stiffness_)) * dq);
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (-(2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis + tau_FC;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  linear_velocity_d_ = filter_params_ * linear_velocity_target_ + (1.0 - filter_params_) * linear_velocity_d_;
  angular_velocity_d_ = filter_params_ * angular_velocity_target_ + (1.0 - filter_params_) * angular_velocity_d_;

  force_d_ = filter_params_ * force_target_ + (1.0 - filter_params_) * force_d_;
  torque_d_ = filter_params_ * torque_target_ + (1.0 - filter_params_) * torque_d_;

  force_meas_f_ = filter_params_ * force_meas_ + (1.0 - filter_params_) * force_meas_f_;
  torque_meas_f_ = filter_params_ * torque_meas_ + (1.0 - filter_params_) * torque_meas_f_;

  // force_d_ = force_target_;
  // torque_d_ = torque_target_;

  // force_meas_f_ = force_meas_;
  // torque_meas_f_ = torque_meas_;


  // ROS_WARN_STREAM_THROTTLE(0.5, "cartesian stiffness:"           << cartesian_stiffness_);
}

Eigen::Matrix<double, 7, 1> CartesianVariableImpedanceControllerPassive::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianVariableImpedanceControllerPassive::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

void CartesianVariableImpedanceControllerPassive::alphaHCallback(const std_msgs::Float64::ConstPtr& msg) {
  alpha_h = msg->data;

  // ROS_ERROR_STREAM("alpha_h: " << alpha_h);

  // K_trans = (1-alpha_h)*K_R_trans + alpha_h*K_H_trans;
  // D_trans = 2*sqrt(K_trans);
  // K_rot = (1-alpha_h)*K_R_rot + alpha_h*K_H_rot;
  // D_rot = 2*sqrt(K_rot);

  // For passivity-based control
  // K_trans = K_H_trans;
  // D_trans = 2*sqrt(K_trans);
  // K_rot = K_H_rot;
  // D_rot = 2*sqrt(K_rot);

  // // For testing
  // K_trans = K_R_trans;
  // D_trans = 2*sqrt(K_trans);
  // K_rot = K_R_rot;
  // D_rot = 2*sqrt(K_rot);

  
  // cartesian_stiffness_.topLeftCorner(3, 3)
  //         << K_trans * Eigen::Matrix3d::Identity();

  // cartesian_stiffness_.bottomRightCorner(3, 3)
  //         << K_rot * Eigen::Matrix3d::Identity();

  // cartesian_damping_.topLeftCorner(3, 3)
  //         << D_trans * Eigen::Matrix3d::Identity();
  // cartesian_damping_.bottomRightCorner(3, 3)
  //         << D_rot * Eigen::Matrix3d::Identity();
}

void CartesianVariableImpedanceControllerPassive::velDCallback(
    const geometry_msgs::TwistStampedConstPtr& msg) {
  std::lock_guard<std::mutex> twist_target_mutex_lock(twist_target_mutex_);
  
  // Extract linear velocity
  linear_velocity_target_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
  
  // Extract angular velocity
  angular_velocity_target_ << msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
}

void CartesianVariableImpedanceControllerPassive::wrenchDCallback(
    const geometry_msgs::WrenchStampedConstPtr& msg) {
  std::lock_guard<std::mutex> wrench_target_mutex_lock(wrench_target_mutex_);
  
  // Extract force
  force_target_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
  
  // Extract torque
  torque_target_ << msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

  // ROS_ERROR_STREAM("force_target_: " << force_target_);
  // ROS_ERROR_STREAM("torque_target_: " << torque_target_);
}

void CartesianVariableImpedanceControllerPassive::wrenchMeasCallback(
    const geometry_msgs::WrenchStampedConstPtr& msg) {
  std::lock_guard<std::mutex> wrench_meas_mutex_lock(wrench_meas_mutex_);
  
  // Extract force
  force_meas_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
  
  // Extract torque
  torque_meas_ << msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

  // ROS_ERROR_STREAM("force_meas_: " << force_meas_);
  // ROS_ERROR_STREAM("torque_meas_: " << torque_meas_);
}

Eigen::VectorXd CartesianVariableImpedanceControllerPassive::updateEnergyTank(
      const Eigen::VectorXd& pose_err,
      const Eigen::VectorXd& twist_err,
      const Eigen::VectorXd& wrench_des,
      const Eigen::VectorXd& wrench_meas,
      double alpha) {
    
    Eigen::MatrixXd K_eff = K_R + alpha * (K_H - K_R);
    Eigen::MatrixXd K_add = K_eff - K_H;    
    Eigen::MatrixXd D_eff = 2 * K_eff.array().sqrt().matrix();
    Eigen::MatrixXd D_H   = 2 * K_H.array().sqrt().matrix();
    Eigen::MatrixXd D_add = D_eff - D_H;

    Eigen::MatrixXd K_F_alpha = (1-alpha)*K_F;

    // Update the integral of the wrench error
    // Anti-windup condition
    double errror_integral_limit = 50.0;
    if (wrench_error_integral.norm() <= errror_integral_limit) {
        wrench_error_integral = (1-alpha)*(wrench_error_integral) + (wrench_meas - wrench_des) * delta_t;
    }
    // wrench_error_integral = (1-alpha)*(wrench_error_integral) + (wrench_meas - wrench_des) * delta_t;

    // Compute the integral force control term
    Eigen::MatrixXd K_I_alpha = (1-alpha) * K_I_F;  // Assuming you have defined K_I as the integral gain matrix
    // ROS_ERROR_STREAM("wrench_error_integral: " << wrench_error_integral.norm());
    Eigen::VectorXd integral_term = K_I_alpha*wrench_error_integral;

    double delta_tank_state = gamma_ / tank_state_ * (
                                                        (twist_err.transpose()*D_H*twist_err)(0,0)*1 //40.0
                                                        - (phi_*twist_err.transpose()*wrench_des)(0,0)
                                                      )
                              + zeta_ / tank_state_ * (
                                                        (-twist_err.transpose()*K_add*pose_err)(0,0)
                                                        - (twist_err.transpose()*D_add*twist_err)(0,0)
                                                        + (1-phi_)*(twist_err.transpose()*wrench_des)(0,0)
                                                        + (twist_err.transpose()*K_F_alpha*(wrench_meas-wrench_des))(0,0)
                                                        + (twist_err.transpose()*integral_term)(0,0)
                                                      );

    tank_state_ += delta_tank_state * delta_t;
    double tank_energy = 0.5 * tank_state_ * tank_state_;

    std_msgs::Float64 tank_energy_msg;
    tank_energy_msg.data = tank_energy; // Assuming tank_energy is the variable you want to publish
    tank_energy_pub_.publish(tank_energy_msg);

    gamma_ = (tank_energy <= max_energy_) ? 1.0 : 0.0;
    zeta_ = (tank_energy >= min_energy_) ? 1.0 : 0.0;
    // zeta_ = 0.0;
    phi_ = (twist_err.transpose() * wrench_des)(0, 0) < 0 ? 1.0 : 0.0;

    // ROS_ERROR_STREAM("tank_energy: " << tank_energy << ", gamma_: " << gamma_ << ", zeta_: " << zeta_ << ", phi_: " << phi_ << ", wrench error:" << (wrench_des - wrench_meas).norm());

    Eigen::VectorXd wrench_command = zeta_*(-K_add*pose_err - D_add*twist_err - (1-phi_)*wrench_des + K_F_alpha*(wrench_meas-wrench_des) + integral_term) - phi_*wrench_des ;
    // Eigen::VectorXd wrench_command = zeta_*(-K_add*pose_err - D_add*twist_err);
    return wrench_command;
  }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVariableImpedanceControllerPassive,
                       controller_interface::ControllerBase)
