// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_controller_damping_ratio.h>

#include <franka_example_controllers/nullspace_optim.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool CartesianImpedanceControllerDampingRatio::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &CartesianImpedanceControllerDampingRatio::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  sub_vel_d_ = node_handle.subscribe(
      "desired_velocity", 20, &CartesianImpedanceControllerDampingRatio::velDCallback, this);

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceControllerDampingRatio: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceControllerDampingRatio: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerDampingRatio: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerDampingRatio: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerDampingRatio: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerDampingRatio: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerDampingRatio: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceControllerDampingRatio: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceControllerDampingRatio::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  linear_velocity_d_.setZero();
  angular_velocity_d_.setZero();
  linear_velocity_target_.setZero();
  angular_velocity_target_.setZero();

  cartesian_stiffness_.setZero();
  // cartesian_damping_.setZero();

  return true;
}

void CartesianImpedanceControllerDampingRatio::starting(const ros::Time& /*time*/) {
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

void CartesianImpedanceControllerDampingRatio::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 49> mass_array = model_handle_->getMass();

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Matrix<double, 7, 1> tau_friction;
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // ROS_ERROR_STREAM(jacobian.rows() << " " << jacobian.cols());
  // ROS_ERROR_STREAM(mass.rows() << " " << mass.cols());
  Eigen::Matrix<double, 6, 6> Lambda = (jacobian * mass.inverse() * jacobian.transpose()).inverse();
  Eigen::MatrixXd L( Lambda.llt().matrixL() );
  Eigen::MatrixXd L_inv = L.inverse();
  Eigen::MatrixXd K_tilda = L_inv * cartesian_stiffness_ * L_inv.transpose();

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver;
  eigensolver.compute(K_tilda);
  if (eigensolver.info() != Eigen::Success) abort();
  Eigen::MatrixXd eigenvalues = eigensolver.eigenvalues();
  Eigen::MatrixXd eigenvectors = eigensolver.eigenvectors();

  Eigen::MatrixXd H = L * eigenvectors;

  Eigen::Matrix<double, 6, 6> cartesian_damping_ = H * (2 * damping_ratio_ * eigenvalues.array().sqrt().matrix()).asDiagonal() * H.transpose();

  // compute error to desired pose
  // position error
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

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_nullspace_damp(7), tau_nullspace_man(7), tau_nullspace_JL(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  // tau_task << jacobian.transpose() *
  //                 (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * error_vel);

  // nullspace PD control with damping ratio = 1
  tau_nullspace_damp << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) * (-nullspace_damping_ * dq);

  // Compute the gradient of the manipulability measure
  Eigen::VectorXd dW_dq = computeManipulabilityGradient(model_handle_, robot_state);

  // Compute the gradient of the joint limits
  Eigen::VectorXd dJL_dq = computeJointLimitGradient(model_handle_, robot_state);

  // ROS_ERROR_STREAM("gtadient " << dW_dq << std::endl);

  tau_nullspace_man << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (-nullspace_manip_P_ * dW_dq);

  tau_nullspace_JL << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (-nullspace_JL_ * dJL_dq);

  tau_nullspace << (1-nullspace_ratio_) * tau_nullspace_damp + nullspace_ratio_ * tau_nullspace_man + tau_nullspace_JL;


  // ROS_ERROR_STREAM("tau_nullspace " << tau_nullspace << std::endl);

  double m = computeManipulability(jacobian);
  // ROS_ERROR_STREAM("Man " << m << std::endl);

  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  // cartesian_damping_ =
  //     filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  damping_ratio_ =
      filter_params_ * damping_ratio_target_ + (1.0 - filter_params_) * damping_ratio_;
  nullspace_damping_ =
      filter_params_ * nullspace_damping_target_ + (1.0 - filter_params_) * nullspace_damping_;

  nullspace_manip_P_ =
      filter_params_ * nullspace_manip_P_target_ + (1.0 - filter_params_) * nullspace_manip_P_;

  nullspace_JL_ =
      filter_params_ * nullspace_JL_target_ + (1.0 - filter_params_) * nullspace_JL_;

  nullspace_ratio_ =
      filter_params_ * nullspace_ratio_target_ + (1.0 - filter_params_) * nullspace_ratio_;


  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  linear_velocity_d_ = filter_params_ * linear_velocity_target_ + (1.0 - filter_params_) * linear_velocity_d_;
  angular_velocity_d_ = filter_params_ * angular_velocity_target_ + (1.0 - filter_params_) * angular_velocity_d_;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceControllerDampingRatio::saturateTorqueRate(
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

void CartesianImpedanceControllerDampingRatio::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  // cartesian_damping_target_.setIdentity();
  // // Damping ratio = 1
  // cartesian_damping_target_.topLeftCorner(3, 3)
  //     << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  // cartesian_damping_target_.bottomRightCorner(3, 3)
  //     << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  damping_ratio_target_ = config.damping_ratio;
  nullspace_damping_target_ = config.nullspace_damping;

  nullspace_manip_P_target_ = config.manipulability_scale;
  nullspace_ratio_target_ = config.mani_vs_damping;
  nullspace_JL_target_ = config.joint_limit_cont;
}

void CartesianImpedanceControllerDampingRatio::equilibriumPoseCallback(
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

void CartesianImpedanceControllerDampingRatio::velDCallback(
    const geometry_msgs::TwistStampedConstPtr& msg) {
  std::lock_guard<std::mutex> twist_target_mutex_lock(twist_target_mutex_);
  
  // Extract linear velocity
  linear_velocity_target_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
  
  // Extract angular velocity
  angular_velocity_target_ << msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceControllerDampingRatio,
                       controller_interface::ControllerBase)