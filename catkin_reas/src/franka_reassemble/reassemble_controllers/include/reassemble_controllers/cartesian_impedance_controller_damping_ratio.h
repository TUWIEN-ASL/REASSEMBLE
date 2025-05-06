// Modified version of the original Franka controller from the repository franka_ros
// Created at Autonomous Systems Lab TU Wien
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <reassemble_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace reassemble_controllers {

class CartesianImpedanceControllerDampingRatio : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double filter_params_{0.005};
  double nullspace_damping_{20.0};
  double nullspace_damping_target_{20.0};
  const double delta_tau_max_{0.5};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;

  double nullspace_manip_P_{2.0};
  double nullspace_manip_P_target_{2.0};

  double nullspace_JL_{0.5};
  double nullspace_JL_target_{0.5};

  double nullspace_ratio_{0.8};
  double nullspace_ratio_target_{0.8};

  double damping_ratio_ = 1;
  double damping_ratio_target_ = 1;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  std::mutex twist_target_mutex_;
  Eigen::Vector3d linear_velocity_target_;
  Eigen::Vector3d angular_velocity_target_;
  Eigen::Vector3d linear_velocity_d_;
  Eigen::Vector3d angular_velocity_d_;

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<reassemble_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(reassemble_controllers::compliance_paramConfig& config,
                               uint32_t level);

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  Eigen::Matrix<double, 7, 1> compensate_friction(Eigen::Matrix<double, 7, 1> dq);

  ros::Subscriber sub_vel_d_;
  void velDCallback(const geometry_msgs::TwistStampedConstPtr& msg);
};

}  // namespace reassemble_controllers
