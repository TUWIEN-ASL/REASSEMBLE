// Modified code for the REASSEMBLE repository. Original source: 2023 Franka Robotics GmbH.
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <franka_msgs/FrankaState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <tf2_msgs/TFMessage.h>

namespace reassemble_control {

/**
 * Controller to publish the robot state as ROS topics.
 */
class ReassembleStateController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaStateInterface> {
 public:
  /**
   * Creates an instance of a ReassembleStateController.
   */
   ReassembleStateController() = default;

  /**
   * Initializes the controller with interfaces and publishers.
   *
   * @param[in] robot_hardware RobotHW instance to get a franka_hw::FrankaStateInterface from.
   * @param[in] root_node_handle Node handle in the controller_manager namespace.
   * @param[in] controller_node_handle Node handle in the controller namespace.
   */
  bool init(hardware_interface::RobotHW* robot_hardware,
            ros::NodeHandle& root_node_handle,
            ros::NodeHandle& controller_node_handle) override;

  /**
   * Reads the current robot state from the franka_hw::FrankaStateInterface and publishes it.
   *
   * @param[in] time Current ROS time.
   * @param[in] period Time since the last update.
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  void publishFrankaStates(const ros::Time& time);
  void publishJointStates(const ros::Time& time);
  void publishTransforms(const ros::Time& time);
  void publishExternalWrench(const ros::Time& time);
  void publishExternalWrenchBase(const ros::Time& time);

  std::string arm_id_;

  franka_hw::FrankaStateInterface* franka_state_interface_{};
  std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_{};

  realtime_tools::RealtimePublisher<tf2_msgs::TFMessage> publisher_transforms_;
  realtime_tools::RealtimePublisher<franka_msgs::FrankaState> publisher_franka_states_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> publisher_joint_states_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> publisher_joint_states_desired_;
  realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> publisher_external_wrench_;
  realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> publisher_external_wrench_base_;
  franka_hw::TriggerRate trigger_publish_;
  franka::RobotState robot_state_;
  uint64_t sequence_number_ = 0;
  std::vector<std::string> joint_names_;
};

}  // namespace reassemble_control
