#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <exception>
 
// Include Eigen library headers
#include <Eigen/Dense>
 
// Include libfranka headers
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/exception.h>
#include <franka/duration.h>
#include <franka/control_types.h>
 
const double delta = 1e-5; // Finite difference step size
 
/**
* @brief Computes the robot's Jacobian at the given joint angles q using libfranka.
* 
* @param model The libfranka Model instance.
* @param q The joint angles (Eigen::VectorXd of size 7).
* @return Eigen::MatrixXd The Jacobian matrix (6x7).
*/
Eigen::MatrixXd computeJacobian(std::unique_ptr<franka_hw::FrankaModelHandle>& model_handle_, const franka::RobotState& rs) {
    std::array<double, 7> q = rs.q;
    const std::array<double, 16> F_T_EE = rs.F_T_EE;
    const std::array<double, 16> EE_T_K = rs.EE_T_K;
    // Compute the Jacobian using libfranka
    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector, q, F_T_EE, EE_T_K);
 
    // Convert std::array to Eigen::MatrixXd
    Eigen::MatrixXd J(6, 7);
    for (size_t i = 0; i < jacobian_array.size(); ++i) {
        J(i / 7, i % 7) = jacobian_array[i];
    }
 
    return J;
}
 
/**
* @brief Computes the manipulability measure W = det(J * J^T).
* 
* @param J The Jacobian matrix (6x7).
* @return double The manipulability measure.
*/
double computeManipulability(const Eigen::MatrixXd& J) {
    Eigen::MatrixXd JJt = J * J.transpose();
    return JJt.determinant();
}
 
/**
* @brief Computes the numerical gradient of W with respect to q using finite differences.
* 
* @param model The libfranka Model instance.
* @param q The joint angles (Eigen::VectorXd of size 7).
* @return Eigen::VectorXd The gradient vector (7x1).
*/
Eigen::VectorXd computeManipulabilityGradient(std::unique_ptr<franka_hw::FrankaModelHandle>& model_handle_, const franka::RobotState& rs) {
    std::array<double, 7> q = rs.q;
    int n = 7;

    Eigen::VectorXd gradient(n);
    Eigen::MatrixXd J = computeJacobian(model_handle_, rs);
    double W = computeManipulability(J);
 
    for (int i = 0; i < n; ++i) {
        std::array<double, 7> q_delta = q;
        q_delta[i] += delta;

        franka::RobotState new_rs = rs;
        new_rs.q = q_delta;
 
        Eigen::MatrixXd J_delta = computeJacobian(model_handle_, new_rs);
        double W_delta = computeManipulability(J_delta);
 
        gradient(i) = (W_delta - W) / delta;
    }
 
    return gradient;
}

double computeLimitFun(double x, double a, double b, double c){
    return std::tanh(a * (x - c) + b) + std::tanh(-a * (x - c) + b) + 2;
}

Eigen::VectorXd computeJointLimitGradient(std::unique_ptr<franka_hw::FrankaModelHandle>& model_handle_, const franka::RobotState& rs) {
    std::array<double, 7> q = rs.q;
    int n = 7;

    Eigen::VectorXd gradient(n);
    double coef[7][3] = {
        {10, -23, 0},
        {10, -15, 0},
        {10, -25, 0},
        {9, -10, -1.6},
        {4, -10, 0},
        {6.1, -10, 2.5},
        {3.7, -10, 0}
    };

    for (int i = 0; i < n; ++i) {
        double W = computeLimitFun(q[i], coef[i][0], coef[i][1], coef[i][2]);
        double W_delta = computeLimitFun(q[i]+delta, coef[i][0], coef[i][1], coef[i][2]);

        gradient(i) = (W_delta - W) / delta;
    }

    return gradient;
}
 
// int main() {
//     try {
//         // Connect to the robot
//         franka::Robot robot("172.16.0.2"); // Replace with your robot's IP address
//         franka::Model model = robot.loadModel();
 
//         // Set the collision behavior (adjust as necessary)
//         robot.setCollisionBehavior(
//             {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//             {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//             {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//             {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}
//         );
 
//         // Define the control loop
//         robot.control([&](const franka::RobotState& state, franka::Duration) -> franka::Torques {
//             // Get current joint positions
//             std::array<double, 7> q_array = state.q;
//             Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd>(q_array.data(), q_array.size());
 
//             // Compute the Jacobian
//             Eigen::MatrixXd J = computeJacobian(model, q);
 
//             // Compute the gradient of the manipulability measure
//             Eigen::VectorXd dW_dq = computeManipulabilityGradient(model, q);
 
//             // Compute the null-space projection matrix
//             Eigen::MatrixXd N = computeNullSpaceProjection(J);
 
//             // Compute the null-space torque to maximize manipulability
//             Eigen::VectorXd tau_null = N * dW_dq;
 
//             // Define your desired joint torques for the primary task (e.g., zero or from another controller)
//             Eigen::VectorXd tau_d(7);
//             tau_d.setZero(); // Replace with your primary task torques
 
//             // Combine primary task torques with null-space torques
//             Eigen::VectorXd tau_command = tau_d + tau_null;
 
//             // Convert Eigen::VectorXd to std::array<double, 7>
//             std::array<double, 7> tau_command_array;
//             Eigen::VectorXd::Map(&tau_command_array[0], tau_command.size()) = tau_command;
 
//             return tau_command_array;
//         });
 
//     } catch (const franka::Exception& ex) {
//         // Handle exceptions
//         std::cerr << ex.what() << std::endl;
//         return -1;
//     }
 
//     return 0;
// }