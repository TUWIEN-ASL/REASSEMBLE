#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from franka_msgs.msg import FrankaState
from tf.transformations import quaternion_from_matrix
import roboticstoolbox as rtb

"""
Use this script to publish franka states vairables as geometry messages.
"""

def get_tf_mat(i, dh):
    a = dh[i][0]
    d = dh[i][1]
    alpha = dh[i][2]
    theta = dh[i][3]
    q = theta

    return np.array([[np.cos(q), -np.sin(q), 0, a],
                    [np.sin(q) * np.cos(alpha), np.cos(q) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
                    [np.sin(q) * np.sin(alpha), np.cos(q) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
                    [0, 0, 0, 1]])


def get_jacobian(joint_angles):
    dh_params = np.array([[0, 0.333, 0, joint_angles[0]],
                [0, 0, -np.pi / 2, joint_angles[1]],
                [0, 0.316, np.pi / 2, joint_angles[2]],
                [0.0825, 0, np.pi / 2, joint_angles[3]],
                [-0.0825, 0.384, -np.pi / 2, joint_angles[4]],
                [0, 0, np.pi / 2, joint_angles[5]],
                [0.088, 0, np.pi / 2, joint_angles[6]],
                [0, 0.107, 0, 0],
                [0, 0, 0, -np.pi / 4],
                [0.0, 0.169, 0, 0]], dtype=np.float64)

    T_EE = np.identity(4)
    for i in range(7 + 3):
        T_EE = T_EE @ get_tf_mat(i, dh_params)

    J = np.zeros((6, 10))
    T = np.identity(4)
    for i in range(7 + 3):
        T = T @ get_tf_mat(i, dh_params)

        p = T_EE[:3, 3] - T[:3, 3]
        z = T[:3, 2]

        J[:3, i] = np.cross(z, p)
        J[3:, i] = z

    return J[:, :7]

class FrankaStatesConverter:
    def __init__(self):
        self.robot = rtb.models.DH.Panda()
        # self.pub_eeff        = rospy.Publisher("/franka_state_controller/O_T_EE", PoseStamped, queue_size=1)
        self.pub_ee_vel        = rospy.Publisher("/franka_state_controller/O_T_EE_vel", TwistStamped, queue_size=1)

        self.sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.convert_to_geometry_msg, queue_size=1, tcp_nodelay=True)

    @staticmethod
    def q_from_R(R):
        """ generates quaternion from 3x3 rotation matrix """
        _R = np.eye(4)
        _R[:3, :3] = R
        return quaternion_from_matrix(_R)

    def convert_to_geometry_msg(self, state_msg):
        """ publishes franka states as geometry msgs """
        
        
        # Tip of finger gripper
        # O_T_EE = np.array(state_msg.O_T_EE).reshape(4, 4).T
        # quat_ee = self.q_from_R(O_T_EE[:3, :3])        

        # msg_o_t_ee = PoseStamped()
        # msg_o_t_ee.header.stamp = state_msg.header.stamp
        # msg_o_t_ee.header.frame_id = "fr3_link0"
        # msg_o_t_ee.pose.position.x = O_T_EE[0, 3]
        # msg_o_t_ee.pose.position.y = O_T_EE[1, 3]
        # msg_o_t_ee.pose.position.z = O_T_EE[2, 3]
        # msg_o_t_ee.pose.orientation.x = quat_ee[0]
        # msg_o_t_ee.pose.orientation.y = quat_ee[1]
        # msg_o_t_ee.pose.orientation.z = quat_ee[2]
        # msg_o_t_ee.pose.orientation.w = quat_ee[3]

        jacob = self.robot.jacob0(state_msg.q)
        ee_twist = jacob @ np.array(state_msg.dq)[..., np.newaxis]

        msg_o_t_ee_vel = TwistStamped()
        msg_o_t_ee_vel.header.stamp = state_msg.header.stamp
        msg_o_t_ee_vel.header.frame_id = "fr3_link0"
        msg_o_t_ee_vel.twist.linear.x = ee_twist[0]
        msg_o_t_ee_vel.twist.linear.y = ee_twist[1]
        msg_o_t_ee_vel.twist.linear.z = ee_twist[2]
        msg_o_t_ee_vel.twist.angular.x = ee_twist[3]
        msg_o_t_ee_vel.twist.angular.y = ee_twist[4]
        msg_o_t_ee_vel.twist.angular.z = ee_twist[5]

        # self.pub_eeff.publish(msg_o_t_ee)
        self.pub_ee_vel.publish(msg_o_t_ee_vel)

        



if __name__ == '__main__':
    try:
        rospy.init_node("franka_states_converter")
        FrankaStatesConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass