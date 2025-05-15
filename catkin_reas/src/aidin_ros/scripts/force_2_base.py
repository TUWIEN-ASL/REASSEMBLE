import rospy
from geometry_msgs.msg import WrenchStamped
from franka_msgs.msg import FrankaState
import numpy as np
import tf

class Force2Base:
    def __init__(self):
        
        # self.freq = 1000.0
        
        # TF sensor
        # self.NE_R_Sensor = np.array([[ 0.0000000,  -1.0000000,  0.0000000],
        #                              [ 1.0000000,  0.0000000,  0.0000000],
        #                              [ 0.0000000,  0.0000000,  1.0000000]])
        # self.NE_R_Sensor = np.array([[ 1.0000000,  0.0000000,  0.0000000],
        #                              [ 0.0000000,  1.0000000,  0.0000000],
        #                              [ 0.0000000,  0.0000000,  1.0000000]])
        # self.NE_R_Sensor = np.array([[ 0.7071,  0.7071,  0.0000],
        #                             [ -0.7071,  0.7071,  0.0000],
        #                             [ 0.0000,  0.0000,  1.0000]])
        self.NE_R_Sensor = np.array([[ 0.0000000,  1.0000000,  0.0000000],
                                     [ -1.0000000,  0.0000000,  0.0000000],
                                     [ 0.0000000,  0.0000000,  1.0000000]])
        self.NE_t_Sensor = np.array([[0.0], [0.0], [0.12]])
        self.NE_T_Sensor = np.zeros((4,4))
        self.NE_T_Sensor[:3, :3] = self.NE_R_Sensor
        self.NE_T_Sensor[:3, 3] = self.NE_t_Sensor.T
        self.NE_T_Sensor[3, 3] = 1.0
        # for different tool, change EE poition in desk settings
        # default settings for translation Flange to EE is: [0, 0, 0.325]

        # # For correcting EE offset due to sensor in twist computation
        # self.F_R_EE_default = np.array([[ 0.7071,  0.7071,  0.0000],
        #                                 [ -0.7071,  0.7071,  0.0000],
        #                                 [ 0.0000,  0.0000,  1.0000]])
        # self.F_t_EE_default = np.array([[0.0], [0.0], [0.1034]])
        # self.F_T_EE_default = np.zeros((4,4))
        # self.F_T_EE_default[:3, :3] = self.F_R_EE_default
        # self.F_T_EE_default[:3, 3] = self.F_t_EE_default.T
        # self.F_T_EE_default[3, 3] = 1.0
        # self.F_T_EE_default_inv = np.linalg.inv(self.F_T_EE_default)


        # self.tf_listener.waitForTransform("/fr3_flanch_mount", "/fr3_sensor", rospy.Time(), rospy.Duration(2.0))
        # try:
        #     (trans, rot) = self.tf_listener.lookupTransform("/fr3_flanch_mount", "/fr3_sensor", rospy.Time(0))
        #     print(trans)
        #     print(rot)
        #     trans_matrix = tf.transformations.translation_matrix(trans)
        #     rot_matrix = tf.transformations.quaternion_matrix(rot)
        #     print(trans_matrix)
        #     print(rot_matrix)
        #     self.F_T_Sensor = np.dot(trans_matrix, rot_matrix)
        #     print(self.F_T_Sensor)
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     rospy.logwarn("Unable to get transform from fr3_sensor to fr3_flanch_mount")

        self.O_F_S = np.zeros((6,1))
        self.O_T_Sensor = None 

        ############################################### ROS setup
        rospy.init_node("force_2_base")

        self.sub_ft_wrench = rospy.Subscriber("/ft_sensor/ft_compensated", WrenchStamped, self.wrench_cb, queue_size=1)
        self.sub_robot_state = rospy.Subscriber("/reassemble_state_controller/franka_states", FrankaState, self.state_cb, queue_size=10)
        self.pub_O_F_S = rospy.Publisher("/ft_sensor/ft_compensated_base", WrenchStamped, queue_size=1)

        # self.rate = rospy.Rate(self.freq)

        rospy.sleep(1) 
        rospy.loginfo("Publishing '/ft_sensor/ft_compensated_base'")

    def state_cb(self, state_msg):        
        self.O_T_EE = np.array(state_msg.O_T_EE).reshape(4, 4).T # transpose necessary because array is stored in column-major but numpy reads in row-major (Eigen reads in column-major order)

        # static value
        if not hasattr(self, 'NE_T_EE'):
            self.NE_T_EE = np.array(state_msg.NE_T_EE).reshape(4, 4).T 

        self.O_T_Sensor = self.O_T_EE @ np.linalg.inv(self.NE_T_EE) @ self.NE_T_Sensor
        self.O_R_Sensor = self.O_T_Sensor[:3, :3]
        self.O_t_Sensor = self.O_T_Sensor[:3, 3]

    def wrench_cb(self, msg: WrenchStamped):
        
        self.wrench_sensor = np.array([[msg.wrench.force.x], [msg.wrench.force.y], [msg.wrench.force.z],
                                       [msg.wrench.torque.x], [msg.wrench.torque.y], [msg.wrench.torque.z]])
        if self.O_T_Sensor is not None:
            self.transform_sensor_wrench_to_base()
            self.publish_wrench(self.pub_O_F_S, "fr3_link0", self.O_F_S[:3], self.O_F_S[3:])


    def transform_sensor_wrench_to_base(self):
        F_transformed = np.zeros((6,1))
        F_transformed[:3] = self.O_R_Sensor @ np.reshape(self.wrench_sensor[:3], (3,1))
        F_transformed[3:] = self.O_R_Sensor @ np.reshape(self.wrench_sensor[3:], (3,1)) + np.cross(self.O_t_Sensor.ravel().T, (F_transformed[:3]).ravel().T).reshape((3,1))

        self.O_F_S = F_transformed.copy()

    def publish_wrench(self, pub, base_frame, force, torque):
        msg = WrenchStamped()
        msg.header.frame_id = base_frame
        msg.header.stamp = rospy.Time.now()

        msg.wrench.force.x = force[0]
        msg.wrench.force.y = force[1]
        msg.wrench.force.z = force[2]

        msg.wrench.torque.x = torque[0]
        msg.wrench.torque.y = torque[1]
        msg.wrench.torque.z = torque[2]

        pub.publish(msg)


if __name__ == "__main__":
    f2b = Force2Base()

    while not rospy.is_shutdown():
        rospy.spin()

    #     f2b.publish_wrench(f2b.pub_O_F_S, "fr3_link0", f2b.O_F_S[:3], f2b.O_F_S[3:])
    #     f2b.rate.sleep()