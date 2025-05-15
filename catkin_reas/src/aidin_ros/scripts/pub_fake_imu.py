import rospy
import tf
import tf.transformations
from sensor_msgs.msg import Imu
import numpy as np

if __name__ == "__main__":
    rospy.init_node("fake_ftsensor_imu")
    # rospy.sleep(1)
    # listener = tf.TransformListener()
    # rospy.sleep(1)
    
    arm_id = rospy.get_param("~arm_id")

    pub = rospy.Publisher("/imu/data", Imu, queue_size=1)

    gravity_vec_base = np.array([[0, 0, 9.80665]]).T

    rate = rospy.Rate(2000)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        # listener.waitForTransform("fr3_sensor", "fr3_link0", now, rospy.Duration(0.1))
        # _, rot = listener.lookupTransform("fr3_sensor", "fr3_link0", now)

        # rot_mtx = np.array(tf.transformations.quaternion_matrix(rot))[:3,:3]

        # gravity_vec_sensor = rot_mtx @ gravity_vec_base

        # print(gravity_vec_sensor)

        msg = Imu()
        msg.header.stamp = now
        # msg.header.frame_id = "panda_link0"
        msg.header.frame_id = arm_id + "_link0"
        # msg.header.frame_id = "fr3_link0"

        msg.linear_acceleration.x = gravity_vec_base[0, 0]
        msg.linear_acceleration.y = gravity_vec_base[1, 0]
        msg.linear_acceleration.z = gravity_vec_base[2, 0]

        pub.publish(msg)
        rate.sleep()