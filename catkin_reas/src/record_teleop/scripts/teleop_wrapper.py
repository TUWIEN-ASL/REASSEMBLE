import rospy
import numpy as np
import pytransform3d.rotations as pr
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped, Vector3, PoseStamped, TransformStamped
from std_msgs.msg import Bool
import copy
import math
import actionlib
import franka_gripper.msg
import tf
import tf2_ros

class GripperWrapper():
    def __init__(self):
        # Get gripper control action
        self._open_client = actionlib.SimpleActionClient("/franka_gripper/move", franka_gripper.msg.MoveAction)
        self._close_client = actionlib.SimpleActionClient("/franka_gripper/grasp", franka_gripper.msg.GraspAction)
        print("wating for gripper action server...")
        self._open_client.wait_for_server()
        self._close_client.wait_for_server()

    def open(self):
        print("oppening")
        goal = franka_gripper.msg.MoveGoal()

        goal.width = 0.08
        goal.speed = 0.5
        self._open_client.send_goal(goal)


    def close(self, force=60):
        print("Closing")
        goal = franka_gripper.msg.GraspGoal()

        goal.width = 0.0
        goal.speed = 0.5
        goal.force = force
        goal.epsilon.inner = 0.2
        goal.epsilon.outer = 0.2
        self._close_client.send_goal(goal)

class TeleopWrapper:
    def __init__(self):
        # Get parameter
        self.use_sim = rospy.get_param('~use_sim', True)  # Default to False if not set
        print(f"use_sim: {self.use_sim}")
        self.use_ft_sensor = rospy.get_param('~use_ft_sensor', False)  # Default to False if not set
        print(f"use_ft_sensor: {self.use_ft_sensor}")
        self.use_gripper = rospy.get_param('~use_gripper', False)  # Default to False if not set
        print(f"use_gripper: {self.use_gripper}")
        self.haptic_z_rotation_deg = rospy.get_param('~haptic_z_rotation_deg', 0.0)
        print(f"haptic_z_rotation_deg: {self.haptic_z_rotation_deg}")
        self.haptic_z_rotation_rad = np.deg2rad(self.haptic_z_rotation_deg)
        self.z_rotation_quaternion = pr.quaternion_from_euler(np.array([0, 0, self.haptic_z_rotation_rad]), 0, 1, 2, extrinsic=0)

        self.pub_pose    = rospy.Publisher('/cartesian_vic_teleop/teleop_pose', PoseStamped, queue_size=1)
        self.pub_force    = rospy.Publisher('/Master/FeedbackForce', WrenchStamped, queue_size=1)
        
        self.robot_init_pose = None
        self.master_init_pose = None

        if not self.use_ft_sensor:
            self.sub_force   = rospy.Subscriber("/franka_state_controller/F_ext_base", WrenchStamped, self.force_cb)
            # sub_force   = rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, force_cb)
        else:
            self.sub_force   = rospy.Subscriber("/ft_sensor/ft_compensated_base", WrenchStamped, self.force_cb)
            # sub_force   = rospy.Subscriber("/ft_sensor/ft_compensated", WrenchStamped, force_cb)

        if self.use_gripper:
            self.gripper = GripperWrapper()
        else:
            self.gripper = None

        self.listener = tf.TransformListener()

    def start(self):
        print("Starting teleop")
        self.listener.waitForTransform("fr3_link0", "fr3_EE", rospy.Time(0), rospy.Duration(0.5))
        robot_init_position, robot_init_orientation_xyzw = self.listener.lookupTransform("fr3_link0", "fr3_EE", rospy.Time())
        robot_init_orientation = pr.quaternion_wxyz_from_xyzw(robot_init_orientation_xyzw)
        print("Robot initial pose", robot_init_position)
        print("Robot initial orientation", robot_init_orientation)
        self.robot_init_pose = np.concatenate([robot_init_position, robot_init_orientation])
        robot_init_position = np.array(robot_init_position)

        static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "fr3_link0"
        static_transformStamped.child_frame_id = "haptic_device"
        static_transformStamped.transform.translation.x = 1.0
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.0
        # quaternion = tf.transformations.quaternion_from_euler(np.pi, -np.pi/2, 0, 'rxyz')
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0, 'rxyz')
        static_transformStamped.transform.rotation.x = quaternion[0]
        static_transformStamped.transform.rotation.y = quaternion[1]
        static_transformStamped.transform.rotation.z = quaternion[2]
        static_transformStamped.transform.rotation.w = quaternion[3]
        static_broadcaster.sendTransform(static_transformStamped)


        print("Creating subsribers")
        self.sub_pose    = rospy.Subscriber("/Master/command/pose", PoseStamped, self.omaga_cb)
        self.sub_button  = rospy.Subscriber("/Master/command/button", Bool, self.button_cb)

    def stop(self):
        self.robot_init_pose = None
        self.master_init_pose = None

        self.sub_pose.unregister()
        self.sub_button.unregister()
        


    def force_cb(self, msg):
        msg_ForceFeedback = copy.deepcopy(msg)    
        msg_ForceFeedback.header.frame_id = "haptic_device"

        # tranform to master device frame
        curr_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        if self.use_sim or self.use_ft_sensor: # for some reason the forces need to be inverted for simulation
            curr_force = -curr_force

        force_transformed = pr.q_prod_vector(self.z_rotation_quaternion, curr_force)
        msg_ForceFeedback.wrench.force.z  = force_transformed[2]
        msg_ForceFeedback.wrench.force.y  = force_transformed[1]
        msg_ForceFeedback.wrench.force.x  = force_transformed[0]

        self.pub_force.publish(msg_ForceFeedback)

    def button_cb(self, msg):
        if self.gripper is None:
            return

        if (msg.data != prev_state and msg.data == True):
            if(toggle):
                self.gripper.open()
            else:
                self.gripper.close(60)
            self.toggle = not self.toggle

        self.prev_state = msg.data

    def omaga_cb(self, msg):#Control integrated
        position_scale = np.array([3, 3, 3])*1.5

        if self.master_init_pose is None:
            self.master_init_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
            print("Master initial pose", self.master_init_pose)

        if self.robot_init_pose is not None and self.master_init_pose is not None and self.pub_pose is not None:
            raw_master_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
            position_diff = raw_master_pose[0:3] - self.master_init_pose[0:3]
            position_diff_rotated = pr.q_prod_vector(self.z_rotation_quaternion, position_diff)
            robot_position = self.robot_init_pose[0:3] + position_diff_rotated * position_scale

            initial_haptic_orientation = pr.concatenate_quaternions(self.z_rotation_quaternion, self.master_init_pose[3:7])
            current_haptic_orientation = pr.concatenate_quaternions(self.z_rotation_quaternion, raw_master_pose[3:7])
            orientation_diff = pr.concatenate_quaternions(current_haptic_orientation, pr.q_conj(initial_haptic_orientation))
            robot_orientation = pr.concatenate_quaternions(orientation_diff, self.robot_init_pose[3:7])

            self.publish_pose(self.pub_pose, "fr3_link0", robot_position, robot_orientation)

    def publish_pose(self, pub, base_frame, position, orientation):
        msg = PoseStamped()
        msg.header.frame_id = base_frame
        msg.header.stamp = rospy.Time.now()

        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]

        msg.pose.orientation.w = orientation[0]
        msg.pose.orientation.x = orientation[1]
        msg.pose.orientation.y = orientation[2]
        msg.pose.orientation.z = orientation[3]

        pub.publish(msg)