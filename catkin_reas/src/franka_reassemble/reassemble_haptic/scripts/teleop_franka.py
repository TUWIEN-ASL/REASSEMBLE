import rospy
from geometry_msgs.msg import PoseStamped
import tf
import tf2_ros
import numpy as np
import scipy
import scipy.signal
import actionlib
import franka_gripper.msg
from geometry_msgs.msg import WrenchStamped, Vector3, PoseStamped, TransformStamped
import pytransform3d.rotations as pr
from std_msgs.msg import Bool
import copy
import math

class LiveFilter:
    """Base class for live filters.
    """
    def process(self, x):
        # do not process NaNs
        if np.isnan(x).any():
            return x

        return self._process(x)

    def __call__(self, x):
        return self.process(x)

    def _process(self, x):
        raise NotImplementedError("Derived class must implement _process")
    
class LiveSosFilter(LiveFilter):
    """Live implementation of digital filter with second-order sections.
    The following is equivalent to calling scipy.sosfilt(sos, xs):
    >>> sosfilter = LiveSosFilter(sos)
    >>> [sosfilter(x) for x in xs]
    """
    def __init__(self, sos):
        """Initialize live second-order sections filter.
        Args:
            sos (array-like): second-order sections obtained from scipy
                filter design (with output="sos").
        """
        self.sos = sos

        self.n_sections = sos.shape[0]
        self.state = np.zeros((self.n_sections, 2, 3))

    def _process(self, x):
        """Filter incoming data with cascaded second-order sections.
        """
        for s in range(self.n_sections):  # apply filter sections in sequence
            b0, b1, b2, a0, a1, a2 = self.sos[s, :]
            
            y = b0*x + self.state[s, 0]
            self.state[s, 0] = b1*x - a1*y + self.state[s, 1]
            self.state[s, 1] = b2*x - a2*y
            x = y  # set biquad output as input of next filter section.

        return y

class GripperWrapper():
    def __init__(self):
        # Get gripper control action
        self._open_client = actionlib.SimpleActionClient("/franka_gripper/move", franka_gripper.msg.MoveAction)
        self._close_client = actionlib.SimpleActionClient("/franka_gripper/grasp", franka_gripper.msg.GraspAction)
        print("wating for action server")
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

# def create_pose(pose):
#     marker_pose = PoseStamped()
#     link_name = "fr3_link0"
#     marker_pose.header.frame_id = link_name
#     marker_pose.header.stamp = rospy.Time(0)

#     marker_pose.pose.position.x = max([min([pose[0],
#                                         position_limits[0][1]]),
#                                         position_limits[0][0]])
#     marker_pose.pose.position.y = max([min([pose[1],
#                                         position_limits[1][1]]),
#                                         position_limits[1][0]])
#     marker_pose.pose.position.z = max([min([pose[2],
#                                         position_limits[2][1]]),
#                                         position_limits[2][0]])
    
#     marker_pose.pose.orientation.w = pose[3]
#     marker_pose.pose.orientation.x = pose[4]
#     marker_pose.pose.orientation.y = pose[5]
#     marker_pose.pose.orientation.z = pose[6]

#     return marker_pose

def publish_wrench(pub, base_frame, force, torque):
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

def publish_pose(pub, base_frame, position, orientation):
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

def omaga_cb(msg):#Control integrated
    position_scale = np.array([3, 3, 3])*2.5           # sensitivity
    global master_init_pose
    global robot_init_pose
    global pub_pose
    global z_rotation_quaternion

    # print("in callback")
    # print(robot_init_pose, master_init_pose, pub_pose)

    if master_init_pose is None:
        master_init_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        print(master_init_pose)

    if robot_init_pose is not None and master_init_pose is not None and pub_pose is not None:
        raw_master_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        position_diff = raw_master_pose[0:3] - master_init_pose[0:3]
        position_diff_rotated = pr.q_prod_vector(z_rotation_quaternion, position_diff)
        robot_position = robot_init_pose[0:3] + position_diff_rotated * position_scale

        initial_haptic_orientation = pr.concatenate_quaternions(z_rotation_quaternion, master_init_pose[3:7])
        current_haptic_orientation = pr.concatenate_quaternions(z_rotation_quaternion, raw_master_pose[3:7])
        orientation_diff = pr.concatenate_quaternions(current_haptic_orientation, pr.q_conj(initial_haptic_orientation))

        # robot_orientation = raw_master_pose[3:8]
        # # robot_orientation = pr.concatenate_quaternions(robot_orientation, pr.quaternion_from_euler(np.array([np.pi,-np.pi/2,0]), 0,1,2, extrinsic=0))
        # robot_orientation = pr.concatenate_quaternions(robot_orientation, pr.quaternion_from_euler(np.array([np.pi, -np.pi/2, 0]), 0, 1, 2, extrinsic=0))

        # # robot_orientation = pr.concatenate_quaternions(robot_init_pose[3:7], orientation_diff)
        # robot_orientation = pr.concatenate_quaternions(orientation_diff, robot_init_pose[3:7])
        # robot_orientation = pr.concatenate_quaternions(z_rotation_quaternion, robot_orientation)
        # # robot_orientation = pr.concatenate_quaternions(robot_orientation, z_rotation_quaternion)


        # orientation_diff_transformed = pr.concatenate_quaternions(z_rotation_quaternion, orientation_diff)
        robot_orientation = pr.concatenate_quaternions(orientation_diff, robot_init_pose[3:7])

        # robot_orientation = robot_init_pose[3:]
        
        # robot_pose = np.concatenate((robot_position, robot_orientation))

        # pose = create_pose(pub_pose, robot_pose)
        # # print("publishing")
        # pub_pose.publish(pose)

        publish_pose(pub_pose, "fr3_link0", robot_position, robot_orientation)
        # publish_pose(pub_pose, "fr3_link0", robot_init_pose[0:3], robot_init_pose[3:7])
    
def button_cb(msg):
    global prev_state
    global gripper
    global toggle
    global z_rotation_quaternion

    if gripper is None:
        return

    if (msg.data != prev_state and msg.data == True):
        if(toggle):
            gripper.open()
        else:
            gripper.close(60)
        toggle = not toggle

    prev_state = msg.data
        

def force_cb(msg):
    global pub_force
    global init_force
    global force_filter
    
    # if init_force is None:
    #     init_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
    # else:
    #     curr_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

    #     force_unbiased = curr_force - init_force

    #     force_filtered = force_filter(force_unbiased)

    #     resp = Vector3()
    #     resp.x = force_filtered[0]
    #     resp.y = force_filtered[1]
    #     resp.z = force_filtered[2]
     
    # v2
    msg_ForceFeedback = copy.deepcopy(msg)    
    msg_ForceFeedback.header.frame_id = "haptic_device"

    # tranform to master device frame
    curr_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
    if use_sim or use_ft_sensor: # for some reason the forces need to be inverted for simulation
        curr_force = -curr_force
    # curr_force = force_filter(curr_force)
    # force_transformed = pr.q_prod_vector(pr.q_conj(pr.quaternion_from_euler(np.array([np.pi,-np.pi/2,0]), 0,1,2, extrinsic=0)), curr_force)
    # force_transformed = curr_force
    force_transformed = pr.q_prod_vector(z_rotation_quaternion, curr_force)
    msg_ForceFeedback.wrench.force.z  = force_transformed[2]
    msg_ForceFeedback.wrench.force.y  = force_transformed[1]
    msg_ForceFeedback.wrench.force.x  = force_transformed[0]

    # # filtering
    # curr_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
    # force_filtered = force_filter(curr_force)
    # msg_ForceFeedback.wrench.force.z  = force_filtered[2] 
    # msg_ForceFeedback.wrench.force.y  = force_filtered[1]
    # msg_ForceFeedback.wrench.force.x  = force_filtered[0]
    
    # # for testing
    # current_time = rospy.Time.now().to_sec()
    # sine_wave_amplitude = 5.0  # Adjust amplitude as needed
    # sine_wave_frequency = 1.0  # Adjust frequency as needed
    # msg_ForceFeedback.wrench.force.z = sine_wave_amplitude * math.sin(2 * math.pi * sine_wave_frequency * current_time)
    # msg_ForceFeedback.wrench.force.y = sine_wave_amplitude * math.cos(2 * math.pi * sine_wave_frequency * current_time)
    # msg_ForceFeedback.wrench.force.x = 0

    # msg_ForceFeedback.wrench.force.z = 0
    # msg_ForceFeedback.wrench.force.y = 0
    # msg_ForceFeedback.wrench.force.x = 0

    pub_force.publish(msg_ForceFeedback)    


def main():
    global robot_init_pose
    global pub_pose
    global pub_force
    global gripper
    global force_filter
    global master_init_pose
    # master_init_pose = None
    global use_sim
    global use_ft_sensor
    global z_rotation_quaternion
    global controller

    rospy.init_node("teleop_franka")

    # Get parameter
    use_sim = rospy.get_param('~use_sim', True)  # Default to False if not set
    print(f"use_sim: {use_sim}")
    use_ft_sensor = rospy.get_param('~use_ft_sensor', False)  # Default to False if not set
    print(f"use_ft_sensor: {use_ft_sensor}")
    use_gripper = rospy.get_param('~use_gripper', False)  # Default to False if not set
    print(f"use_gripper: {use_gripper}")
    haptic_z_rotation_deg = rospy.get_param('~haptic_z_rotation_deg', 0.0)
    print(f"haptic_z_rotation_deg: {haptic_z_rotation_deg}")
    haptic_z_rotation_rad = np.deg2rad(haptic_z_rotation_deg)
    z_rotation_quaternion = pr.quaternion_from_euler(np.array([0, 0, haptic_z_rotation_rad]), 0, 1, 2, extrinsic=0)
    controller = rospy.get_param('~controller', True)  # Default to False if not set
    print(f"controller: {controller}")

    sos = scipy.signal.iirfilter(4, Wn=1, fs=1000, btype="low",
                             ftype="butter", output="sos")
    force_filter = LiveSosFilter(sos)

    # pub_pose    = rospy.Publisher('/cartesian_vic_teleop/teleop_pose', PoseStamped, queue_size=10)
    # pub_pose    = rospy.Publisher('/cartesian_impedance_controller_damping_ratio/equilibrium_pose', PoseStamped, queue_size=10)
    pub_pose    = rospy.Publisher('/' + controller + '/equilibrium_pose', PoseStamped, queue_size=10)
    pub_force    = rospy.Publisher('/Master/FeedbackForce', WrenchStamped, queue_size=10)

    if use_ft_sensor == False:
        sub_force   = rospy.Subscriber("/franka_state_controller/F_ext_base", WrenchStamped, force_cb)
        # sub_force   = rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, force_cb)
    else:
        sub_force   = rospy.Subscriber("/ft_sensor/ft_compensated_base", WrenchStamped, force_cb)
        # sub_force   = rospy.Subscriber("/ft_sensor/ft_compensated", WrenchStamped, force_cb)

    if use_gripper:
        gripper = GripperWrapper()

    listener = tf.TransformListener()
    listener.waitForTransform("fr3_link0", "fr3_EE", rospy.Time(), rospy.Duration(0.5))
    robot_init_position, robot_init_orientation_xyzw = listener.lookupTransform("fr3_link0", "fr3_EE", rospy.Time())
    robot_init_orientation = pr.quaternion_wxyz_from_xyzw(robot_init_orientation_xyzw)
    print(robot_init_position)
    print(robot_init_orientation)
    robot_init_pose = np.concatenate([robot_init_position, robot_init_orientation])
    robot_init_position = np.array(robot_init_position)


    # broadcaster = tf.TransformBroadcaster()
    # broadcaster.sendTransform((1, 0, 0), tf.transformations.quaternion_from_euler(np.pi,-np.pi/2,0, 'rxyz'),
    #                     rospy.Time.now(), "haptic_device", "world")
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

    print("READY")

    sub_pose    = rospy.Subscriber("/Master/command/pose", PoseStamped, omaga_cb)
    sub_button  = rospy.Subscriber("/Master/command/button", Bool, button_cb)

    rospy.spin()

# Global Variables
# Pose
# position_limits = [[-1, 1], [-1, 1], [0.02, 0.8]]
position_limits = [[-1, 1], [-1, 1], [-10.02, 0.8]]
master_init_pose = None
robot_init_pose = None
pub_pose = None

# Force
init_force = None
pub_force = None
force_filter = None

# Gripper
toggle = False
gripper = None
prev_state = 0

# Sim vs Real
use_sim = False

# Controller
controller = 'cartesian_impedance_controller_damping_ratio'

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
