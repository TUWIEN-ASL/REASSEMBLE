import rospy
from std_msgs.msg import Int32, Bool, String, Float64
from record_teleop.msg import RecSegmentStamped
# from teleop_wrapper import TeleopWrapper
# from vr_teleop.msg import Buttons
import subprocess
import os
import sys  # getKey stuff
from select import select
import yaml 
from datetime import datetime
from pathlib import Path
import tf
import json

if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

class RecordManager:
    def __init__(self):
        # Recording states
        self.is_recording = False
        self.segment_num = 0
        self.segment_success = {}
        self.recording_process = None
        self.timestamp = None

        rospy.init_node("record_manager", anonymous=True)

        self.base_path = rospy.get_param("~base_path")

        # Timer for publishing flags
        self.rate_hz = 60
        self.rate = rospy.Rate(self.rate_hz)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.publish_flags)  # Timer callback at self.rate Hz

        # Pubs
        self.pub_record_flag = rospy.Publisher("/record_manager/record_flag", Bool, queue_size=1)
        self.pub_segment_num = rospy.Publisher("/record_manager/segment_num", RecSegmentStamped, queue_size=1)
        self.pub_audio_command = rospy.Publisher("audio_commands", String, queue_size=1)
        self.pub_curr_timestamp = rospy.Publisher("curr_timestamp", String, queue_size=1)

    def get_pose(self, frame, now):
        self.listener.waitForTransform('/Robot_base', frame, now, rospy.Duration(1))
        trans, rot = self.listener.lookupTransform('/Robot_base', frame, now)

        return trans, rot


    def calibrate_poses(self):
        rospy.loginfo("Calibrating scene. Turn On mocap illumination.")
        input("Press ENTER once illumination is turned on")
        print("callibrating...")
        command = ["roslaunch", "natnet_ros_cpp", "natnet_ros.launch"]
        netnet_pid = subprocess.Popen(command)
        self.listener = tf.TransformListener()

        rospy.sleep(10)

        now = rospy.Time.now()
       
        hama1_trans, hama1_rot = self.get_pose("Hama1", now)
        hama2_trans, hama2_rot = self.get_pose("Hama2", now)
        dvis_trans,  dvis_rot  = self.get_pose("DAVIS346", now)
        board_trans, board_rot = self.get_pose("NIST_Board1", now)

        netnet_pid.terminate()
        netnet_pid.wait()

        rospy.loginfo("Calibration finished. Turn Off mocap illumination.")
        input("Press ENTER once illumination is turned off")

        return {
            "Hama1": [hama1_trans, hama1_rot],
            "Hama2": [hama2_trans, hama2_rot],
            "DAVIS346": [dvis_trans, dvis_rot],
            "NIST_Board1": [board_trans, board_rot],
        }
    

    def start_recording(self):
        if not self.is_recording:
            
            # poses = self.calibrate_poses()
            rospy.loginfo("Starting recording...")
            # Get params
            topic = rospy.get_param("~topic")
            topic = topic.split()
            path_save = os.path.join(self.base_path, "teleop_bags")
            os.makedirs(path_save, exist_ok=True)
            file_name = self.get_timestamp()

            path = os.path.join(path_save, file_name)
            path_poses = path + "_poses.json"

            # with open(path_poses, "w") as f:
            #     json.dump(poses, f)

            # Start the recording node
            command = ["rosbag", "record",
                        "--output-name", path]
            command.extend(topic)
            rospy.logwarn("Launching roslaunch with args: {}".format(command))
            rospy.logwarn(" ".join(command))
            self.recording_process = subprocess.Popen(command)
            self.audio_recording('start')

            self.is_recording = True
            self.segment_num = 0
            # self.segment_success[self.segment_num] = True
            rospy.loginfo("Started recording.")

    def stop_recording(self):
        if self.is_recording:
            rospy.loginfo("Stopping recording...")
            if self.recording_process:
                # Terminate the rosbag process
                self.recording_process.terminate()
                self.recording_process.wait()
                self.recording_process = None
            self.segment_success[self.segment_num] = True   # assume last demo is correct?
            self.audio_recording('stop')
            self.save_segment_success()
            self.is_recording = False
            self.segment_num = 0
            self.segment_success = {}
            rospy.loginfo("Stopped recording.")

    def toggle_recording(self):
        if not self.is_recording:
            self.start_recording()
        else:
            self.stop_recording()

    def add_segment(self):
        if self.is_recording:
            self.segment_success[self.segment_num] = True
            self.segment_num += 1
            self.audio_recording('segment')
            rospy.loginfo(f"Started new segment number {self.segment_num}.")    
        else:
            rospy.loginfo("Currently not recording -> Can't start new segmentation part.")

    def toggle_segment_success(self):
        if self.is_recording and self.segment_num > 0:
            self.segment_success[self.segment_num] = not self.segment_success.get(self.segment_num, True)
            rospy.loginfo(f"Segment {self.segment_num} success status changed to {self.segment_success[self.segment_num]}")
        else:
            rospy.loginfo("No segment to toggle or not recording.")

    def audio_recording(self, command):
        if command == 'start' or command == 'stop' or command == 'segment':
            msg = String()
            msg.data = command
            self.pub_audio_command.publish(msg)
        else:
            rospy.logerr("Command invalid! Has to be either start or stop!")
            rospy.logerr("No audio instruction forwarded.")


    def save_segment_success(self):
        """Save the segment success information to a YAML file."""
        save_path = os.path.join(f"{self.base_path}/audio_processed/{self.timestamp}", 'segment_success.yaml')                # TODO: Change this, formalize it
        folder = f"{self.base_path}/audio_processed/{self.timestamp}"
        os.makedirs(folder, exist_ok=True)                      # had to makedirs here too because this happens before 
                                                                # the folder is created from audio node, shouldnt be a problem

        # Save the segment statuses to a YAML file
        with open(save_path, 'w') as yaml_file:
            yaml.dump({'segments': self.segment_success}, yaml_file, default_flow_style=False)

        rospy.loginfo(f"Segment statuses saved to {save_path}")

    def get_timestamp(self):
        self.timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        msg = String()
        msg.data = self.timestamp
        self.pub_curr_timestamp.publish(msg)
        return self.timestamp
    
    def add_segment_success(self):
        if self.is_recording:
            self.segment_success[self.segment_num] = True
            self.segment_num += 1
            self.audio_recording('segment')
            rospy.loginfo(f"Started new segment number {self.segment_num}.")    
        else:
            rospy.logwarn("Currently not recording -> Can't start new segmentation part.")

    def add_segment_failure(self):
        if self.is_recording:
            self.segment_success[self.segment_num] = False
            self.segment_num += 1
            self.audio_recording('segment')
            rospy.loginfo(f"Started new segment number {self.segment_num}.")    
        else:
            rospy.logwarn("Currently not recording -> Can't start new segmentation part.")


    def publish_flags(self, event):
        """Publish the current flags at a constant rate (30 Hz)."""
        self.pub_record_flag.publish(self.is_recording)

        msg = RecSegmentStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = self.segment_num
        self.pub_segment_num.publish(msg)



def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



if __name__ == "__main__":
    settings = None if sys.platform == 'win32' else termios.tcgetattr(sys.stdin)
    record_manager = RecordManager()

    sa_pub = rospy.Publisher("/cartesian_vic_teleop/alpha_virt", Float64, queue_size=1)

    # tm = TeleopWrapper()
    # toggle = False

    sa_val = 0

    try:
        while not rospy.is_shutdown():
            key = getKey(settings=settings, timeout=0.1)
            if key == 's':
                if record_manager.is_recording:
                    record_manager.add_segment_success()
                else:
                    record_manager.start_recording()
            elif key == 'p':
                record_manager.stop_recording()
            elif key == 'd':
                # record_manager.toggle_segment_success()
                record_manager.add_segment_failure()
            # elif key == 't':
            #     if not toggle:
            #         tm.start()
            #         toggle = True
            #         sa_val = 1
            #     else:
            #         tm.stop()
            #         toggle = False
            #         sa_val = 0
            else:
                if key == '\x03':   # ctrl-c
                    break

            sa_pub.publish(sa_val)
            
    except Exception as e:
        rospy.logerr(f"Exception occurred: {e}")
            


    # rospy.spin()



