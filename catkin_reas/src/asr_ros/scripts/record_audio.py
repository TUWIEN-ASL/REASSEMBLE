#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import wave
import os
from datetime import datetime
from pathlib import Path
import rospkg

class AudioRecorder:
    def __init__(self):
        rospy.init_node('audio_recorder', anonymous=True)
        self.recording = False
        self.audio_data = []
        self.timestamp = None
        self.curr_segment = 0

        self.teleop_folder = rospy.get_param("~base_path")
        self.audio_folder = os.path.join(self.teleop_folder, 'audio')
        self.audio_subfolder = None
        os.makedirs(self.audio_folder, exist_ok=True) 

        self.audio_processed_publisher = rospy.Publisher('/audio_processed', String, queue_size=200)
        rospy.Subscriber('/audio_commands', String, self.command_callback)
        rospy.Subscriber('/audio/audio', AudioData, self.audio_callback)
        rospy.Subscriber('/curr_timestamp', String, self.timestamp_callback)

        rospy.loginfo("Audio Recorder Node Initialized")
        rospy.spin()


    def command_callback(self, msg):
        if msg.data == 'start':
            rospy.loginfo("Starting recording...")
            self.start_recording()
        elif msg.data == 'stop':
            rospy.loginfo("Stopping recording...")
            self.stop_recording()
            self.save_audio()
            self.curr_segment = 0
        elif msg.data == 'segment':
            self.segment_recording()

    def audio_callback(self, msg):
        if self.recording:
            self.audio_data.append(msg.data)

    def start_recording(self):
        self.recording = True
        self.audio_data = []
        print(self.audio_folder)
        print(self.timestamp)
        self.audio_subfolder = os.path.join(self.audio_folder, self.timestamp)
        os.makedirs(self.audio_subfolder, exist_ok=True)

    def stop_recording(self):
        self.recording = False

    def segment_recording(self):
        self.save_audio()
        self.audio_data = []
        self.curr_segment += 1

    def timestamp_callback(self, msg):
        self.timestamp = msg.data

    def save_audio(self):
        if not self.audio_data:
            rospy.logwarn("No audio data recorded.")
            return

        file_name = os.path.join(self.audio_subfolder, f'{self.curr_segment}.wav')

        # Save audio data to a file
        with wave.open(file_name, 'wb') as wf:
            wf.setnchannels(2)
            wf.setsampwidth(2)          # 16-bit audio
            wf.setframerate(32000)      # 16 kHz sample rate
            wf.writeframes(b''.join(self.audio_data))

        rospy.loginfo(f"Audio data saved to {file_name}")

        # Notify the processing node
        self.audio_processed_publisher.publish(file_name)

if __name__ == '__main__':
    try:
        AudioRecorder()
    except rospy.ROSInterruptException:
        pass
