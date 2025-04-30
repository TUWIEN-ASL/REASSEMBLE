#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import os
from pydub import AudioSegment
import wave
import whisper
import json
import rospkg
from pathlib import Path



class AudioProcessor:
    def __init__(self):
        rospy.init_node('audio_processor', anonymous=True)
        self.teleop_folder = rospy.get_param("~base_path")
        self.audio_folder = os.path.join(self.teleop_folder, 'audio')
        self.output_base_path = os.path.join(self.teleop_folder, 'audio_processed')
        os.makedirs(self.output_base_path, exist_ok=True)

        print("loading whisper model")
        self.model = whisper.load_model("small.en")  # You can choose a different model size
        print("whisper model loaded")

        self.timestamp = None

        print("Creating subscribers")
        rospy.Subscriber('/audio_processed', String, self.audio_processed_callback, queue_size=200)
        rospy.Subscriber('/curr_timestamp', String, self.timestamp_callback, queue_size=200)
        rospy.loginfo("Audio Processor Node Initialized")
        rospy.spin()

    def audio_processed_callback(self, msg):
        file_name = msg.data
        rospy.loginfo(f"Received audio file for processing: {file_name}")
        self.process_audio_file(file_name)
    
    def timestamp_callback(self, msg):
        self.timestamp = msg.data

    def convert_to_whisper_format(self, input_file_path, output_file_path):
        # Load the audio file
        audio = AudioSegment.from_file(input_file_path)
        
        # # Convert to mono
        # mono_audio = audio.set_channels(1)
        
        # # Set sample rate to 16 kHz
        # mono_audio = mono_audio.set_frame_rate(16000)
        
        # # Export as 16-bit PCM WAV file
        # mono_audio.export(output_file_path, format="wav", codec="pcm_s16le")
        audio.export(output_file_path, format="wav", codec="pcm_s16le")

    def verify_audio_format(self, file_path):
        # Open the WAV file and check its properties
        with wave.open(file_path, 'rb') as wf:
            sample_width = wf.getsampwidth()
            num_channels = wf.getnchannels()
            frame_rate = wf.getframerate()

            # Check if the audio file is 16-bit PCM mono
            if sample_width != 2:
                raise ValueError("Audio file must be 16-bit PCM format")
            if num_channels != 1:
                raise ValueError("Audio file must be mono")
            if frame_rate != 16000:
                raise ValueError("Audio file must have a sample rate of 16 kHz")

        rospy.loginfo(f"Audio file '{file_path}' is ready for Whisper.")

    def transcribe_audio_whisper(self, file_path):
        # Transcribe the audio
        result = self.model.transcribe(file_path)
        return result['text']
    

    def process_audio_file(self, file_name):
        # Create a unique folder for the processed audio
        segment_num = os.path.basename(file_name).replace('.wav', '')
        path_parts = file_name.split('/')
        processed_folder = os.path.join(self.output_base_path, path_parts[-2])
        os.makedirs(processed_folder, exist_ok=True)

        # Paths for processed audio and transcript
        processed_audio_path = os.path.join(processed_folder, f"{segment_num}.wav")
        transcript_path = os.path.join(processed_folder, f"{segment_num}.txt")

        # Convert and verify the audio
        self.convert_to_whisper_format(file_name, processed_audio_path)
        # self.verify_audio_format(processed_audio_path)

        # Transcribe and save the transcript
        transcript = self.transcribe_audio_whisper(processed_audio_path)
        with open(transcript_path, 'w') as f:
            f.write(transcript)
        
        rospy.loginfo("Transcript: ")
        rospy.loginfo(transcript)
        rospy.loginfo(f"Processed audio and transcript saved to '{processed_folder}'")

if __name__ == '__main__':
    try:
        AudioProcessor()
    except rospy.ROSInterruptException:
        pass
