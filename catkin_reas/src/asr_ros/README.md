## Installation

Clone this repo and install it in your workspace.


### Dependencies

Install gstreamer dependencies:
```bash
sudo apt install portaudio19-dev python3-all-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
```

Install python depencies for audio and ASR processing:
```bash
pip install pydub 
```

## How to use
In one terminal run:
```bash
roslaunch asr_ros audio_record.launch 
```

In another terminal run:
```bash
roslaunch asr_ros audio_process.launch
```
***
If you want to debug or test, you could use 
```bash
rostopic pub -1 /audio_commands std_msgs/String "data: 'start'"
```
and
```bash
rostopic pub -1 /audio_commands std_msgs/String "data: 'stop'"
```

to start and stop recording the audio, respectively. In general, other nodes should publish start/stop commands through this topic in order to control the audio recording process.


## Workflow
There are two main nodes in this package, node for *audio recording* and node for *audio processing*. 

*Audio recording* node starts capturing the wave file from default audio input device, usually the microphone you use, and waits for the signal to start or stop the recording. The file is then saved in audio_folder path which can be changed in ```record_audio.py```.

It also publishes the path to the newly saved recording to audio processing node, which applies ASR on it using Whisper. Transcription is shown in the terminal as a way for the user to check the results, as well as saved in the audio_folder next to the corresponding .wav file.

