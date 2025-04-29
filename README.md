# REASSEMBLE: A Multimodal Dataset for Contact-rich Robotic Assembly and Disassembly

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![GitHub Stars](https://img.shields.io/github/stars/TUWIEN-ASL/REASSEMBLE.svg)](https://github.com/TUWIEN-ASL/REASSEMBLE/stargazers)
[![GitHub Issues](https://img.shields.io/github/issues/TUWIEN-ASL/REASSEMBLE.svg)](https://github.com/TUWIEN-ASL/REASSEMBLE/issues)
[![GitHub Pull Requests](https://img.shields.io/github/issues-pr/TUWIEN-ASL/REASSEMBLE.svg)](https://github.com/TUWIEN-ASL/REASSEMBLE/pulls)
[![Python Version](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![Paper](https://img.shields.io/badge/arXiv-2502.05086-b31b1b.svg)](https://arxiv.org/abs/2502.05086)
[![Website](https://img.shields.io/badge/Website-Project_Page-green.svg)](https://tuwien-asl.github.io/REASSEMBLE_page/)

## 📋 Abstract

Robotic manipulation remains a core challenge in robotics, particularly for contact-rich tasks such as industrial assembly and disassembly. Existing datasets have significantly advanced learning in manipulation but are primarily focused on simpler tasks like object rearrangement, falling short of capturing the complexity and physical dynamics involved in assembly and disassembly. To bridge this gap, we present REASSEMBLE (Robotic assEmbly disASSEMBLy datasEt), a new dataset designed specifically for contact-rich manipulation tasks. Built around the NIST Assembly Task Board 1 benchmark, REASSEMBLE includes four actions (pick, insert, remove, and place) involving 17 objects. The dataset contains 4,551 demonstrations, of which 4,035 were successful, spanning a total of 781 minutes. Our dataset features multi-modal sensor data including event cameras, force-torque sensors, microphones, and multi-view RGB cameras. This diverse dataset supports research in areas such as learning contact-rich manipulation, task condition identification, action segmentation, and more. We believe REASSEMBLE will be a valuable resource for advancing robotic manipulation in complex, real-world scenarios.

### ✨ Key Features

- Multimodality: REASSEMBLE contains data from robot proprioception, RGB cameras, Force&Torque sensors, microphones, and event cameras
- Multitask labels: REASSEMBLE contains labeling which enables research in Temporal Action Segmentation, Motion Policy Learning, Anomaly detection, and Task Inversion.
- Long horizon: Demonstrations in the REASSEMBLE dataset cover long horizon tasks and actions which usually span multiple steps.
- Hierarchical labels: REASSEMBLE contains actions segmentation labels at two hierarchical levels.

## 🚀 Getting Started

### Prerequisites

List of all the prerequisites required to use the project:

```
Python 3.10+
conda (recommended)
```

### Installation

Step-by-step guide on how to install the project:

```bash
# Clone the repository
git clone https://github.com/TUWIEN-ASL/REASSEMBLE.git

# Navigate to the project directory
cd REASSEMBLE

# Create and activate a conda environment (optional but recommended)
conda create -n REASSEMBLE python=3.10
conda activate REASSEMBLE

# Install dependencies
pip install -r requirements.txt
conda install conda-forge::ffmpeg

# Install REASSEMBLE package
pip install -e .
```

Before building an image from the Dockerfile, the NatNet SDK must be downloaded in advance. Run ```install_sdk.sh``` in ```catkin_reas/src/natnet_ros_cpp``` to download and extract the SDK.

### Downloading dataset
The dataset can be downloaded from the following link [google drive](https://drive.google.com/drive/u/1/folders/1HPsG63iI2tpovJoh_o2zhmyx9muNcnVx)

### Usage
Scripts are parametrized with argparse, so in doubt you and always add the ```-h``` flag to get a description of the parameters.

Example for running the dataset conversion:
```
python scripts/conversions/h5_to_rlds.py --input_dir data/REASSEMBLE --output_dir data/ds_rlds --max_files 2
```

Example for running visualization:
```
python scripts/visualization/vizualize_data.py  data/REASSEMBLE_corrected/2025-01-13-09-43-29.h5 --cleanup
```

## 📑 Dataset Structure

The dataset consists of several HDF5 (.h5) and JSON (.json) files, organized into two directories. The poses directory contains the JSON files, which store the poses of the cameras and the board in the world coordinate frame. The data directory contains the HDF5 files, which store the sensory readings and annotations collected as part of the REASSEMBLE dataset. Each JSON file can be matched with its corresponding HDF5 file based on their filenames, which include the timestamp when the data was recorded. For example, 2025-01-09-13-59-54_poses.json corresponds to 2025-01-09-13-59-54.h5.

The structure of the JSON files is as follows:

```
{"Hama1": [
        [x ,y, z],
        [qx, qy, qz, qw]
 ], 
 "Hama2": [
        [x ,y, z],
        [qx, qy, qz, qw]
 ], 
 "DAVIS346": [
        [x ,y, z],
        [qx, qy, qz, qw]
 ], 
 "NIST_Board1": [
        [x ,y, z],
        [qx, qy, qz, qw]
 ]
}
```
[x, y, z] represent the position of the object, and [qx, qy, qz, qw] represent its orientation as a quaternion.

The HDF5 (.h5) format organizes data into two main types of structures: datasets, which hold the actual data, and groups, which act like folders that can contain datasets or other groups. In the diagram below, groups are shown as folder icons, and datasets as file icons. The main group of the file directly contains the video, audio, and event data. To save memory, video and audio are stored as encoded byte strings, while event data is stored as arrays. The robot’s proprioceptive information is kept in the robot_state group as arrays. Because different sensors record data at different rates, the arrays vary in length (signified by the N_xxx variable in the data shapes). To align the sensory data, each sensor’s timestamps are stored separately in the timestamps group. Information about action segments is stored in the segments_info group. Each segment is saved as a subgroup, named according to its order in the demonstration, and includes a start timestamp, end timestamp, a success indicator, and a natural language description of the action. Within each segment, low-level skills are organized under a low_level subgroup, following the same structure as the high-level annotations.

```
📁 <date_time>.h5
├──📄 hama1 - mp4 encoded video
├──📄 hama2_audio - mp3 encoded audio
├──📄 hama2 - mp4 encoded video
├──📄 hama2_audio - mp3 encoded audio
├──📄 hand - mp4 encoded video
├──📄 hand_audio - mp3 encoded audio
├──📄 capture_node - mp4 encoded video (Event camera)
├──📄 events - N_events x 3 (x, y, polarity)
├──📁 robot_state
│   ├──📄 compensated_base_force - N_bf x 3 (x, y, z)
│   ├──📄 compenseted_base_torque - N_bt x 3 (x, y, z)
│   ├──📄 gripper_positions - N_grip x 2 (left, right)
│   ├──📄 joint_efforts - N_je x 7 (one for each joint)
│   ├──📄 joint_positions - N_jp x 7 (one for each joint)
│   ├──📄 joint_velocities - N_jv x 7 (one for each joint)
│   ├──📄 measured_force - N_mf x 3 (x, y, z)
│   ├──📄 measured_torque - N_mt x 7 (x, y, z)
│   ├──📄 pose - N_poses x 7 (x, y, z, qw, qx, qy, qz)
│   └──📄 velocity - N_vels x 7 (x, y, z, ω, γ, θ)
├──📁 timestamps
│   ├──📄 hama1 - N_hama1 x 1
│   ├──📄 hama2 - N_hama1 x 1
│   ├──📄 hand - N_hand x 1
│   ├──📄 capture_node - N_capture x 1
│   ├──📄 events - N_events x 1
│   ├──📄 compensated_base_force - N_bf x 1
│   ├──📄 compenseted_base_torque - N_bt x 1
│   ├──📄 gripper_positions - N_grip x 1
│   ├──📄 joint_efforts - N_je x 1
│   ├──📄 joint_positions - N_jp x 1
│   ├──📄 joint_velocities - N_jv x 1
│   ├──📄 measured_force - N_mf x 1
│   ├──📄 measured_torque - N_mt x 1
│   ├──📄 pose - N_poses x 1
│   └──📄 velocity - N_vels x 1
└──📁 segments_info
    ├──📁 0
    │   ├──📄 start - scalar
    │   ├──📄 end - scalar
    │   ├──📄 success - Boolean
    │   ├──📄 text - scalar
    │   └──📁 Low_level
    │       ├──📁 0
    │       │   ├──📄 start - scalar
    │       │   ├──📄 end - scalar
    │       │   ├──📄 success - Boolean
    │       │   └──📄 text - scalar
    │       └──📁 1
    │           ⋮
    └──📁 1
        ⋮
```

## ⚠️ File comments

| Recording              | Issue                             |
|------------------------|-----------------------------------|
| 2025-01-10-15-28-50.h5 | hand cam missing at beginning     |
| 2025-01-10-16-17-40.h5 | missing hand cam                  |
| 2025-01-10-17-10-38.h5 | hand cam missing at beginning     |
| 2025-01-10-17-54-09.h5 | no empty action at beginning      |
| 2025-01-11-14-22-09.h5 | no empty action at beginning      |
| 2025-01-11-14-45-48.h5 | F/T not valid for last action     |
| 2025-01-11-15-27-19.h5 | F/T not valid for last action     |
| 2025-01-11-15-35-08.h5 | F/T not valid for last action     |
| 2025-01-13-11-16-17.h5 | gripper broke for last action     |
| 2025-01-13-11-18-57.h5 | pose not available for last action |

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- We would like to thank the people responsible for the [DROID dataset](https://droid-dataset.github.io/) and for sharing their codebase. The code structure within this
repository is inspired by their structure.

## 📖 Bibtex

```
@INPROCEEDINGS{Sliwowski-RSS-25, 
    AUTHOR    = {Daniel Sliwowski AND Shail Jadav AND Sergej Stanovcic AND Jedrzej Orbik AND Johannes Heidersberger AND Dongheui Lee}, 
    TITLE     = {{Demonstrating REASSEMBLE: A Multimodal Dataset for Contact-rich Robotic Assembly and Disassembly}}, 
    BOOKTITLE = {Proceedings of Robotics: Science and Systems}, 
    YEAR      = {2025}, 
    ADDRESS   = {Los Angeles, USA}, 
    MONTH     = {June}, 
    DOI       = {} 
} 
  
```

## 📞 Contact

Daniel Sliwowski - [daniel.sliwowski@tuwien.ac.at](daniel.sliwowski@tuwien.ac.at)
