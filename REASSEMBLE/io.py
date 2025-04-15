"""
HDF5 Utilities for Robot Data Processing

This module provides utilities for processing robot data stored in HDF5 format.
It includes functions for video encoding/decoding, data reorganization, and file I/O.
"""

import h5py
import numpy as np
import io
import imageio
import os
import cv2  # Note: imported but not directly used in the code


def numpy_to_mp4_blob(images: np.ndarray, fps: float = 30.0) -> bytes:
    """
    Convert a NumPy array of images to an MP4 video blob.
    
    Args:
        images: NumPy array of shape (num_frames, height, width, channels)
        fps: Frames per second for the output video
        
    Returns:
        Binary MP4 data as bytes
    """
    # Create an in-memory binary stream
    binary_stream = io.BytesIO()

    # Get the writer for MP4 format using the stream
    with imageio.get_writer(
        binary_stream, format="mp4", mode="I", fps=fps, macro_block_size=1
    ) as writer:
        for img in images:
            writer.append_data(img)

    # Get the binary blob and close the stream
    binary_blob = binary_stream.getvalue()
    binary_stream.close()

    return binary_blob


def mp4_blob_to_numpy(binary_blob: bytes) -> np.ndarray:
    """
    Convert an MP4 binary blob to a NumPy array of images.
    
    Args:
        binary_blob: Binary MP4 data as bytes
        
    Returns:
        NumPy array of shape (num_frames, height, width, channels)
    """
    # Create an in-memory binary stream from the MP4 binary blob
    binary_stream = io.BytesIO(binary_blob)

    # Open the video file from the binary stream
    reader = imageio.get_reader(binary_stream, format="mp4")

    # Read all frames and convert them to a list of NumPy arrays
    frames = []
    for frame in reader:
        frames.append(frame)

    reader.close()
    return np.array(frames)


def merge_dict_keys(data_dict):
    """
    Reorganize robot state data by merging related keys into structured arrays.
    
    This function combines separate x, y, z components into single vector arrays
    and organizes joint data into logical groups.
    
    Args:
        data_dict: Dictionary containing robot state data with individual components
        
    Returns:
        Reorganized dictionary with merged components
    """
    # Define which keys to merge and what their new names should be
    merge_map = {
        # Force and torque components to merge
        "compensated_base_force": ["compensated_base_force_x", "compensated_base_force_y", "compensated_base_force_z"],
        "compensated_base_torque": ["compensated_base_torque_x", "compensated_base_torque_y", "compensated_base_torque_z"],
        "measured_force": ["measured_force_x", "measured_force_y", "measured_force_z"],
        "measured_torque": ["measured_torque_x", "measured_torque_y", "measured_torque_z"]
    }
    
    # Define pose components (position and quaternion orientation)
    pose_keys = ["pose_x", "pose_y", "pose_z", "pose_qw", "pose_qx", "pose_qy", "pose_qz"]
    
    # Define velocity components (linear and angular)
    vel_keys = ["vel_x", "vel_y", "vel_z", "vel_qx", "vel_qy", "vel_qz"]

    # Define joint data keys (for a 9-joint robot: 7 arm joints + 2 gripper joints)
    joint_position_keys = [f"pos_joint{i}" for i in range(1, 10)]  # Joint positions
    joint_velocity_keys = [f"vel_joint{i}" for i in range(1, 10)]  # Joint velocities
    joint_effort_keys = [f"eff_joint{i}" for i in range(1, 10)]  # Joint efforts

    new_dict = {}
    
    # Merge keys from merge_map (force, torque, etc.)
    for new_key, old_keys in merge_map.items():
        merged_array = np.stack([data_dict[key] for key in old_keys], axis=-1)
        new_dict[new_key] = merged_array
        
        # Remove old keys after merging
        for key in old_keys:
            data_dict.pop(key, None)

    # Handle pose (position and orientation combined into one vector)
    pose_array = np.stack([data_dict[key] for key in pose_keys], axis=-1)
    new_dict["pose"] = pose_array
    
    # Remove old pose keys after merging
    for key in pose_keys:
        data_dict.pop(key, None)

    # Handle velocity (linear and angular combined into one vector)
    vel_array = np.stack([data_dict[key] for key in vel_keys], axis=-1)
    new_dict["velocity"] = vel_array
    
    # Remove old velocity keys after merging
    for key in vel_keys:
        data_dict.pop(key, None)

    # Handle main joint positions (first 7 joints - the robot arm)
    joint_position_array = np.stack([data_dict[key] for key in joint_position_keys[:7]], axis=-1)
    new_dict["joint_positions"] = joint_position_array

    # Handle gripper position (last 2 joints - the gripper)
    gripper_position_array = np.stack([data_dict[key] for key in joint_position_keys[7:]], axis=-1)
    new_dict["gripper_positions"] = gripper_position_array

    # Remove old joint position keys after merging
    for key in joint_position_keys:
        data_dict.pop(key, None)

    # Handle main joint velocities (first 7 joints - the robot arm)
    joint_velocity_array = np.stack([data_dict[key] for key in joint_velocity_keys[:7]], axis=-1)
    new_dict["joint_velocities"] = joint_velocity_array

    # Handle gripper velocities (last 2 joints - the gripper)
    gripper_velocity_array = np.stack([data_dict[key] for key in joint_velocity_keys[7:]], axis=-1)
    new_dict["gripper_velocities"] = gripper_velocity_array

    # Remove old joint velocity keys after merging
    for key in joint_velocity_keys:
        data_dict.pop(key, None)

    # Handle main joint efforts (first 7 joints - the robot arm)
    joint_effort_array = np.stack([data_dict[key] for key in joint_effort_keys[:7]], axis=-1)
    new_dict["joint_efforts"] = joint_effort_array

    # Handle gripper efforts (last 2 joints - the gripper)
    gripper_effort_array = np.stack([data_dict[key] for key in joint_effort_keys[7:]], axis=-1)
    new_dict["gripper_efforts"] = gripper_effort_array

    # Remove old joint effort keys after merging
    for key in joint_effort_keys:
        data_dict.pop(key, None)

    # Add other untouched keys back to the new dictionary
    new_dict.update(data_dict)
    
    return new_dict


def load_h5_file(file_path, decode=True):
    """
    Load and process an HDF5 file containing robot data.
    
    This function recursively converts HDF5 groups and datasets into Python dictionaries
    and numpy arrays, then reorganizes robot state data for easier access.
    
    Args:
        file_path: Path to the HDF5 file
        
    Returns:
        Dictionary containing the processed data
    """
    def recursively_convert_to_dict(obj, decode=decode):
        """
        Recursively convert HDF5 groups and datasets to Python dictionaries and arrays.
        
        Args:
            obj: An HDF5 object (group or dataset)
            
        Returns:
            Dictionary or array representation of the HDF5 object
        """
        if isinstance(obj, h5py.Group):
            return {key: recursively_convert_to_dict(obj[key]) for key in obj.keys()}
        elif isinstance(obj, h5py.Dataset):
            data = obj[()]
            if decode and isinstance(data, np.void):
                return mp4_blob_to_numpy(data)
            else:
                return data  # Convert dataset to numpy array or other types
        else:
            raise TypeError("Unknown object type")

    # Open the HDF5 file
    with h5py.File(file_path, 'r') as h5_file:
        data = recursively_convert_to_dict(h5_file)
        
        # Process robot state data if it contains the expected keys
        if "robot_state" in data and "compensated_base_force_x" in data["robot_state"]:
            data["robot_state"] = merge_dict_keys(data["robot_state"])
            
            # Also process timestamp data if available
            if "timestamps" in data:
                data["timestamps"] = merge_dict_keys(data["timestamps"])
        
        # Ensure timestamps are 1D arrays
        if "timestamps" in data:
            for k, v in data["timestamps"].items():
                if len(v.shape) != 1:
                    data["timestamps"][k] = v[:, 0]

        return data


def write_dict_to_hdf5(hdf5_file, data_dict: dict, keys_to_ignore: tuple = ()):
    """
    Write a nested dictionary to an HDF5 file.
    
    Args:
        hdf5_file: An open HDF5 file object
        data_dict: Dictionary containing data to write
        keys_to_ignore: Tuple of keys to skip when writing
    """
    for key, curr_data in data_dict.items():
        if key in keys_to_ignore:
            continue

        # Convert lists to numpy arrays
        if isinstance(curr_data, list):
            curr_data = np.array(curr_data)

        # Recursively handle nested dictionaries
        if isinstance(curr_data, dict):
            if key not in hdf5_file:
                hdf5_file.create_group(key)
            write_dict_to_hdf5(hdf5_file[key], curr_data, keys_to_ignore=keys_to_ignore)
        else:
            # Replace existing dataset if it exists
            if key in hdf5_file:
                del hdf5_file[key]
            hdf5_file[key] = curr_data


def load_segment_info(h5_dir):
    """
    Load segmentation information from all HDF5 files in a directory.
    
    This function extracts task segmentation data, including hierarchical 
    subtask information with start/end times and success status.
    
    Args:
        h5_dir: Directory containing HDF5 files
        
    Returns:
        Dictionary mapping filenames to lists of segment information
    """
    # Get all HDF5 files in the directory
    h5_file_paths = [os.path.join(h5_dir, p) for p in os.listdir(h5_dir) if p.endswith('.h5')]

    records = {}
    for h5_file_path in h5_file_paths:
        with h5py.File(h5_file_path, "r") as f:
            segments_info_group = f["segments_info"]

            all_segments = {}
            for segment_idx, segment_group in segments_info_group.items():
                # Verify segment index consistency
                segment_idx2 = str(int(segment_group['index'][...]))
                assert segment_idx == segment_idx2, f'Index mismatch in {h5_file_path} at {segment_idx}.'

                # Extract segment data
                segment_data = {
                    'start': float(segment_group['start'][...]),
                    'end': float(segment_group['end'][...]),
                    'success': bool(segment_group['success'][...]),
                    'text': segment_group['text'][...].item().decode('utf-8')}

                # Process subsegments (low-level tasks)
                all_subsegments = {}
                if 'low_level' in segment_group.keys():
                    for group_idx, subgroup in segment_group['low_level'].items():
                        subsegment_data = {
                            'start': float(subgroup['start'][...]),
                            'end': float(subgroup['end'][...]),
                            'success': bool(subgroup['success'][...]),
                            'text': subgroup['text'][...].item().decode('utf-8')}

                        all_subsegments[group_idx] = subsegment_data

                # Convert dictionary of subsegments to ordered list
                all_subsegments_list = [all_subsegments[str(i)] for i in range(len(all_subsegments))]
                segment_data['low_level'] = all_subsegments_list
                all_segments[segment_idx] = segment_data

            # Convert dictionary of segments to ordered list
            all_segments_list = [all_segments[str(i)] for i in range(len(all_segments))]
            records[os.path.basename(h5_file_path)] = all_segments_list
            print(f'Loaded {len(segments_info_group)} actions from {h5_file_path}.')

    return records


if __name__ == "__main__":
    # Example usage
    h5_file_path = '/home/dsliwowski/datasets/gp/processed_test/2025-01-09-13-57-17.h5'
    data = load_h5_file(h5_file_path)
    
    # Enter interactive debugging mode
    import pdb
    pdb.set_trace()