#!/usr/bin/env python3
"""
RLDS Data Conversion Tool

This script is an example hot o convert the h5 raw data into RLDS format. The target RLDS format used
in this script is compatible with the one of DORID, meaning their policy learning implementation
should work with the resulting dataset.

This script converts HDF5 robot demonstration data into the RLDS (Reinforcement Learning Datasets) format,
specifically optimized for robot manipulation tasks. The tool supports both single-process and
multiprocessing modes for efficient data conversion.

Original data contains:
- Robot state information (joint positions, velocities)
- Robot end-effector pose (position and orientation)
- Gripper state (position and velocity)
- Camera images from multiple cameras (hand/wrist camera and external cameras)
- Natural language instructions describing the tasks

The converted data follows the RLDS format which is compatible with TensorFlow Datasets (TFDS).
Each episode is stored with appropriate metadata and can be filtered by target action phrases.

Usage:
    python convert_to_rlds.py --input_dir /path/to/h5/files --output_dir /output/path --dataset_name ROBOT_DEMOS --target_action "gear" --multiprocess

Author: Daniel Sliwowski (daniel.sliwowski@tuwien.ac.at)
Date: April 15, 2025
"""

import os
import cv2 
import glob
import json
import string
import argparse
import numpy as np
import multiprocessing as mp
from typing import Dict, Any, List, Optional, Tuple, Union
import tqdm
import h5py
import tensorflow as tf
from scipy.spatial.transform import Rotation as R

# Import custom module for loading HDF5 files
# from read_data import load_h5_file
from REASSEMBLE.io import load_h5_file


def parse_arguments():
    """
    Parse command line arguments for the RLDS data conversion tool.
    
    Returns:
        argparse.Namespace: Parsed command line arguments
    """
    parser = argparse.ArgumentParser(description='Convert robot demonstration data to RLDS format')
    
    parser.add_argument('--input_dir', type=str, required=True, 
                        help='Directory containing HDF5 files to process')
    
    parser.add_argument('--output_dir', type=str, default='./ds_rlds',
                        help='Directory to save the converted RLDS dataset (default: ./ds_rlds)')
    
    parser.add_argument('--dataset_name', type=str, default='REASSEMBLE',
                        help='Name of the output dataset (default: REASSEMBLE)')
    
    parser.add_argument('--num_shards', type=int, default=50,
                        help='Number of shards to split the dataset into (default: 50)')
    
    parser.add_argument('--target_action', type=str, default='gear',
                        help='Target action to filter for in segments (default: "gear")')
    
    parser.add_argument('--downsample_ratio', type=int, default=1,
                        help='Downsample ratio for frame rate (default: 1, no downsampling)')
    
    parser.add_argument('--multiprocess', action='store_true', 
                        help='Use multiprocessing for faster conversion')
    
    parser.add_argument('--num_processes', type=int, default=None,
                        help='Number of processes to use (default: number of CPU cores)')
    
    parser.add_argument('--max_files', type=int, default=None,
                        help='Maximum number of files to process (default: all files)')
    
    parser.add_argument('--verbose', action='store_true',
                        help='Enable verbose output')
    
    return parser.parse_args()


def convert_dict_to_rlds_dataset(merged_dict: Dict[str, Any]) -> tf.data.Dataset:
    """
    Convert a merged dictionary to an RLDS dataset.
    
    Args:
        merged_dict: A dictionary containing RLDS data
                    
    Returns:
        A TensorFlow dataset in RLDS format.
    """
    # Determine the number of steps in the trajectory
    num_steps = len(merged_dict.get('reward', merged_dict.get('is_first', [])))
    
    # Create a dataset generator function
    def generator():
        for i in range(num_steps):
            step = {}
            
            # Extract scalar fields
            for key in ['discount', 'is_first', 'is_last', 'is_terminal', 'reward']:
                if key in merged_dict and len(merged_dict[key]) > i:
                    step[key] = merged_dict[key][i]
            
            # Extract string fields (language instructions)
            for key in ['language_instruction', 'language_instruction_2', 'language_instruction_3']:
                if key in merged_dict and len(merged_dict[key]) > i:
                    step[key] = merged_dict[key][i]
            
            # Extract nested observation dictionary
            if 'observation' in merged_dict:
                step['observation'] = {}
                for obs_key, obs_value in merged_dict['observation'].items():
                    if len(obs_value) > i:  # Check if the list is not empty at this index
                        step['observation'][obs_key] = obs_value[i]
            
            # Extract action (can be a tensor or a dictionary)
            if 'action' in merged_dict and len(merged_dict['action']) > i:
                step['action'] = merged_dict['action'][i]
            
            # Extract action_dict if present
            if 'action_dict' in merged_dict:
                step['action_dict'] = {}
                for act_key, act_value in merged_dict['action_dict'].items():
                    if len(act_value) > i:  # Check if the list is not empty at this index
                        step['action_dict'][act_key] = act_value[i]
            
            yield step
    
    # Define the output signature for the dataset
    output_signature = create_step_signature(merged_dict)
    
    # Create the steps dataset using from_generator
    steps_dataset = tf.data.Dataset.from_generator(
        generator,
        output_signature=output_signature
    )
    
    # Create the episode structure
    episode_data = {'steps': steps_dataset}
    
    # If there's episode metadata, include it
    if 'episode_metadata' in merged_dict:
        episode_data['episode_metadata'] = merged_dict['episode_metadata']
    
    # Return a dataset with a single element (the episode)
    return tf.data.Dataset.from_tensors(episode_data)


def create_step_signature(merged_dict: Dict[str, Any]) -> Dict[str, Any]:
    """
    Create a TensorSpec signature for the steps dataset based on the merged_dict structure.
    
    Args:
        merged_dict: The merged dictionary containing RLDS data.
        
    Returns:
        A dictionary of TensorSpecs matching the structure of each step.
    """
    signature = {}
    
    # Add scalar fields
    for key in ['discount', 'is_first', 'is_last', 'is_terminal', 'reward']:
        if key in merged_dict and len(merged_dict[key]) > 0:
            # Get the first element to determine shape and dtype
            sample = merged_dict[key][0]
            if isinstance(sample, np.ndarray) and sample.size == 1:
                # For scalar numpy arrays, use the scalar shape
                signature[key] = tf.TensorSpec(shape=(), dtype=tf.dtypes.as_dtype(sample.dtype))
            else:
                # For other types, infer from the value
                signature[key] = tf.TensorSpec(shape=(), dtype=tf.dtypes.as_dtype(type(sample)))
    
    # Add string fields
    for key in ['language_instruction', 'language_instruction_2', 'language_instruction_3']:
        if key in merged_dict and len(merged_dict[key]) > 0:
            signature[key] = tf.TensorSpec(shape=(), dtype=tf.string)
    
    # Add observation fields
    if 'observation' in merged_dict:
        signature['observation'] = {}
        for obs_key, obs_value in merged_dict['observation'].items():
            if len(obs_value) > 0:
                sample = obs_value[0]
                if isinstance(sample, np.ndarray):
                    signature['observation'][obs_key] = tf.TensorSpec(
                        shape=sample.shape, dtype=tf.dtypes.as_dtype(sample.dtype)
                    )
    
    # Add action field
    if 'action' in merged_dict and len(merged_dict['action']) > 0:
        sample = merged_dict['action'][0]
        if isinstance(sample, np.ndarray):
            signature['action'] = tf.TensorSpec(
                shape=sample.shape, dtype=tf.dtypes.as_dtype(sample.dtype)
            )
    
    # Add action_dict fields
    if 'action_dict' in merged_dict:
        signature['action_dict'] = {}
        for act_key, act_value in merged_dict['action_dict'].items():
            if len(act_value) > 0:
                sample = act_value[0]
                if isinstance(sample, np.ndarray):
                    signature['action_dict'][act_key] = tf.TensorSpec(
                        shape=sample.shape, dtype=tf.dtypes.as_dtype(sample.dtype)
                    )
    
    return signature


def create_rlds_episode(steps_dict: Dict[str, Any], episode_metadata: Dict[str, Any] = None) -> tf.data.Dataset:
    """
    Create an RLDS episode from a dictionary of steps and optional metadata.
    
    Args:
        steps_dict: Dictionary containing step data with the same structure as in merged_dict.
        episode_metadata: Optional metadata for the episode.
        
    Returns:
        A TensorFlow dataset in RLDS format representing a single episode.
    """
    # If episode metadata is provided, add it to the dictionary
    if episode_metadata is not None:
        steps_dict['episode_metadata'] = episode_metadata
    
    # Convert to RLDS dataset
    return convert_dict_to_rlds_dataset(steps_dict)


def save_as_tfds_dataset(episodes, output_dir, dataset_name, num_shards=10):
    """
    Save episodes as a sharded TensorFlow Dataset in TFDS format,
    generating all metadata files to match the expected structure.
    
    Args:
        episodes: List of episode dictionaries to save
        output_dir: Directory to save the dataset
        dataset_name: Name for the dataset
        num_shards: Number of shards to split the dataset into
    
    Returns:
        str: Path to the saved dataset directory
    """
    # Create output directory structure
    dataset_dir = os.path.join(output_dir, dataset_name)
    os.makedirs(dataset_dir, exist_ok=True)
    
    # Create TFRecord writers for each shard
    writers = []
    for shard_id in range(num_shards):
        shard_path = os.path.join(dataset_dir, f"{dataset_name}-train.tfrecord-{shard_id:05d}-of-{num_shards:05d}")
        writers.append(tf.io.TFRecordWriter(shard_path))
    
    # Track examples per shard for metadata
    shard_lengths = [0] * num_shards
    
    # Helper functions for serialization
    def _bytes_feature(value):
        """Convert string/bytes to bytes feature."""
        if isinstance(value, str):
            value = value.encode()
        return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))
    
    def _float_feature(value):
        """Convert float to float feature."""
        return tf.train.Feature(float_list=tf.train.FloatList(value=[value]))
    
    def _int64_feature(value):
        """Convert int to int64 feature."""
        return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))
    
    def _bool_feature(value):
        """Convert bool to int64 feature."""
        return tf.train.Feature(int64_list=tf.train.Int64List(value=[int(value)]))
    
    def _float_array_feature(value):
        """Convert array of floats to float array feature."""
        if not isinstance(value, (list, np.ndarray)):
            value = [value]
        return tf.train.Feature(float_list=tf.train.FloatList(value=value.flatten() if isinstance(value, np.ndarray) else value))
    
    # Process episodes
    episode_count = 0
    total_step_count = 0
    total_bytes = 0
    
    # Sample the first episode to determine data structure
    sample_episode = episodes[0]
    
    # Generate feature schema from data
    feature_schema = {
        "featuresDict": {
            "features": {
                "episode_metadata": {
                    "featuresDict": {
                        "features": {
                            "file_path": {
                                "description": "Path to the original data file.",
                                "pythonClassName": "tensorflow_datasets.core.features.tensor_feature.Tensor",
                                "tensor": {
                                    "dtype": "string",
                                    "encoding": "none",
                                    "shape": {}
                                }
                            },
                            "recording_folderpath": {
                                "description": "Path to the folder of recordings.",
                                "pythonClassName": "tensorflow_datasets.core.features.tensor_feature.Tensor",
                                "tensor": {
                                    "dtype": "string",
                                    "encoding": "none",
                                    "shape": {}
                                }
                            }
                        }
                    },
                    "pythonClassName": "tensorflow_datasets.core.features.features_dict.FeaturesDict"
                },
                "steps": {
                    "pythonClassName": "tensorflow_datasets.core.features.dataset_feature.Dataset",
                    "sequence": {
                        "feature": {
                            "featuresDict": {
                                "features": {
                                    "action": {
                                        "description": "Robot action, consists of joint velocities and gripper position.",
                                        "pythonClassName": "tensorflow_datasets.core.features.tensor_feature.Tensor",
                                        "tensor": {
                                            "dtype": "float64",
                                            "encoding": "none",
                                            "shape": {
                                                "dimensions": [str(sample_episode['steps']['action'].shape[1])]
                                            }
                                        }
                                    },
                                    "action_dict": {
                                        "featuresDict": {
                                            "features": {}
                                        },
                                        "pythonClassName": "tensorflow_datasets.core.features.features_dict.FeaturesDict"
                                    },
                                    "discount": {
                                        "description": "Discount if provided, default to 1.",
                                        "pythonClassName": "tensorflow_datasets.core.features.scalar.Scalar",
                                        "tensor": {
                                            "dtype": "float32",
                                            "encoding": "none",
                                            "shape": {}
                                        }
                                    },
                                    "is_first": {
                                        "pythonClassName": "tensorflow_datasets.core.features.tensor_feature.Tensor",
                                        "tensor": {
                                            "dtype": "bool",
                                            "encoding": "none",
                                            "shape": {}
                                        }
                                    },
                                    "is_last": {
                                        "pythonClassName": "tensorflow_datasets.core.features.tensor_feature.Tensor",
                                        "tensor": {
                                            "dtype": "bool",
                                            "encoding": "none",
                                            "shape": {}
                                        }
                                    },
                                    "is_terminal": {
                                        "pythonClassName": "tensorflow_datasets.core.features.tensor_feature.Tensor",
                                        "tensor": {
                                            "dtype": "bool",
                                            "encoding": "none",
                                            "shape": {}
                                        }
                                    },
                                    "language_instruction": {
                                        "description": "Language Instruction.",
                                        "pythonClassName": "tensorflow_datasets.core.features.tensor_feature.Tensor",
                                        "tensor": {
                                            "dtype": "string",
                                            "encoding": "none",
                                            "shape": {}
                                        }
                                    },
                                    "language_instruction_2": {
                                        "description": "Language Instruction 2.",
                                        "pythonClassName": "tensorflow_datasets.core.features.tensor_feature.Tensor",
                                        "tensor": {
                                            "dtype": "string",
                                            "encoding": "none",
                                            "shape": {}
                                        }
                                    },
                                    "language_instruction_3": {
                                        "description": "Language Instruction 3.",
                                        "pythonClassName": "tensorflow_datasets.core.features.tensor_feature.Tensor",
                                        "tensor": {
                                            "dtype": "string",
                                            "encoding": "none",
                                            "shape": {}
                                        }
                                    },
                                    "observation": {
                                        "featuresDict": {
                                            "features": {}
                                        },
                                        "pythonClassName": "tensorflow_datasets.core.features.features_dict.FeaturesDict"
                                    },
                                    "reward": {
                                        "description": "Reward if provided, 1 on final step for demos.",
                                        "pythonClassName": "tensorflow_datasets.core.features.scalar.Scalar",
                                        "tensor": {
                                            "dtype": "float32",
                                            "encoding": "none",
                                            "shape": {}
                                        }
                                    }
                                }
                            },
                            "pythonClassName": "tensorflow_datasets.core.features.features_dict.FeaturesDict"
                        },
                        "length": "-1"
                    }
                }
            }
        },
        "pythonClassName": "tensorflow_datasets.core.features.features_dict.FeaturesDict"
    }
    
    # Fill in action_dict features from sample episode
    action_dict_features = feature_schema["featuresDict"]["features"]["steps"]["sequence"]["feature"]["featuresDict"]["features"]["action_dict"]["featuresDict"]["features"]
    
    for key, value in sample_episode['steps']['action_dict'].items():
        if len(value.shape) == 2:
            dimensions = [str(value.shape[1])]
        elif len(value.shape) == 1:
            dimensions = ["1"]
        else:
            dimensions = []
            
        action_dict_features[key] = {
            "description": f"Commanded {key}",
            "pythonClassName": "tensorflow_datasets.core.features.tensor_feature.Tensor",
            "tensor": {
                "dtype": "float64",
                "encoding": "none",
                "shape": {
                    "dimensions": dimensions
                }
            }
        }
    
    # Fill in observation features from sample episode
    obs_features = feature_schema["featuresDict"]["features"]["steps"]["sequence"]["feature"]["featuresDict"]["features"]["observation"]["featuresDict"]["features"]
    
    for key, value in sample_episode['steps']['observation'].items():
        if "image" in key:
            # For images
            obs_features[key] = {
                "description": f"{key.replace('_', ' ').title()}",
                "image": {
                    "dtype": "uint8",
                    "encodingFormat": "jpeg",
                    "shape": {
                        "dimensions": ["240", "320", "3"]
                    }
                },
                "pythonClassName": "tensorflow_datasets.core.features.image_feature.Image"
            }
        else:
            # For tensors
            if len(value.shape) == 2:
                dimensions = [str(value.shape[1])]
            elif len(value.shape) == 1:
                dimensions = ["1"]
            else:
                dimensions = []
                
            obs_features[key] = {
                "description": f"{key.replace('_', ' ').title()} state",
                "pythonClassName": "tensorflow_datasets.core.features.tensor_feature.Tensor",
                "tensor": {
                    "dtype": "float64",
                    "encoding": "none",
                    "shape": {
                        "dimensions": dimensions
                    }
                }
            }
    
    # Process episodes
    for episode in episodes:
        # Process each step in the episode
        num_steps = len(episode['steps']['reward']) - 1  # -1 because last step has no action
        
        for step_idx in range(num_steps):
            features = {}
            
            # Episode metadata (same for all steps in an episode)
            features['episode_metadata/file_path'] = _bytes_feature(episode['metadata']['file_path'])
            features['episode_metadata/recording_folderpath'] = _bytes_feature(episode['metadata']['recording_folderpath'])
            
            # Step metadata
            features['steps/discount'] = _float_feature(float(episode['steps']['discount'][step_idx]))
            features['steps/is_first'] = _bool_feature(bool(episode['steps']['is_first'][step_idx]))
            features['steps/is_last'] = _bool_feature(bool(episode['steps']['is_last'][step_idx]))
            features['steps/is_terminal'] = _bool_feature(bool(episode['steps']['is_terminal'][step_idx]))
            features['steps/reward'] = _float_feature(float(episode['steps']['reward'][step_idx]))
            
            # Language instructions
            features['steps/language_instruction'] = _bytes_feature(episode['steps']['language_instruction'][step_idx])
            features['steps/language_instruction_2'] = _bytes_feature(episode['steps']['language_instruction_2'][step_idx])
            features['steps/language_instruction_3'] = _bytes_feature(episode['steps']['language_instruction_3'][step_idx])
            
            # Action
            features['steps/action'] = _float_array_feature(episode['steps']['action'][step_idx])
            
            # Action dictionary
            for key, value in episode['steps']['action_dict'].items():
                if len(value.shape) == 2:
                    features[f'steps/action_dict/{key}'] = _float_array_feature(value[step_idx])
                else:
                    # Handle scalar or 1D array
                    features[f'steps/action_dict/{key}'] = _float_array_feature(value[step_idx])
            
            # Observations
            for key, value in episode['steps']['observation'].items():
                if "image" in key:
                    # Get the image for this step
                    image = value[step_idx]
                    
                    # Resize to match the expected dimensions (240, 320, 3)
                    if image.shape != (240, 320, 3):
                        image = tf.image.resize(image, [240, 320]).numpy().astype(np.uint8)
                    
                    # Encode as JPEG
                    encoded_image = tf.io.encode_jpeg(image, quality=95).numpy()
                    features[f'steps/observation/{key}'] = _bytes_feature(encoded_image)
                else:
                    # Handle tensor observations
                    features[f'steps/observation/{key}'] = _float_array_feature(value[step_idx])
            
            # Create and write the example
            example = tf.train.Example(features=tf.train.Features(feature=features))
            serialized_example = example.SerializeToString()
            example_size = len(serialized_example)  # Get size for tracking total bytes
            
            shard_id = total_step_count % num_shards
            writers[shard_id].write(serialized_example)
            
            # Update shard lengths
            shard_lengths[shard_id] += 1
            
            # Update counters
            total_step_count += 1
            total_bytes += example_size
        
        episode_count += 1
        if episode_count % 10 == 0:  # Progress update every 10 episodes
            print(f"Processed {episode_count}/{len(episodes)} episodes, {total_step_count} steps")
    
    # Close all writers
    for writer in writers:
        writer.close()
    
    # Create features.json containing the schema
    with open(os.path.join(dataset_dir, "features.json"), "w") as f:
        json.dump(feature_schema, f, indent=2)
    
    # Create dataset_metadata.json
    dataset_metadata = {
        "fileFormat": "tfrecord",
        "moduleName": "__main__",
        "name": dataset_name,
        "releaseNotes": {
            "1.0.0": "Initial release."
        },
        "splits": [
            {
                "filepathTemplate": "{DATASET}-{SPLIT}.{FILEFORMAT}-{SHARD_X_OF_Y}",
                "name": "train",
                "numBytes": str(total_bytes),
                "shardLengths": [str(length) for length in shard_lengths]
            }
        ],
        "version": "1.0.0"
    }
    
    # Save dataset_metadata.json as dataset_info.json
    with open(os.path.join(dataset_dir, "dataset_info.json"), "w") as f:
        json.dump(dataset_metadata, f, indent=2)
    
    print(f"Successfully saved {episode_count} episodes ({total_step_count} steps) to {dataset_dir}")
    print(f"Dataset created with {num_shards} shards")
    print(f"Total dataset size: {total_bytes/1024/1024:.2f} MB")
    
    return dataset_dir


def save_nested_dict_to_hdf5(group: h5py.Group, data: dict) -> None:
    """
    Recursively save nested dictionary to HDF5 group.
    
    Args:
        group: HDF5 group to save data to
        data: Dictionary of data to save
    """
    for key, value in data.items():
        if isinstance(value, dict):
            subgroup = group.create_group(key)
            save_nested_dict_to_hdf5(subgroup, value)
        else:
            try:
                group.create_dataset(key, data=value, compression="gzip")
            except TypeError as e:
                raise TypeError(f"Could not save data for key '{key}'. Data type not supported: {type(value)}")


def save_data(data, filename: str) -> None:
    """
    Save data to an HDF5 file.
    
    Args:
        data: Data to save
        filename: Path to save the HDF5 file
    """
    try:
        with h5py.File(filename, 'w') as f:
            if isinstance(data, dict):
                save_nested_dict_to_hdf5(f, data)
            else:
                f.create_dataset('data', data=data, compression="gzip")
    except Exception as e:
        print(f"Error saving file {filename}: {e}")
        raise


def find_timestamps_between(timestamps, start_time, end_time):
    """
    Find indices of timestamps that fall between start_time and end_time (inclusive)
    
    Args:
        timestamps: List of float timestamps
        start_time: Float start time
        end_time: Float end time
    
    Returns:
        List of indices where timestamps fall between start and end time
    """
    return [i for i, t in enumerate(timestamps) if start_time <= t <= end_time]


def find_closest_timestamps_numpy(timestamps1, timestamps2):
    """
    Find the indices in timestamps2 that are closest to each timestamp in timestamps1.
    Uses vectorized numpy operations for efficiency.
    
    Args:
        timestamps1: First array of timestamps
        timestamps2: Second array of timestamps to find closest matches in
    
    Returns:
        Array of indices in timestamps2 that best match each timestamp in timestamps1
    """
    timestamps1 = np.array(timestamps1)
    timestamps2 = np.array(timestamps2)
    
    # Calculate all pairwise differences using broadcasting
    differences = np.abs(timestamps1[:, np.newaxis] - timestamps2)
    
    # Find index of minimum difference for each timestamp
    closest_indices = np.argmin(differences, axis=1)
    
    return closest_indices


def convert_quaternion_to_euler(trajectory):
    """
    Convert a trajectory with quaternion rotations to Euler angles.
    
    Args:
        trajectory: numpy array of shape (T, 7) with each row being [x, y, z, qw, qx, qy, qz]
    
    Returns:
        numpy array of shape (T, 6) with each row being [x, y, z, roll, pitch, yaw]
        where roll, pitch, yaw are Euler angles in radians
    """
    # Initialize output array
    T = trajectory.shape[0]
    result = np.zeros((T, 6))
    
    # Copy position data (x, y, z)
    result[:, 0:3] = trajectory[:, 0:3]
    
    # Convert quaternions to Euler angles
    for i in range(T):
        # Get quaternion (note: scipy uses [x, y, z, w] order, but our data is [w, x, y, z])
        quat = trajectory[i, 3:7]
        # Reorder from [qw, qx, qy, qz] to [qx, qy, qz, qw] for scipy
        quat_scipy = np.array([quat[1], quat[2], quat[3], quat[0]])
        
        # Create rotation object and convert to Euler angles (using 'xyz' convention)
        rot = R.from_quat(quat_scipy)
        euler = rot.as_euler('xyz', degrees=False)
        
        # Store Euler angles
        result[i, 3:6] = euler
    
    return result


def process_h5_file(h5_file_path, target_action="gear", downsample_ratio=1, verbose=False):
    """
    Process a single HDF5 file to extract robot demonstrations.
    
    Args:
        h5_file_path: Path to the HDF5 file
        target_action: String to match in the segment text to identify relevant segments
        downsample_ratio: Ratio to downsample timestamps (e.g., 2 means keep every 2nd frame)
        verbose: Whether to print verbose output
    
    Returns:
        List of episode dictionaries
    """
    if verbose:
        print(f"Processing file: {h5_file_path}")
    
    data_name = os.path.split(h5_file_path)[-1]
    data_name = os.path.splitext(data_name)[0]

    # Load the HDF5 file
    data = load_h5_file(h5_file_path, decode=True)

    # List to store all actions extracted from this file
    all_actions = []

    # Iterate through segments in the file
    for k, v in data["segments_info"].items():
        segment_text = v["text"].decode("utf-8")
        
        # Check if this segment matches our target action
        if target_action in segment_text:
            start = float(v["start"])
            end = float(v["end"])
            success = bool(v["success"])

            # Only process successful demonstrations
            if success:
                # Skip if hand camera data is missing
                if "hand" not in data.keys():
                    if verbose:
                        print(f"No hand camera data in {data_name}")
                    continue

                # Find timestamps within the segment timeframe
                indexes = find_timestamps_between(data["timestamps"]["hand"], start, end)
                
                # Apply downsampling if requested
                if downsample_ratio > 1:
                    indexes = indexes[::downsample_ratio]
                
                # Skip if no frames are found in the segment
                if not indexes:
                    continue
                
                # Get hand camera frames and timestamps
                frames_hand = data["hand"][indexes]
                hand_stamps = data["timestamps"]["hand"][indexes]
                frames_hand = tf.image.resize(frames_hand, (240, 320), method='bilinear').numpy()

                # Get corresponding frames from other cameras by finding closest timestamps
                hama1_index = find_closest_timestamps_numpy(hand_stamps, data["timestamps"]["hama1"])
                frames_hama1 = data["hama1"][hama1_index]
                frames_hama1 = tf.image.resize(frames_hama1, (240, 320), method='bilinear').numpy()

                hama2_index = find_closest_timestamps_numpy(hand_stamps, data["timestamps"]["hama2"])
                frames_hama2 = data["hama2"][hama2_index]
                frames_hama2 = tf.image.resize(frames_hama2, (240, 320), method='bilinear').numpy()

                # Get robot state data aligned with hand camera timestamps
                # Joint positions
                qpos_index = find_closest_timestamps_numpy(hand_stamps, data["timestamps"]["joint_positions"])
                qpos = data["robot_state"]["joint_positions"][qpos_index]

                # Joint velocities
                qvel_index = find_closest_timestamps_numpy(hand_stamps, data["timestamps"]["joint_velocities"])
                qvel = data["robot_state"]["joint_velocities"][qvel_index]

                # Gripper positions
                gripper_index = find_closest_timestamps_numpy(hand_stamps, data["timestamps"]["gripper_positions"])
                gripper = data["robot_state"]["gripper_positions"][gripper_index, 0]

                # Gripper velocities
                gripper_v_index = find_closest_timestamps_numpy(hand_stamps, data["timestamps"]["gripper_velocities"])
                gripper_v = data["robot_state"]["gripper_velocities"][gripper_v_index, 0]

                # Robot pose (position and orientation)
                pose_index = find_closest_timestamps_numpy(hand_stamps, data["timestamps"]["pose"])
                pose = data["robot_state"]["pose"][pose_index]
                pose = convert_quaternion_to_euler(pose)  # Convert quaternions to Euler angles

                # Robot velocity
                vel_index = find_closest_timestamps_numpy(hand_stamps, data["timestamps"]["velocity"])
                vel = data["robot_state"]["velocity"][vel_index]

                # Create episode markers
                is_first = np.zeros(len(pose))
                is_first[0] = 1  # Mark first step

                is_last = np.zeros(len(pose))
                is_last[-1] = 1  # Mark last step

                is_terminal = np.zeros(len(pose))
                is_terminal[-1] = 1  # Mark terminal step

                reward = np.zeros(len(pose))
                reward[-1] = 1  # Reward at the final step

                discount = np.ones(len(pose))  # Discount factor is 1 everywhere

                # Language instruction (repeated for all steps)
                language = [segment_text] * len(pose)

                # Actions are the next pose and gripper position
                # We use pose[1:] (shifted by 1) since actions lead to the next state
                action = np.concatenate([pose[1:], gripper[1:][..., np.newaxis]], axis=-1)

                # Assemble the episode data structure
                steps = {
                    'action': action, 
                    'action_dict': {
                        'cartesian_position': pose[1:],
                        'cartesian_velocity': vel[1:],
                        'gripper_position': gripper[1:],
                        'gripper_velocity': gripper_v[1:],
                        'joint_position': qpos[1:],
                        'joint_velocity': qvel[1:]
                    },
                    'discount': discount,
                    'is_first': is_first,
                    'is_last': is_last,
                    'is_terminal': is_terminal,
                    'language_instruction': language,
                    'language_instruction_2': language,
                    'language_instruction_3': language,
                    'observation': {
                        'cartesian_position': pose[:-1],  # Exclude last state as it has no following action
                        'exterior_image_1_left': frames_hama1[:-1],
                        'exterior_image_2_left': frames_hama2[:-1],
                        'gripper_position': gripper[:-1],
                        'joint_position': qpos[:-1],
                        'wrist_image_left': frames_hand[:-1]
                    },
                    'reward': reward
                }

                # Add file metadata
                all_actions.append({
                    'steps': steps, 
                    'metadata': {
                        'file_path': h5_file_path,
                        'recording_folderpath': h5_file_path
                    }
                })

    return all_actions

def print_episode_structure(episode_dict, max_depth=5, current_depth=0, prefix=''):
    """
    Print the structure of an episode dictionary with appropriate indentation.
    
    Args:
        episode_dict: The episode dictionary to print
        max_depth: Maximum depth to print (to avoid too much detail)
        current_depth: Current depth in the recursive printing
        prefix: Prefix for the current line (for indentation)
    """
    if current_depth > max_depth:
        print(f"{prefix}... (max depth reached)")
        return
        
    if isinstance(episode_dict, dict):
        print(f"{prefix}{{")
        for key, value in episode_dict.items():
            if isinstance(value, (dict, list)):
                print(f"{prefix}  '{key}': ", end='')
                print_episode_structure(value, max_depth, current_depth + 1, prefix + '  ')
            elif isinstance(value, np.ndarray):
                print(f"{prefix}  '{key}': ndarray(shape={value.shape}, dtype={value.dtype})")
            elif isinstance(value, bytes):
                print(f"{prefix}  '{key}': bytes(length={len(value)})")
            elif isinstance(value, tf.Tensor):
                print(f"{prefix}  '{key}': Tensor(shape={value.shape}, dtype={value.dtype})")
            else:
                print(f"{prefix}  '{key}': {type(value).__name__}({value})")
        print(f"{prefix}}}")
    elif isinstance(episode_dict, list):
        if len(episode_dict) == 0:
            print(f"{prefix}[]")
            return
            
        print(f"{prefix}[")
        # Print just the first and last elements if the list is too long
        if len(episode_dict) > 3:
            # First element
            print(f"{prefix}  [0]: ", end='')
            print_episode_structure(episode_dict[0], max_depth, current_depth + 1, prefix + '  ')
            print(f"{prefix}  ... ({len(episode_dict) - 2} more items)")
            # Last element
            print(f"{prefix}  [{len(episode_dict) - 1}]: ", end='')
            print_episode_structure(episode_dict[-1], max_depth, current_depth + 1, prefix + '  ')
        else:
            for i, item in enumerate(episode_dict):
                print(f"{prefix}  [{i}]: ", end='')
                print_episode_structure(item, max_depth, current_depth + 1, prefix + '  ')
        print(f"{prefix}]")
    else:
        print(f"{prefix}{type(episode_dict).__name__}({episode_dict})")


def main():
    """
    Main function to run the RLDS data conversion process.
    """
    # Parse command line arguments
    args = parse_arguments()
    
    # Get H5 file paths from the input directory
    h5_paths = glob.glob(os.path.join(args.input_dir, "*.h5"))
    
    # Limit number of files if requested
    if args.max_files is not None:
        h5_paths = h5_paths[:args.max_files]
    
    print(f"Found {len(h5_paths)} HDF5 files to process")
    
    # Process files
    if args.multiprocess:
        # Multi-processing mode
        num_processes = args.num_processes or mp.cpu_count()
        print(f"Using multiprocessing with {num_processes} processes")
        
        with mp.Pool(processes=num_processes) as pool:
            # Process files in parallel with fixed arguments
            process_func = lambda file_path: process_h5_file(
                file_path, 
                args.target_action, 
                args.downsample_ratio,
                args.verbose
            )
            
            # Using imap_unordered to maintain a progress bar while processing
            results = [
                result for result in tqdm.tqdm(
                    pool.imap_unordered(process_func, h5_paths),
                    total=len(h5_paths),
                    desc="Processing files",
                    unit="file"
                )
            ]
    else:
        # Single-process mode
        print("Using single process mode")
        results = []
        for h5_file_path in tqdm.tqdm(h5_paths, desc="Processing files", unit="file"):
            result = process_h5_file(
                h5_file_path, 
                args.target_action, 
                args.downsample_ratio,
                args.verbose
            )
            results.append(result)
    
    # Flatten the list of lists
    episodes_list = [item for sublist in results for item in sublist]
    
    if not episodes_list:
        print("No episodes found matching the target action criteria")
        return
    
    print(f"Extracted {len(episodes_list)} episodes total")
    
    # Create the output directory if it doesn't exist
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Save the episodes to a TFDS dataset
    save_as_tfds_dataset(
        episodes_list, 
        args.output_dir, 
        args.dataset_name, 
        args.num_shards
    )
    
    print(f"Conversion complete. Dataset saved to: {os.path.join(args.output_dir, args.dataset_name)}")


if __name__ == "__main__":
    main()