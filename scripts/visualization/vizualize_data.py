"""
Multi-modal Data Visualization using Rerun SDK.
This script processes and visualizes multi-sensor data including:
- Video from multiple cameras
- Event camera data
- Audio streams
- Robot state information
- High and low-level action annotations
"""

import numpy as np
import rerun as rr
import argparse
import rerun.blueprint as rrb
import io
import shutil
from pydub import AudioSegment
import cv2
import tqdm
import imageio
import os
from REASSEMBLE.io import load_h5_file


def group_points_by_intervals(points, timestamps, interval_ms):
    """
    Group points into intervals based on timestamps and retain the last timestamp in each group.
    
    This function divides time into fixed intervals and groups data points that fall within each
    interval. This is especially useful for event-based cameras where events occur asynchronously.

    Args:
        points (np.ndarray): Array of points with shape (N, 3) where each point is [x, y, value].
        timestamps (np.ndarray): Array of timestamps for each point.
        interval_ms (float): Interval size in milliseconds.

    Returns:
        tuple: 
            - list: A list of arrays, where each array contains points in a specific interval.
            - list: A list of the last timestamp in each group.
    """
    # Find time range
    min_time = timestamps.min()
    max_time = timestamps.max()
    num_intervals = int(np.ceil((max_time - min_time) / interval_ms))
    
    # Sort timestamps and points for easier grouping
    sorted_indices = np.argsort(timestamps)
    timestamps_sorted = timestamps[sorted_indices]
    points_sorted = points[sorted_indices]
    
    groups = []
    last_timestamps = []

    # Group points by intervals
    for i in range(num_intervals):
        start = min_time + i * interval_ms
        end = start + interval_ms
        
        # Find points within current interval using binary search
        interval_start_idx = np.searchsorted(timestamps_sorted, start)
        interval_end_idx = np.searchsorted(timestamps_sorted, end)

        # Extract points in this interval
        grouped_points = points_sorted[interval_start_idx:interval_end_idx]
        groups.append(grouped_points)
        
        # Store the last timestamp in the group (for synchronization)
        if len(grouped_points) > 0:
            last_timestamps.append(timestamps_sorted[interval_end_idx - 1])
        else:
            # If no points in this interval, use None as placeholder
            last_timestamps.append(None)

    return groups, last_timestamps


def generate_event_video(groups, grid_size):
    """
    Generate visualization of event camera data as a video file.
    
    Event cameras capture asynchronous events (changes in pixel intensity).
    This function creates a visual representation where:
    - Green pixels represent positive events (brightness increase)
    - Red pixels represent negative events (brightness decrease)

    Args:
        groups (list): List of arrays, where each array contains points in a specific interval.
        grid_size (tuple): Size of the grid (height, width) for the output frames.
        
    Returns:
        str: Path to the generated video file.
    """
    output_path = "event_vid_temp.mp4"
    
    # Use imageio writer for more stable video generation
    with imageio.get_writer(
        output_path, format="mp4", mode="I", fps=30, macro_block_size=1
    ) as writer:      
        # Process each time interval
        for interval, points in tqdm.tqdm(enumerate(groups), total=len(groups), desc="Generating event frames"):
            # Create a black canvas for this frame
            canvas = np.zeros((grid_size[0], grid_size[1], 3), dtype=np.uint8)

            if len(points) > 0:
                # Convert to integer coordinates
                points = np.array(points, dtype=int)
                x, y, values = points[:, 0], points[:, 1], points[:, 2]

                # Color code the events:
                # Green for positive events (value == 1), Red for negative events (value != 1)
                canvas[y[values == 1], x[values == 1]] = [0, 255, 0]  # Green for positive events
                canvas[y[values != 1], x[values != 1]] = [255, 0, 0]  # Red for negative events
        
            writer.append_data(canvas)
            
    return output_path


def setup_visualization_blueprint():
    """
    Create a rerun visualization blueprint defining the layout of all views.
    
    Returns:
        rrb.Blueprint: The configured visualization layout.
    """
    # Create views for sensor data
    sensor_views = []
    for k in data["robot_state"].keys():
        sensor_views.append(rrb.TimeSeriesView(name=k, origin=f"/{k}"))

    # Define the overall layout with multiple panels
    blueprint = rrb.Blueprint(
        rrb.Vertical(
            # Camera views panel
            rrb.Horizontal(
                rrb.Spatial2DView(name="Cam1", origin="/hama1"),
                rrb.Spatial2DView(name="Cam2", origin="/hama2"),
                rrb.Spatial2DView(name="Cam3", origin="/hand"),
                column_shares=[3, 1],
                name="Cameras",
            ),
            # Event camera and text log panel
            rrb.Horizontal(
                rrb.TextLogView(origin="/High_level", name="High level Class"),
                rrb.TextLogView(origin="/Low_level", name="Low level Class"),
                rrb.Spatial2DView(name="Event Camera", origin="/capture_node-camera-image"),
                rrb.Spatial2DView(name="Events", origin="/event"),
                column_shares=[3, 1],
                name="Event Data",
            ),
            # Audio visualization panel
            rrb.Horizontal(
                rrb.TimeSeriesView(name="Hama1", origin=f"/hama1_audio"),
                rrb.TimeSeriesView(name="Hama2", origin=f"/hama2_audio"),
                rrb.TimeSeriesView(name="Hand",  origin=f"/hand_audio"),
                column_shares=[3, 1],
                name="Audio",
            ),
            # Sensor data grid
            rrb.Grid(
                *sensor_views,
                grid_columns=3,
                name="Internal Sensors"
            ),
        )
    )
    
    return blueprint


def process_video_data(data):
    """
    Process and log video data from multiple cameras.
    
    Args:
        data (dict): Dictionary containing video data and timestamps.
    """
    # List of camera sources to process
    cameras = ["hama1", "hama2", "hand", "capture_node-camera-image"]
    
    for cam in cameras:
        f_name = f"{cam}.mp4"
        
        # Write video data to file
        with open(f_name, "wb") as f:
            binary_stream = io.BytesIO(data[cam])
            shutil.copyfileobj(binary_stream, f)

        # Log video asset to Rerun
        video_asset = rr.AssetVideo(path=f_name)
        rr.log(f"{cam}", video_asset, static=True)

        # Extract frame timestamps for synchronization
        frame_timestamps_ns = video_asset.read_frame_timestamps_ns()
        
        # Send video frames with corresponding timestamps
        rr.send_columns(
            f"{cam}",
            # Map timeline values to video timestamps
            times=[rr.TimeSecondsColumn("timestamp", data["timestamps"][cam])],
            components=[
                rr.VideoFrameReference.indicator(), 
                rr.components.VideoTimestamp.nanoseconds(frame_timestamps_ns)
            ],
        )


def process_event_camera_data(data):
    """
    Process and visualize event camera data.
    
    Args:
        data (dict): Dictionary containing event data and timestamps.
        
    Returns:
        str: Path to the generated event video.
    """
    print("Processing event camera data...")
    
    # Group events by time intervals (33ms ~= 30fps)
    groups, last_timestamps = group_points_by_intervals(
        data["events"], 
        data["timestamps"]["events"], 
        0.033
    )
    
    # Generate video from event data
    # Specify grid size (height, width) based on event camera resolution
    event_vid_path = generate_event_video(groups, (260, 346))

    # Log the generated video to Rerun
    e_video_asset = rr.AssetVideo(path=event_vid_path)
    rr.log("event", e_video_asset, static=True)

    # Send video frames with corresponding timestamps
    e_frame_timestamps_ns = e_video_asset.read_frame_timestamps_ns()
    rr.send_columns(
        "event",
        times=[rr.TimeSecondsColumn("timestamp", last_timestamps)],
        components=[
            rr.VideoFrameReference.indicator(), 
            rr.components.VideoTimestamp.nanoseconds(e_frame_timestamps_ns)
        ],
    )
    
    return event_vid_path


def process_audio_data(data, audio_downsize_ratio):
    """
    Process and visualize audio data from multiple sources.
    
    Args:
        data (dict): Dictionary containing audio data.
        audio_downsize_ratio (int): Factor by which to downsample audio (for performance).
    """
    print("Processing audio data...")
    
    # Audio sources to process
    audio_keys = ["hand_audio", "hama1_audio", "hama2_audio"]
    
    for audio_key in audio_keys:
        # Extract audio data
        audio_data = data[audio_key]
        
        # Convert binary data to audio
        mp3_byte_data = bytearray(audio_data.tolist())
        mp3_io = io.BytesIO(mp3_byte_data)
        audio = AudioSegment.from_mp3(mp3_io)
        
        # Export for debugging (optional)
        audio.export(f"{audio_key}_test.wav", format="wav")
        
        # Extract audio samples
        # Note: hand_audio might have different channel configuration
        if audio_key != "hand_audio":
            samples = np.array(audio.get_array_of_samples())[::2]  # Take every other sample (one channel)
        else:
            samples = np.array(audio.get_array_of_samples())
        
        # Generate timestamps for each sample
        audio_ts = np.arange(0, len(samples) * (1/audio.frame_rate), (1/audio.frame_rate))
        
        # Downsample for visualization (improves performance)
        samples = samples[::audio_downsize_ratio]
        audio_ts = audio_ts[::audio_downsize_ratio]
        
        # Log audio samples to Rerun
        for s, t in zip(samples, audio_ts):
            rr.set_time_seconds("timestamp", t)
            rr.log(f"{audio_key}", rr.Scalar(s))


def process_robot_state(data, internal_downsize_ratio):
    """
    Process and visualize robot state data (joint positions, velocities, etc.).
    
    Args:
        data (dict): Dictionary containing robot state data.
        internal_downsize_ratio (int): Factor by which to downsample data (for performance).
    """
    print("Processing robot state data...")
    
    # Process each type of robot state data
    for data_key in data["robot_state"].keys():
        # Downsample data for visualization
        for i in range(0, len(data["timestamps"][data_key]), internal_downsize_ratio):
            # Set timestamp for this data point
            rr.set_time_seconds("timestamp", data["timestamps"][data_key][i])
            
            # Process different types of data based on their dimension
            values = data["robot_state"][data_key][i]
            
            # XYZ position (3D)
            if len(values) == 3:
                for _id, name in enumerate(["x", "y", "z"]):
                    rr.log(f"{data_key}/{name}", rr.Scalar(values[_id]))
            
            # Pose with position and quaternion (7D)
            elif len(values) == 7 and "joint" not in data_key:
                for _id, name in enumerate(["x", "y", "z", "qx", "qy", "qz", "qw"]):
                    rr.log(f"{data_key}/{name}", rr.Scalar(values[_id]))
            
            # Velocity with linear and angular components (6D)
            elif len(values) == 6:
                for _id, name in enumerate(["x", "y", "z", "wx", "wy", "wz"]):
                    rr.log(f"{data_key}/{name}", rr.Scalar(values[_id]))
            
            # Joint positions (7D)
            elif len(values) == 7 and "joint" in data_key:
                for _id in range(7):
                    rr.log(f"{data_key}/joint{_id}", rr.Scalar(values[_id]))
            
            # Gripper state (2D)
            elif len(values) == 2:
                for _id, name in enumerate(["finger1", "finger2"]):
                    rr.log(f"{data_key}/{name}", rr.Scalar(values[_id]))


def process_segments_info(data, ts_min):
    """
    Process and visualize action segments and their success/failure status.
    
    Args:
        data (dict): Dictionary containing segment information.
        ts_min (float): Minimum timestamp (for normalization).
    """
    print("Processing action segments...")
    
    # Process high-level action segments
    for value in data["segments_info"].values():
        # Normalize start time
        s_time = value["start"] - ts_min
        success = bool(value["success"])
        action = value["text"].decode("utf-8")
        
        # Log high-level action
        rr.set_time_seconds("timestamp", s_time)
        log_level = rr.TextLogLevel.INFO if success else rr.TextLogLevel.ERROR
        rr.log("High_level", rr.TextLog(
            f"Action: {action}, Success: {success}", 
            level=log_level
        ))
        
        # Process low-level action segments if available
        if "low_level" in value.keys():
            for value_low in value["low_level"].values():
                s_time_low = value_low["start"] - ts_min
                success_low = bool(value_low["success"])
                action_low = value_low["text"].decode("utf-8")
                
                # Log low-level action
                rr.set_time_seconds("timestamp", s_time_low)
                log_level = rr.TextLogLevel.INFO if success_low else rr.TextLogLevel.ERROR
                rr.log("Low_level", rr.TextLog(
                    f"Action: {action_low}, Success: {success_low}", 
                    level=log_level
                ))


def cleanup_temp_files(file_paths):
    """
    Clean up temporary files created during visualization.
    
    Args:
        file_paths (list): List of file paths to clean up.
    """
    for path in file_paths:
        if os.path.exists(path):
            try:
                os.remove(path)
                print(f"Removed temporary file: {path}")
            except Exception as e:
                print(f"Error removing {path}: {e}")


def main():
    """Main function to process and visualize data."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="Visualize multi-modal sensor data using Rerun SDK"
    )
    parser.add_argument("file", type=str, help="Path to h5 file which should be visualized")
    parser.add_argument("--ra", type=int, help="Downsize ratio for audio data", default=100)
    parser.add_argument("--ri", type=int, help="Downsize ratio for internal data", default=10)
    parser.add_argument("--cleanup", action="store_true", help="Clean up temporary files after visualization")
    rr.script_add_args(parser)
    args = parser.parse_args()

    # Load data from H5 file
    global data  # Make data accessible to blueprint setup function
    data = load_h5_file(args.file, decode=False)
    audio_downsize_ratio = args.ra
    internal_downsize_ratio = args.ri

    # Initialize the minimum timestamp for normalization
    ts_min = np.inf
    for k, v in data["timestamps"].items():
        if v[0] < ts_min:
            ts_min = v[0]

    # Normalize timestamps to start from 0
    for k, v in data["timestamps"].items():
        data["timestamps"][k] = data["timestamps"][k] - ts_min

    # Set up visualization layout
    blueprint = setup_visualization_blueprint()
    
    # Initialize Rerun SDK
    rr.script_setup(args, "multi_modal_visualization", default_blueprint=blueprint)
    
    # Track temporary files for cleanup
    temp_files = []

    try:
        # Process all data types
        process_video_data(data)
        
        event_vid_path = process_event_camera_data(data)
        temp_files.append(event_vid_path)
        
        process_audio_data(data, audio_downsize_ratio)
        process_robot_state(data, internal_downsize_ratio)
        process_segments_info(data, ts_min)
        
        print("Visualization complete! Use the Rerun viewer to explore the data.")
        
    finally:
        # Clean up temporary files if requested
        if args.cleanup:
            # Add camera videos to cleanup list
            for cam in ["hama1", "hama2", "hand", "capture_node-camera-image"]:
                temp_files.append(f"{cam}.mp4")
            
            # Add audio files
            for audio_key in ["hand_audio", "hama1_audio", "hama2_audio"]:
                temp_files.append(f"{audio_key}_test.wav")
                
            cleanup_temp_files(temp_files)


if __name__ == "__main__":
    main()