import os
import h5py

valid_actions = {"Grasp", "Pull", "Release", "Push", "Twist", "Approach", "Align", "Lift", "Nudge"}

def check_low_level_action_names(h5_dir):
    h5_file_paths = [os.path.join(h5_dir, f) for f in os.listdir(h5_dir) if f.endswith('.h5')]

    for h5_file_path in h5_file_paths:
        try:
            with h5py.File(h5_file_path, 'r') as f:
                segments_info_group = f["segments_info"]
                
                for segment_idx, segment_group in segments_info_group.items():
                    if 'low_level' not in segment_group:
                        continue
                    
                    for sub_idx, subgroup in segment_group['low_level'].items():
                        action = subgroup['text'][()].decode('utf-8')
                        if action not in valid_actions:
                            print(f"{os.path.basename(h5_file_path)} - Segment {segment_idx}, Sub {sub_idx}: Unknown action '{action}'")
        except Exception as e:
            print(f"Could not open {os.path.basename(h5_file_path)}: {e}")




check_low_level_action_names('/home/dsliwowski/Projects/REASSEMBLE/REASSEMBLE/data/REASSEMBLE_corrected/data')

