import numpy as np
import os
import rospkg
import pandas as pd
from bagpy import bagreader
import scipy.spatial.transform
import matplotlib.pyplot as plt

def load_bag_with_time(path):
    print("loading bag file...")
    bag = bagreader(path)
    print("pd.read_csv...")
    # wrench_df = pd.read_csv(bag.message_by_topic('/franka_state_controller/F_ext_base'))
    wrench_df = pd.read_csv(bag.message_by_topic('/ft_sensor/ft_compensated_base'))
    wrench = np.array([
        wrench_df['wrench.force.x'].values,
        wrench_df['wrench.force.y'].values,
        wrench_df['wrench.force.z'].values,
        wrench_df['wrench.torque.x'].values,
        wrench_df['wrench.torque.y'].values,
        wrench_df['wrench.torque.z'].values]).T
    
    franka_state_df = pd.read_csv(bag.message_by_topic('/reassemble_state_controller/franka_states'))
    
    # o_t_ee_matrices = [np.reshape(np.array(franka_state_df.loc[i, 'O_T_EE_0':'O_T_EE_15']), (4, 4)) for i in range(len(franka_state_df))]
    o_t_ee_matrices = np.array([np.array(franka_state_df.loc[i, 'O_T_EE_0':'O_T_EE_15']).reshape(4, 4).T for i in range(len(franka_state_df))])

    timestamps = franka_state_df["time"]
    timestamps = timestamps - timestamps[0]
    time_array = timestamps
    
    data = []

    # Downsample the larger dataset
    min_length = min(len(wrench), len(o_t_ee_matrices))
    if len(wrench) > min_length:
        indices = np.linspace(0, len(wrench) - 1, min_length).astype(int)
        wrench = wrench[indices]
    else:
        indices = np.linspace(0, len(o_t_ee_matrices) - 1, min_length).astype(int)
        o_t_ee_matrices = o_t_ee_matrices[indices]
        time_array = timestamps[indices]

    print("loop over pose data...")
    for i, o_t_ee in enumerate(o_t_ee_matrices):
        O_T_EE = o_t_ee.copy()
        # Extract position
        x, y, z = O_T_EE[0:3, 3]
        
        # Convert rotation matrix to quaternion
        rotation_matrix = O_T_EE[0:3, 0:3]
        quaternion = scipy.spatial.transform.Rotation.from_matrix(rotation_matrix).as_quat()
        
        # Append wrench data if available for this index
        if i < len(wrench):
            force_x, force_y, force_z, torque_x, torque_y, torque_z = wrench[i]
        else:
            # Default to zeros if wrench data is not available for this index
            force_x = force_y = force_z = torque_x = torque_y = torque_z = 0
        
        data.append([x, y, z, *quaternion, force_x, force_y, force_z, torque_x, torque_y, torque_z])
    
    return np.array(data), time_array

def visualize_forces(data, time, npy_path):
    force_x = data[:, 7]
    force_y = data[:, 8]
    force_z = data[:, 9]
    total_force = np.sqrt(force_x**2 + force_y**2 + force_z**2)
    
    plt.figure(figsize=(15, 10))
    
    plt.subplot(4, 1, 1)
    plt.plot(time, force_x, label='Force X')
    plt.xlabel('Time (s)')
    plt.ylabel('Force X (N)')
    plt.legend()
    
    plt.subplot(4, 1, 2)
    plt.plot(time, force_y, label='Force Y')
    plt.xlabel('Time (s)')
    plt.ylabel('Force Y (N)')
    plt.legend()
    
    plt.subplot(4, 1, 3)
    plt.plot(time, force_z, label='Force Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Force Z (N)')
    plt.legend()
    
    plt.subplot(4, 1, 4)
    plt.plot(time, total_force, label='Total Force')
    plt.xlabel('Time (s)')
    plt.ylabel('Total Force (N)')
    plt.legend()
    
    plt.tight_layout()
    plt.savefig(npy_path.replace('.npz', '_forces.png'))
    plt.close()

# Get the path of the ROS package
rospack = rospkg.RosPack()
package_path = rospack.get_path('haptic_ros')  # Replace 'your_package_name' with your actual package name

# Define the directories
bag_dir = os.path.join(package_path, 'recordings/')
np_dir = os.path.join(package_path, 'recordings/np/')

# Ensure npy directory exists
if not os.path.exists(np_dir):
    os.makedirs(np_dir)

# List all bag files in the bag directory
bag_files = [f for f in os.listdir(bag_dir) if f.endswith('.bag')]

for bag_file in bag_files:
    # Construct file paths
    bag_path = os.path.join(bag_dir, bag_file)
    npy_path = os.path.join(np_dir, bag_file.replace('.bag', '.npz'))

    # Check if npy file exists
    if not os.path.exists(npy_path):
        # Load bag file and save as npy
        data, time = load_bag_with_time(path=bag_path)
        
        np.savez(npy_path, data=data, time=time)
        print(f"Saved {npy_path}")

        # Visualize forces
        visualize_forces(data, time, npy_path)
    else:
        print(f"{npy_path} already exists.")
