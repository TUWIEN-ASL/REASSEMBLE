# aidin_ROS

# FT_AIDIN.launch - Parameters
_sample_rate = sample rate of the FT sensor [Hz] (default 1000 Hz)
_cutoff_frequency = cutoff_frequency of low-pass filter [Hz] (default 35 Hz)
_arm_id = name of robot arm (default fr3)
_comm_type = use of CAN bus (can) or EtherCAT (ec) (default ec)
_ec_use_imu = use of sensor's IMU (default true)
_ec_interface = which network interface (default enp2s0)


# calibration procedure
1. roslaunch aidin_ros FT_AIDIN.launch arm_id:=panda ...
2. roslaunch force_torque_sensor_calib franka_calib.launch 

# use calibrated sensor
3. roslaunch aidin_ros ft_calibrated.launch arm_id:=fr3
5. use some controller with arm_id:=fr3 ...

# Requirements
1. https://github.com/orocos/soem
2. https://github.com/shadow-robot/ethercat_grant

# Adjustment of sensor pose
If the the mounting pose of the sensor with respect to NE frame changes adjust NE_T_Sensor in force_2_base.py accordingly. 