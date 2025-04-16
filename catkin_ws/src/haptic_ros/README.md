# haptic_ros
- haptic device Omega 3 & 6 ros driver
- franka teleoperation with force feedback

# Notes
for some reason in Simulation (compared to Real) the forces need to have negative sign.

# Start haptic device
roslaunch haptic_ros HapticDevice.launch force:=true centering:=false

# Start teleop
roslaunch haptic_ros TeleopFranka.launch use_sim:=false use_gripper:=false use_ft_sensor:=true record:=false haptic_z_rotation_deg:=-90.0