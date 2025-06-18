## How to use

To start the record manager, use:
```bash
roslaunch record_teleop record.launch
```

You can pass ```topic```, ```path_save``` and ```file_name``` to the launch file to specify topics to record, path on which to save all the recordings, and file_name (I suggest leaving file_name empty).

Adjusting path_save in launch file AS WELL AS in the ```record_manager.py``` script is important! Otherwise recordings may not be saved properly (you should get error notification in terminal in case this happens)

Current configuration is:
* 's' button is used to start/stop the recording
* 'd' button is uses as a segmentation button


# Recording data
0. Adapt the topics written in the launch file, and also the save path
1. roslaunch record_teleop record.launch
2. roslaunch realsense2_camera rs_camera.launch enable_depth:=false color_width:=640 color_height:=480 color_fps:=30
3. roslaunch haptic_ros HapticDevice.launch force:=true centering:=false
4. roslaunch haptic_ros TeleopFranka.launch use_sim:=false use_gripper:=true use_ft_sensor:=true record:=false haptic_z_rotation_deg:=180
5. select the terminal where you launched command #2

controlls:
s -- start recoredin / end segment sussecfully
d -- end segment in faliure
p -- end trail

* after pressing "s" or "d" speak out the action that you are performing, try to say it to the microphone, and be clear in your speach
* after pressing "p" you have to press "s" again to start the recording

* Reorgnaize the objects and board orientation every trail
* After 3-4 trials reorganize the camera positions
* Always be sure the objects are visible in the two external camera FOV

# Recordings
Each Person do 20, after each person try to move around the robot so the background is different
* Daniel --
* Sergej -- 
* Shail  --
* Johannes --
* Jedrzej --


