### Run camera
roslaunch racing_vla rs_t265.launch

### Run FCU
roslaunch racing_vla apm.launch

### Run VLA
rosrun vlm_drone drone.py 

### Recording experements
rosbag record /t265/fisheye1/image_raw /t265/fisheye1/camera_info /mavros/vision_pose/pose /mavros/setpoint_velocity/cmd_vel /vla_image /tf

### Recording experements (only traj and actions)
rosbag record /t265/fisheye1/image_raw /vicon/drone_vla/drone_vla /mavros/vision_pose/pose /mavros/setpoint_velocity/cmd_vel 

for bashrc 
alias vla_rs="roslaunch racing_vla rs_t265.launch"
alias vla_fcu="roslaunch racing_vla apm.launch"
alias vla_run="rosrun vlm_drone drone.py"

### All launch 
/vla_ws/src/vlm_drone/scripts/vla_start.sh