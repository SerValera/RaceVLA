#!/bin/bash

tmux new-session -d -s flightsession

# initial window
tmux send-keys -t flightsession:0.0 "source /opt/ros/noetic/setup.bash && roscore" Enter

# first window
tmux new-window -n "Camera"
tmux split-window -v
tmux send-keys -t flightsession:1.0 "source /home/nuc/vlm_ws/devel/setup.bash && mon launch racing_vla rs_t265.launch" Enter
sleep 3
tmux send-keys -t flightsession:1.1 "rosbag record /t265/fisheye1/image_raw /vicon/drone_vla/drone_vla /mavros/vision_pose/pose /mavros/setpoint_velocity/cmd_vel /vicon/gate/gate"

# second window
tmux new-window -n "VIO"
tmux split-window -v
tmux send-keys -t flightsession:2.0 "source /home/nuc/openvins_ws/devel/setup.bash && mon launch /home/nuc/openvins_ws/src/open_vins/ov_msckf/launch/subscribe_with_transform.launch config:=rs_t265" Enter
tmux send-keys -t flightsession:2.1 "source /home/nuc/openvins_ws/devel/setup.bash && rosrun ov_msckf apply_tf_to_global.py" Enter
tmux split-window -v
tmux send-keys -t flightsession:2.2 "source /home/nuc/openvins_ws/devel/setup.bash && rosrun ov_msckf apply_ov_to_room.py" Enter
tmux split-window -h
tmux send-keys -t flightsession:2.3 "source /home/nuc/openvins_ws/devel/setup.bash && rosrun ov_msckf correct_tf_frame.py" Enter


# third window
tmux new-window -n "APM Initialize"
tmux send-keys -t flightsession:3.0 "source /home/nuc/vlm_ws/devel/setup.bash && mon launch racing_vla apm.launch" Enter
tmux split-window -v
tmux send-keys -t flightsession:3.1 "source /home/nuc/vlm_ws/devel/setup.bash && mon launch vlm_drone node.launch"

tmux bind-key -n S-Left select-pane -L
tmux bind-key -n S-Right select-pane -R
tmux bind-key -n S-Up select-pane -U
tmux bind-key -n S-Down select-pane -D
tmux bind-key -n M-S-Left previous-window
tmux bind-key -n M-S-Right next-window

tmux set-option -g mouse on
tmux set-option -g default-terminal "screen-256color"
tmux set-option -g terminal-overrides ',*:smcup@:rmcup@'
tmux set-option -g history-limit 10000

tmux attach-session -t flightsession
