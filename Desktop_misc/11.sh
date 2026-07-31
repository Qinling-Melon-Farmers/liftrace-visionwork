#gnome-terminal -x bash -c "roscore"
#!/bin/bash 

tmux kill-session -t 11
tmux new-session -d -s 11  ;

tmux split-window -h 
tmux select-pane -t 0
tmux split-window -v 
tmux select-pane -t 2
tmux split-window -v 

tmux select-pane -t 0
# 1cmd
# mid-360
tmux send-keys "cd ~/patrol_uav_ws-patrol_planner/ && source ~/patrol_uav_ws-patrol_planner/devel/setup.bash" C-m
tmux send-keys "roslaunch livox_ros_driver2 msg_MID360.launch" C-m

tmux select-pane -t 1
# 2cmd
tmux send-keys "sleep 4s" C-m 
# fastlio
tmux send-keys "cd ~/patrol_uav_ws-patrol_planner/ && source ~/patrol_uav_ws-patrol_planner/devel/setup.bash" C-m
tmux send-keys "roslaunch fast_lio mapping_mid360.launch" C-m

tmux select-pane -t 2
# 3cmd
tmux send-keys "sleep 8s" C-m 
# mavros
tmux send-keys "roslaunch mavros px4.launch" C-m

tmux select-pane -t 3
# 4cmd
tmux send-keys "sleep 12s" C-m 
# pose
tmux send-keys "rostopic echo /mavros/local_position/pose" C-m

tmux -2 attach-session -t 11
