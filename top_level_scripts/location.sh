#gnome-terminal
#!/bin/bash 

# 先kill再new，可以避免重复启动脚本时乱套（乱分割窗口）
# 不同脚本的窗口名记得更改 【session1】
tmux kill-session -t session1
tmux new-session -d -s session1

#  0 | 2
#  1 | 2
tmux split-window -h 
tmux select-pane -t 0 
tmux split-window -v 
tmux select-pane -t 2 
tmux split-window -v 
tmux select-pane -t 0 
tmux split-window -v 

#
tmux select-pane -t 0 
tmux send-keys "sleep 13s; rostopic echo /mavros/local_position/pose" C-m

tmux select-pane -t 1
tmux send-keys "roslaunch mavros px4.launch" C-m 

#
tmux select-pane -t 2 
tmux send-keys "sleep 3s ; roslaunch livox_ros_driver2 msg_MID360.launch" C-m 


#
tmux select-pane -t 3 
tmux send-keys "sleep 6s ; roslaunch fast_lio mapping_mid360.launch" C-m 

tmux select-pane -t 4
tmux send-keys "sleep 10s; sudo script patrol_control.log" C-m 
tmux send-keys "cd patrol_uav_ws-patrol_planner && source devel/setup.bash" C-m 
tmux send-keys "roslaunch patrol_control patrol_control_real.launch" C-m 




tmux -2 attach-session -t session1 

########### 移动光标 ############
# tmux select-pane -U  # 光标切换到上方窗格
# tmux select-pane -D  # 光标切换到下方窗格
# tmux select-pane -L  # 光标切换到左边窗格
# tmux select-pane -R  # 光标切换到右边窗格
