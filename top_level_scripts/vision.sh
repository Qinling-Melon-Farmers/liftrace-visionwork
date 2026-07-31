#gnome-terminal
#!/bin/bash 

# 先kill再new，可以避免重复启动脚本时乱套（乱分割窗口）
# 不同脚本的窗口名记得更改 【session1】
tmux kill-session -t session2
tmux new-session -d -s session2

#  0 | 2
#  1 | 2
tmux split-window -h 
tmux select-pane -t 0 
tmux split-window -v
tmux select-pane -t 2 
tmux split-window -v
tmux select-pane -t 0 
tmux split-window -v
tmux select-pane -t 2 
tmux split-window -v

#
tmux select-pane -t 0 
tmux send-keys "sudo su" C-m 
tmux send-keys "source /home/orangepi/patrol_uav_ws-patrol_planner/devel/setup.bash" C-m
tmux send-keys "roslaunch actuator_pwm launch_all.launch" C-m

tmux select-pane -t 1 
tmux send-keys "cd detect_ws && source devel/setup.bash && roslaunch camera_sdk camera_video0.launch" C-m 

#
tmux select-pane -t 2 
tmux send-keys "sleep 2s ; sudo script aligning.log" C-m 
tmux send-keys "sleep 3s ; cd patrol_uav_ws-patrol_planner && source devel/setup.bash" C-m 
tmux send-keys "sleep 4s ; roslaunch patrol_control circle_detection.launch" C-m 


tmux select-pane -t 3 
tmux send-keys "sleep 3s ; roslaunch patrol_control cross_detection.launch" C-m 

tmux select-pane -t 4 
tmux send-keys "sleep 4s ; cd Visual && source devel/setup.bash && roslaunch yolov5_detect detect.launch" C-m 

tmux select-pane -t 5 
tmux send-keys "sleep 5s ; roslaunch patrol_control landing_detector_node.launch" C-m  
#
tmux -2 attach-session -t session2

########### 移动光标 ############
# tmux select-pane -U  # 光标切换到上方窗格
# tmux select-pane -D  # 光标切换到下方窗格
# tmux select-pane -L  # 光标切换到左边窗格
# tmux select-pane -R  # 光标切换到右边窗格

