tmux kill-session -t build
tmux new-session -d -s build

tmux split-window -h 

tmux select-pane -t 0
tmux send-keys "cd patrol_uav_ws-patrol_planner && find . -type f -exec touch {} +" C-m 

tmux select-pane -t 1
tmux send-keys "catkin_make -DROS_EDITION=ROS1" C-m 

tmux -2 attach-session -t build
