tmux kill-session -t session4
tmux new-session -d -s session4

#  0 | 2
#  1 | 2
tmux select-pane -t 0

tmux send-keys "sudo su" C-m 
tmux send-keys "source /home/orangepi/patrol_uav_ws-patrol_planner/devel/setup.bash && source /home/orangepi/.bashrc" C-m
tmux send-keys "roslaunch actuator_pwm launch_all.launch" C-m


tmux -2 attach-session -t session4
