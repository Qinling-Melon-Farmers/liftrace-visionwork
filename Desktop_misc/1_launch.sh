#gnome-terminal -x bash -c "roscore"
#!/bin/bash 

tmux kill-session -t 1_launch
tmux new-session -d -s 1_launch  ;

tmux split-window -h 

tmux select-pane -t 0
# 1cmd
# chmod pwm2
tmux send-keys "sudo chmod 666 /sys/class/pwm/pwmchip0/pwm0/*" C-m 

tmux select-pane -t 1
# 2cmd
# control3
tmux send-keys "cd ~/actuator/ && source ~/actuator/devel/setup.bash" C-m
tmux send-keys "roslaunch actuator_pwm control1.launch" C-m




tmux -2 attach-session -t 1_launch

