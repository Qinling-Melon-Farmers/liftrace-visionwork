#gnome-terminal -x bash -c "roscore"
#!/bin/bash 

tmux kill-session -t QRcode
tmux new-session -d -s QRcode  ;

tmux split-window -h 
tmux select-pane -t 0

tmux select-pane -t 0
# 1cmd
# camera
tmux send-keys "qr" C-m
tmux send-keys "cd ~/detect_ws/ && source ~/detect_ws/devel/setup.bash" C-m
tmux send-keys "roslaunch detect_pkg camera.launch" C-m

tmux select-pane -t 1
# 2cmd
tmux send-keys "sleep 2s" C-m 
# qrcode
tmux send-keys "qr" C-m
tmux send-keys "cd ~/detect_ws/ && source ~/detect_ws/devel/setup.bash" C-m
tmux send-keys "roslaunch detect_pkg qrcode.launch" C-m

tmux -2 attach-session -t QRcode
