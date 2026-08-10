echo ===board_rknn_md5===
md5sum ~/Desktop/best-rk3588.rknn ~/Visual/src/yolov5_detect/best_rknn_model/best-rk3588.rknn ~/Visual/src/yolov5_detect/tank_rknn_model/best-rk3588.rknn 2>/dev/null
echo ===board_new_chain===
find ~ -maxdepth 4 -iname "*uav_vision*" -o -maxdepth 4 -iname "*rknn*eval*" -o -maxdepth 4 -iname "*board_rknn*" 2>/dev/null | grep -vE "build|devel" | head -10
echo ===board_eval_scripts===
find ~/Desktop ~/Documents ~/ -maxdepth 3 -name "*.py" 2>/dev/null | grep -iE "rknn|infer|eval" | grep -vE "build|devel|lib|site-packages" | head -12
echo ===board_display===
echo "DISPLAY=$DISPLAY"
xrandr 2>&1 | head -5
echo ===board_ultralytics===
python3 -c 'import ultralytics; print("ultralytics", ultralytics.__version__)' 2>&1 | head -2
echo ===board_rknn_py===
python3 -c 'from rknnlite.api import RKNNLite; print("rknnlite OK")' 2>&1 | head -2
