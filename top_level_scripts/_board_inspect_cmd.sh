echo ===desktop_full===
ls ~/Desktop/
echo ===board_scripts===
find ~/Desktop ~/Documents ~/Downloads -maxdepth 2 -name "*.py" 2>/dev/null | head -12
echo ===cv2===
python3 -c 'import cv2; print(cv2.__version__)' 2>&1 | head -2
echo ===camera_grab===
python3 -c 'import cv2; cap=cv2.VideoCapture(0); ok,f=cap.read(); print("ok=",ok,"shape=",f.shape if ok else "none"); cap.release()' 2>&1 | head -5
echo ===flight_csv_head===
head -3 ~/Desktop/flight_20260803_143413.csv 2>/dev/null
