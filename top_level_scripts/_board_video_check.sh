echo ===all_out_videos===
ls -la ~/board_eval/out_*.mp4 2>/dev/null
echo ===ffprobe_each===
for f in ~/board_eval/out_*.mp4; do
  echo "--- $(basename $f) ---"
  ffprobe -v error -show_entries format=duration,size -show_entries stream=codec_name,width,height -of default=noprint_wrappers=1 "$f" 2>&1 | head -6
done
