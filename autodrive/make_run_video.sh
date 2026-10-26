#!/usr/bin/env bash
# 회차 1개의 3카메라 4배속 영상을 만들어 드라이브 _videos 폴더에 올림. 사용: make_run_video.sh <회차번호>
set -u
S=$1; F=$HOME/morai_competition/data/autodrive/auto_collect/runs.jsonl
until grep -q "\"sid\": \"$S\"" "$F"; do sleep 5; done
R=/media/david/3C5CD1F35CD1A842/morai_collector_data/raw/$S/raw
O=$HOME/morai_competition/data/autodrive/videos/주행영상_${S}회차_4배속.mp4
mkdir -p "$(dirname "$O")"
cd "$R" || { echo "no raw $S"; exit 1; }
nice -n 19 ffmpeg -loglevel error -y -threads 2 \
  -framerate 48 -pattern_type glob -i 'left/rgb/*.jpg' \
  -framerate 48 -pattern_type glob -i 'front/rgb/*.jpg' \
  -framerate 48 -pattern_type glob -i 'right/rgb/*.jpg' \
  -filter_complex "[0]scale=640:360,setsar=1[l];[1]scale=640:360,setsar=1[f];[2]scale=640:360,setsar=1[r];[l][f][r]hstack=3,fps=24[v]" \
  -map "[v]" -shortest -c:v libx264 -preset veryfast -crf 26 -pix_fmt yuv420p "$O" || exit 1
$HOME/.local/bin/rclone copyto "$O" "gdrive:_videos/$(basename "$O")" --drive-root-folder-id 1t31n0ffY2MW4FD8vWt91zfV89jffXObm 2>&1 | grep -v client_id
echo "uploaded $S $(stat -c %s "$O")" && rm -f "$O"
