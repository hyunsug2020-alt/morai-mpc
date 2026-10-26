#!/usr/bin/env bash
# 자동 수집 10회차마다(회차번호가 10의 배수) 영상을 만들어 드라이브에 올림. 끄기: kill $(cat .../videos/watcher.pid)
D=$HOME/morai_competition/data/autodrive/videos; mkdir -p "$D"; echo $$ > "$D/watcher.pid"
L=$HOME/morai_competition/data/autodrive/auto_collect/runs.jsonl
tail -n0 -F "$L" | while IFS= read -r line; do
  sid=$(echo "$line" | python3 -c "import json,sys;print(json.loads(sys.stdin.read()).get('sid',''))" 2>/dev/null)
  case "$sid" in *[!0-9]*|'') continue;; esac
  if [ $((sid % 10)) -eq 0 ]; then
    echo "$(date +%F_%T) start $sid" >> "$D/watcher.log"
    bash $HOME/morai_competition/autodrive/make_run_video.sh "$sid" >> "$D/watcher.log" 2>&1
  fi
done
