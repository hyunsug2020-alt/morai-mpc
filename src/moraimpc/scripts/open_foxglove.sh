#!/bin/bash
# foxglove_bridge가 뜰 때까지 대기 후 Foxglove Studio 자동 실행
sleep 3
foxglove-studio --url "foxglove://open?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A8765" &
