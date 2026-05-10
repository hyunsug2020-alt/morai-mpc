#!/bin/bash
# MORAI TF noise stderr filter — TF 관련 ERROR/WARN spam만 제거, INFO는 통과
exec "$@" 2> >(grep --line-buffered -vE "child_frame_id not set|TF_REPEATED_DATA|Ignoring transform from authority|frame_ids cannot be empty|canTransform argument" >&2)
