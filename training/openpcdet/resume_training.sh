#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
OPENPCDET_ROOT="$REPO_ROOT/third_party/OpenPCDet"
TOOLS_DIR="$OPENPCDET_ROOT/tools"
OUTPUT_DIR="$OPENPCDET_ROOT/output/custom_models/pointpillar_morai/morai_10k"
CKPT_DIR="$OUTPUT_DIR/ckpt"

BIOS_VERSION=$(tr -d '[:space:]' < /sys/class/dmi/id/bios_version)
if [[ ! $BIOS_VERSION =~ ^[0-9]+$ ]]; then
    printf '학습 중단: BIOS 버전을 판독할 수 없음: %s\n' "$BIOS_VERSION" >&2
    exit 2
fi

if ! nvidia-smi >/dev/null 2>&1; then
    printf '학습 중단: NVIDIA 드라이버가 정상 응답하지 않음\n' >&2
    exit 3
fi

NO_TURBO=$(tr -d '[:space:]' < /sys/devices/system/cpu/intel_pstate/no_turbo)
GPU_LIMIT=$(nvidia-smi --query-gpu=power.limit --format=csv,noheader,nounits | head -n 1 | tr -d '[:space:]')
if (( 10#$BIOS_VERSION < 2603 )); then
    if [[ $NO_TURBO != 1 ]] || ! awk -v limit="$GPU_LIMIT" 'BEGIN { exit !(limit <= 250.5) }'; then
        printf '학습 중단: 구형 BIOS에서는 CPU turbo=off, GPU limit<=250W가 필수임\n' >&2
        exit 2
    fi
    printf '경고: BIOS %s 임시 보호 모드로 학습함. BIOS 4003 업데이트가 여전히 필요함\n' "$BIOS_VERSION"
fi

CHECKPOINT=$(
    find "$CKPT_DIR" -maxdepth 1 -type f -name 'checkpoint_epoch_*.pth' -printf '%f\n' |
        sort -V |
        tail -n 1
)
if [[ -z $CHECKPOINT ]]; then
    printf '학습 중단: 완료된 epoch 체크포인트가 없음\n' >&2
    exit 4
fi
CHECKPOINT="$CKPT_DIR/$CHECKPOINT"

export CUDA_VISIBLE_DEVICES=0
export CUDA_MODULE_LOADING=LAZY
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export OPENBLAS_NUM_THREADS=4
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128
export LD_LIBRARY_PATH="/usr/local/cuda-11.8/lib64:${LD_LIBRARY_PATH:-}"

mkdir -p "$OUTPUT_DIR"
RUN_LOG="$OUTPUT_DIR/validated_resume_$(date +%Y%m%d-%H%M%S).out"
HEALTH_LOG="$OUTPUT_DIR/health_watchdog_$(date +%Y%m%d-%H%M%S).log"
printf 'BIOS=%s checkpoint=%s log=%s health_log=%s\n' \
    "$BIOS_VERSION" "$CHECKPOINT" "$RUN_LOG" "$HEALTH_LOG"

cd "$TOOLS_DIR"
"$REPO_ROOT/.venv-openpcdet/bin/python" train.py \
    --cfg_file "$SCRIPT_DIR/pointpillar_morai.yaml" \
    --extra_tag morai_10k \
    --batch_size 2 \
    --workers 0 \
    --fix_random_seed \
    --num_epochs_to_eval 5 \
    --ckpt "$CHECKPOINT" \
    --ckpt_save_interval 1 \
    --ckpt_save_time_interval 180 \
    --max_ckpt_save_num 30 \
    --wo_gpu_stat \
    > >(tee -a "$RUN_LOG") 2>&1 &
TRAIN_PID=$!
START_TIME=$(date --iso-8601=seconds)
STOP_REASON=
LAST_HEARTBEAT=0

stop_training() {
    local reason=$1
    printf '%s WATCHDOG STOP: %s\n' "$(date --iso-8601=seconds)" "$reason" | tee -a "$HEALTH_LOG"
    kill -INT "$TRAIN_PID" 2>/dev/null || true
    for _ in $(seq 1 40); do
        kill -0 "$TRAIN_PID" 2>/dev/null || return 0
        sleep 0.5
    done
    kill -TERM "$TRAIN_PID" 2>/dev/null || true
}

trap 'stop_training "service termination requested"' INT TERM

while kill -0 "$TRAIN_PID" 2>/dev/null; do
    if ! GPU_ROW=$(nvidia-smi \
        --query-gpu=temperature.gpu,power.draw,power.limit,memory.used \
        --format=csv,noheader,nounits 2>/dev/null); then
        STOP_REASON='nvidia-smi stopped responding'
    else
        IFS=',' read -r GPU_TEMP GPU_POWER GPU_POWER_LIMIT GPU_MEMORY <<< "$GPU_ROW"
        GPU_TEMP=${GPU_TEMP//[[:space:]]/}
        GPU_POWER=${GPU_POWER//[[:space:]]/}
        GPU_POWER_LIMIT=${GPU_POWER_LIMIT//[[:space:]]/}
        GPU_MEMORY=${GPU_MEMORY//[[:space:]]/}
    fi

    CPU_TEMP=$(sensors 2>/dev/null | awk '/Package id 0/ {gsub(/[+°C]/, "", $4); print int($4); exit}')
    CPU_TEMP=${CPU_TEMP:-0}
    MEM_AVAILABLE_KIB=$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)
    DISK_AVAILABLE_KIB=$(df -Pk "$OUTPUT_DIR" | awk 'NR == 2 {print $4}')

    if [[ -z $STOP_REASON ]] && awk -v temp="$GPU_TEMP" 'BEGIN { exit !(temp >= 80) }'; then
        STOP_REASON="GPU temperature ${GPU_TEMP}C reached the 80C stop threshold"
    elif [[ -z $STOP_REASON ]] && (( CPU_TEMP >= 85 )); then
        STOP_REASON="CPU temperature ${CPU_TEMP}C reached the 85C stop threshold"
    elif [[ -z $STOP_REASON ]] && (( MEM_AVAILABLE_KIB < 3145728 )); then
        STOP_REASON="available memory fell below 3 GiB"
    elif [[ -z $STOP_REASON ]] && (( DISK_AVAILABLE_KIB < 5242880 )); then
        STOP_REASON="available disk space fell below 5 GiB"
    elif [[ -z $STOP_REASON ]] && /usr/bin/journalctl -k --since "$START_TIME" --no-pager 2>/dev/null |
        /bin/grep -Ei 'NVRM: Xid|machine check|hardware error|out of memory|killed process|invalid opcode|watchdog.*lockup' >/dev/null; then
        STOP_REASON='kernel reported a GPU, CPU, memory, or lockup error'
    fi

    if [[ -n $STOP_REASON ]]; then
        stop_training "$STOP_REASON"
        break
    fi

    NOW=$(date +%s)
    if (( NOW - LAST_HEARTBEAT >= 60 )); then
        printf '%s WATCHDOG OK gpu=%sC/%sW limit=%sW vram=%sMiB cpu=%sC mem=%sKiB disk=%sKiB\n' \
            "$(date --iso-8601=seconds)" "$GPU_TEMP" "$GPU_POWER" "$GPU_POWER_LIMIT" \
            "$GPU_MEMORY" "$CPU_TEMP" "$MEM_AVAILABLE_KIB" "$DISK_AVAILABLE_KIB" |
            tee -a "$HEALTH_LOG"
        LAST_HEARTBEAT=$NOW
    fi
    sleep 5
done

set +e
wait "$TRAIN_PID"
TRAIN_RC=$?
set -e
if [[ -n $STOP_REASON ]]; then
    printf '학습이 워치독에 의해 안전 중단됨: %s\n' "$STOP_REASON" >&2
    exit 70
fi
if (( TRAIN_RC != 0 )); then
    printf '학습 프로세스가 코드 %d로 종료됨. 자동 재시작하지 않음\n' "$TRAIN_RC" >&2
    exit "$TRAIN_RC"
fi
printf '학습과 평가가 정상 완료됨\n'
