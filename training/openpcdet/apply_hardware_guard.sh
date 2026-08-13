#!/usr/bin/env bash
set -Eeuo pipefail

log() {
    printf '%s hardware-guard: %s\n' "$(date --iso-8601=seconds)" "$*"
}

if [[ ${EUID} -ne 0 ]]; then
    log "root 권한이 필요함"
    exit 1
fi

# The i9-14900KS is intentionally kept out of turbo while this workstation is
# used for long-running CUDA jobs. This avoids the high-voltage operating point
# that caused native SIGILL failures before the motherboard firmware update,
# and remains a conservative reliability guard afterwards.
if [[ -w /sys/devices/system/cpu/intel_pstate/no_turbo ]]; then
    printf '1' > /sys/devices/system/cpu/intel_pstate/no_turbo
    log "Intel turbo disabled for training reliability"
else
    log "intel_pstate no_turbo interface not available"
fi

if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --persistence-mode=1 >/dev/null
    nvidia-smi --power-limit=250 >/dev/null
    log "RTX 3090 persistence enabled and power limit set to 250 W"
else
    log "nvidia-smi not available"
    exit 1
fi
