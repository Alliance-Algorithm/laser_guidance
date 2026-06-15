#!/usr/bin/env bash
set -euo pipefail

laser_wait_for_sdp() {
    local sdp_path="$1"
    local pid="${2-}"
    local label="$3"
    local log_file="${4-}"

    for _ in $(seq 1 200); do
        if [ -s "$sdp_path" ]; then
            return 0
        fi
        if [ -n "$pid" ] && ! kill -0 "$pid" 2>/dev/null; then
            echo "$label exited before SDP was ready"
            if [ -n "$log_file" ] && [ -s "$log_file" ]; then
                echo "--- $log_file ---"
                sed -n '1,120p' "$log_file"
            fi
            return 1
        fi
        sleep 0.1
    done

    echo "timed out waiting for RTP SDP at $sdp_path"
    if [ -n "$pid" ] && ! kill -0 "$pid" 2>/dev/null; then
        if [ -n "$log_file" ] && [ -s "$log_file" ]; then
            echo "--- $log_file ---"
            sed -n '1,120p' "$log_file"
        fi
    fi
    return 1
}
