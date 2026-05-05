#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STORE="$SCRIPT_DIR/.config-path"

CONFIG="${1:-}"
if [ -z "$CONFIG" ]; then
    if [ -f "$STORE" ]; then
        CONFIG="$(cat "$STORE")"
    else
        CONFIG="$REPO_ROOT/config/default.yaml"
    fi
fi

echo "=== RTP 推流预览 ==="
echo "配置: $CONFIG"
echo ""

TMP_CONFIG="$(mktemp /tmp/laser_guidance_stream_XXXX.yaml)"
python3 -c "
import yaml, sys
with open('$CONFIG') as f:
    cfg = yaml.safe_load(f)
cfg.setdefault('streaming', {})['enabled'] = True
cfg.setdefault('debug', {})['show_window'] = False
yaml.dump(cfg, sys.stdout, default_flow_style=False, allow_unicode=True)
" > "$TMP_CONFIG"

echo "临时配置: $TMP_CONFIG (streaming=enabled, show_window=false)"
echo ""
echo "播放命令: ffplay -protocol_whitelist file,rtp,udp -fflags nobuffer -flags low_delay /tmp/laser_guidance.sdp"
echo ""

cd "$REPO_ROOT"
./build/example_v4l2_preview "$TMP_CONFIG"
rm -f "$TMP_CONFIG"
