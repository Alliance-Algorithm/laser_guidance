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

echo "=== 正常预览 (imshow) ==="
echo "配置: $CONFIG"
echo ""

cd "$REPO_ROOT"
./build/example_v4l2_preview "$CONFIG"
