#!/usr/bin/env bash
set -euo pipefail

echo "=== V4L2 采集卡扫描 ==="
echo ""

if command -v v4l2-ctl &>/dev/null; then
    v4l2-ctl --list-devices 2>/dev/null || true
else
    echo "v4l2-ctl 未安装，使用 /dev/video* 检测:"
    echo ""
    for dev in /dev/video*; do
        if [ -e "$dev" ]; then
            info="$(v4l2-ctl -d "$dev" --info 2>/dev/null || echo "无法读取")"
            echo "  $dev  $(echo "$info" | head -1)"
        fi
    done
fi

echo ""
echo "=== 支持的分辨率/格式 (v4l2-ctl --list-formats-ext) ==="
echo ""
for dev in /dev/video*; do
    if [ -e "$dev" ]; then
        echo "--- $dev ---"
        v4l2-ctl -d "$dev" --list-formats-ext 2>/dev/null | head -40 || true
        echo ""
    fi
done
