#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STORE="$SCRIPT_DIR/.config-path"

list_configs() {
    echo "可用的配置文件:"
    local i=1
    for f in "$REPO_ROOT"/config/*.yaml; do
        local name="$(basename "$f")"
        local mark=" "
        if [ -f "$STORE" ] && [ "$(cat "$STORE")" = "$f" ]; then
            mark="*"
        fi
        printf "  %s %d) %s\n" "$mark" "$i" "$name"
        eval "cfg_$i=\"$f\""
        ((i++))
    done
}

if [ $# -gt 0 ]; then
    echo "$(realpath "$1")" > "$STORE"
    echo "已选择: $(realpath "$1")"
    exit 0
fi

list_configs
echo ""
read -rp "选择配置 [1-$((i-1))]: " choice
cfg_var="cfg_$choice"
if [ -z "${!cfg_var:-}" ]; then
    echo "无效选择"
    exit 1
fi
echo "${!cfg_var}" > "$STORE"
echo "已选择: ${!cfg_var}"
