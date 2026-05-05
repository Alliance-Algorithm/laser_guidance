#!/usr/bin/env bash

# UGREEN 采集卡 -> v4l2loopback 虚拟摄像头启动脚本
# 当前适配环境：
# - Arch Linux
# - UGREEN 95348 暴露 /dev/video2(Video Capture) + /dev/video3(Metadata Capture)
# - 输出虚拟摄像头默认 /dev/video4，YUYV 1920x1080@30

set -Eeuo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

UGREEN_PATTERN="${UGREEN_PATTERN:-UGREEN}"
SOURCE_DEVICE="${SOURCE_DEVICE:-}"
SOURCE_FORMAT="${SOURCE_FORMAT:-auto}"
WIDTH="${WIDTH:-1920}"
HEIGHT="${HEIGHT:-1080}"
INPUT_FPS="${INPUT_FPS:-60}"
OUTPUT_FPS="${OUTPUT_FPS:-30}"
VIRTUAL_DEVICE_NR="${VIRTUAL_DEVICE_NR:-4}"
VIRTUAL_DEVICE="${VIRTUAL_DEVICE:-/dev/video${VIRTUAL_DEVICE_NR}}"
VIRTUAL_LABEL="${VIRTUAL_LABEL:-VirtualCamera}"
CLEANUP_FFMPEG="${CLEANUP_FFMPEG:-1}"
SET_USER_ACL="${SET_USER_ACL:-1}"

REAL_USER="${SUDO_USER:-${USER:-$(id -un)}}"

info() {
    echo -e "${GREEN}$*${NC}"
}

warn() {
    echo -e "${YELLOW}$*${NC}"
}

fail() {
    echo -e "${RED}错误: $*${NC}" >&2
    exit 1
}

usage() {
    cat <<EOF
用法:
  $0 [source_device]

可选环境变量:
  SOURCE_DEVICE=/dev/video2       指定 UGREEN 输入设备，默认自动查找
  SOURCE_FORMAT=auto|mjpeg|yuyv   指定输入格式，默认 auto
  WIDTH=1920 HEIGHT=1080          输入/输出分辨率
  INPUT_FPS=60 OUTPUT_FPS=30      输入帧率和虚拟摄像头输出帧率
  VIRTUAL_DEVICE_NR=4             v4l2loopback 设备号
  VIRTUAL_DEVICE=/dev/video4      显式指定虚拟摄像头路径
  CLEANUP_FFMPEG=0                不清理占用虚拟摄像头的旧 ffmpeg
  SET_USER_ACL=0                  不给当前用户设置 /dev/video* ACL

示例:
  $0
  SOURCE_DEVICE=/dev/video2 $0
  VIRTUAL_DEVICE_NR=6 OUTPUT_FPS=60 $0
EOF
}

as_root() {
    if [[ "${EUID}" -eq 0 ]]; then
        "$@"
        return
    fi

    command -v sudo >/dev/null 2>&1 || fail "需要 sudo 执行: $*"
    sudo "$@"
}

require_command() {
    command -v "$1" >/dev/null 2>&1 || fail "未找到 $1。Arch Linux 上请执行: sudo pacman -S --needed $2"
}

has_video_capture_format() {
    local dev="$1"
    v4l2-ctl --device="$dev" --list-formats-ext 2>/dev/null |
        grep -Eq '^[[:space:]]*\[[0-9]+\]:'
}

card_type() {
    local dev="$1"
    v4l2-ctl --device="$dev" --info 2>/dev/null |
        awk -F: '/Card type/ { sub(/^[ \t]+/, "", $2); print $2; exit }'
}

find_ugreen_capture_device() {
    local dev
    for dev in /dev/video*; do
        [[ -e "$dev" ]] || continue

        local card
        card="$(card_type "$dev")"
        [[ "$card" == *"$UGREEN_PATTERN"* ]] || continue

        if has_video_capture_format "$dev"; then
            echo "$dev"
            return 0
        fi

        warn "跳过 $dev: $card 不是可取图 Video Capture 节点，通常是 metadata 节点"
    done

    return 1
}

normalize_source_format() {
    local requested="$1"
    local dev="$2"

    case "$requested" in
        auto)
            if v4l2-ctl --device="$dev" --list-formats-ext | grep -q "'YUYV'"; then
                echo "yuyv"
            elif v4l2-ctl --device="$dev" --list-formats-ext | grep -q "'MJPG'"; then
                echo "mjpeg"
            else
                fail "$dev 不支持 YUYV 或 MJPG，无法为当前脚本自动选择输入格式"
            fi
            ;;
        mjpeg|MJPG|mjpg)
            echo "mjpeg"
            ;;
        yuyv|YUYV|yuyv422)
            echo "yuyv"
            ;;
        *)
            fail "SOURCE_FORMAT 必须是 auto、mjpeg 或 yuyv"
            ;;
    esac
}

ffmpeg_input_format() {
    case "$1" in
        mjpeg) echo "mjpeg" ;;
        yuyv) echo "yuyv422" ;;
        *) fail "内部错误: 未知输入格式 $1" ;;
    esac
}

ensure_loopback_available() {
    if modinfo v4l2loopback >/dev/null 2>&1; then
        return
    fi

    fail "当前内核没有 v4l2loopback 模块。Arch Linux 上请先安装并重启或重新加载内核模块:
  sudo pacman -S --needed v4l2loopback-dkms linux-headers
  sudo modprobe v4l2loopback"
}

ensure_virtual_device() {
    if [[ -e "$VIRTUAL_DEVICE" ]]; then
        warn "虚拟摄像头设备已存在: $VIRTUAL_DEVICE"
        return
    fi

    warn "加载 v4l2loopback: $VIRTUAL_DEVICE label=$VIRTUAL_LABEL"
    as_root modprobe v4l2loopback \
        video_nr="$VIRTUAL_DEVICE_NR" \
        card_label="$VIRTUAL_LABEL" \
        exclusive_caps=1 \
        max_buffers=8

    [[ -e "$VIRTUAL_DEVICE" ]] || fail "加载 v4l2loopback 后仍未看到 $VIRTUAL_DEVICE"
}

set_virtual_caps() {
    warn "设置虚拟摄像头 caps: YUYV:${WIDTH}x${HEIGHT}@${OUTPUT_FPS}/1"

    if command -v v4l2loopback-ctl >/dev/null 2>&1; then
        as_root v4l2loopback-ctl set-caps "$VIRTUAL_DEVICE" "YUYV:${WIDTH}x${HEIGHT}@${OUTPUT_FPS}/1" ||
            warn "v4l2loopback-ctl set-caps 失败，后续由 ffmpeg 写入时协商格式"
        return
    fi

    if [[ -x "$HOME/v4l2loopback/utils/v4l2loopback-ctl" ]]; then
        as_root "$HOME/v4l2loopback/utils/v4l2loopback-ctl" set-caps "$VIRTUAL_DEVICE" "YUYV:${WIDTH}x${HEIGHT}@${OUTPUT_FPS}/1" ||
            warn "本地 v4l2loopback-ctl set-caps 失败，后续由 ffmpeg 写入时协商格式"
        return
    fi

    warn "未找到 v4l2loopback-ctl，尝试使用 v4l2-ctl 设置格式"
    as_root v4l2-ctl --device="$VIRTUAL_DEVICE" \
        --set-fmt-video="width=${WIDTH},height=${HEIGHT},pixelformat=YUYV" ||
        warn "v4l2-ctl 设置虚拟摄像头格式失败，后续由 ffmpeg 写入时协商格式"
}

grant_acl_if_requested() {
    [[ "$SET_USER_ACL" == "1" ]] || return

    if ! command -v setfacl >/dev/null 2>&1; then
        warn "未找到 setfacl，跳过 ACL 设置；如果权限不足，请把 $REAL_USER 加入 video 组后重新登录"
        return
    fi

    local dev
    for dev in "$SOURCE_DEVICE" "$VIRTUAL_DEVICE"; do
        [[ -e "$dev" ]] || continue
        as_root setfacl -m "u:${REAL_USER}:rw" "$dev" ||
            warn "无法为 $REAL_USER 设置 $dev 的 ACL"
    done
}

cleanup_old_ffmpeg() {
    [[ "$CLEANUP_FFMPEG" == "1" ]] || return

    warn "清理写入 $VIRTUAL_DEVICE 的旧 ffmpeg 进程"
    as_root pkill -f "ffmpeg .*${VIRTUAL_DEVICE}" 2>/dev/null || true
}

print_device_summary() {
    echo "--------------------------------------------------"
    echo "UGREEN 输入设备:"
    v4l2-ctl --device="$SOURCE_DEVICE" --all | grep -E "Card type|Width/Height|Pixel Format|Frames per second" | sed 's/^/  /' || true
    echo
    echo "虚拟摄像头设备:"
    v4l2-ctl --device="$VIRTUAL_DEVICE" --all | grep -E "Card type|Width/Height|Pixel Format|Frames per second" | sed 's/^/  /' || true
    echo "--------------------------------------------------"
}

main() {
    if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
        usage
        exit 0
    fi

    if [[ $# -gt 0 ]]; then
        SOURCE_DEVICE="$1"
    fi

    info "========================================"
    info "     UGREEN 采集卡虚拟摄像头启动脚本"
    info "========================================"

    require_command v4l2-ctl v4l-utils
    require_command ffmpeg ffmpeg
    require_command modprobe kmod

    if [[ -z "$SOURCE_DEVICE" ]]; then
        warn "自动查找 UGREEN Video Capture 设备"
        SOURCE_DEVICE="$(find_ugreen_capture_device)" ||
            fail "未找到 UGREEN 可取图节点。当前设备列表:
$(v4l2-ctl --list-devices)"
    fi

    [[ -e "$SOURCE_DEVICE" ]] || fail "输入设备不存在: $SOURCE_DEVICE"
    has_video_capture_format "$SOURCE_DEVICE" || fail "$SOURCE_DEVICE 不是可取图 Video Capture 节点"

    SOURCE_FORMAT="$(normalize_source_format "$SOURCE_FORMAT" "$SOURCE_DEVICE")"
    local ffmpeg_format
    ffmpeg_format="$(ffmpeg_input_format "$SOURCE_FORMAT")"

    info "已选择输入设备: $SOURCE_DEVICE format=$SOURCE_FORMAT"

    ensure_loopback_available

    cleanup_old_ffmpeg
    ensure_virtual_device
    set_virtual_caps
    grant_acl_if_requested
    print_device_summary

    local ffmpeg_cmd=(
        ffmpeg
        -hide_banner
        -loglevel info
        -nostdin
        -f v4l2
        -thread_queue_size 8
        -input_format "$ffmpeg_format"
        -video_size "${WIDTH}x${HEIGHT}"
        -framerate "$INPUT_FPS"
        -i "$SOURCE_DEVICE"
        -vf "fps=${OUTPUT_FPS},format=yuyv422"
        -f v4l2
        "$VIRTUAL_DEVICE"
    )

    warn "启动视频转发，按 Ctrl+C 停止"
    info "输入: $SOURCE_DEVICE ${SOURCE_FORMAT} ${WIDTH}x${HEIGHT}@${INPUT_FPS}"
    info "输出: $VIRTUAL_DEVICE YUYV ${WIDTH}x${HEIGHT}@${OUTPUT_FPS}"
    echo "--------------------------------------------------"
    printf '转发命令:'
    printf ' %q' "${ffmpeg_cmd[@]}"
    echo
    echo "--------------------------------------------------"

    if [[ ! -r "$SOURCE_DEVICE" || ! -w "$VIRTUAL_DEVICE" ]]; then
        warn "当前用户权限不足，使用 sudo 运行 ffmpeg"
        as_root "${ffmpeg_cmd[@]}"
    else
        "${ffmpeg_cmd[@]}"
    fi

    warn "转发已停止"
}

main "$@"
