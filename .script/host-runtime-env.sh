#!/usr/bin/env bash
set -euo pipefail

laser_prepare_host_runtime_env() {
    local script_dir repo_root compat_root compat_lib
    script_dir="$(cd "$(dirname "$(realpath "${BASH_SOURCE[0]}")")" && pwd)"
    repo_root="$(cd "$script_dir/.." && pwd)"
    compat_root="$repo_root/.runtime-compat"
    compat_lib="$compat_root/lib"

    mkdir -p "$compat_lib"

    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
        set +u
        # shellcheck disable=SC1091
        source /opt/ros/jazzy/setup.bash
        set -u
    fi

    if [[ ! -e "$compat_lib/libspdlog.so.1.12" && -e /usr/lib/libspdlog.so.1.17 ]]; then
        ln -sf /usr/lib/libspdlog.so.1.17 "$compat_lib/libspdlog.so.1.12"
    fi

    if [[ ! -e "$compat_lib/libpython3.12.so.1.0" && -e /usr/lib/libpython3.12.so.1.0 ]]; then
        ln -sf /usr/lib/libpython3.12.so.1.0 "$compat_lib/libpython3.12.so.1.0"
    fi

    export LD_LIBRARY_PATH="$compat_lib:/opt/ros/jazzy/lib:${LD_LIBRARY_PATH-}"

    # MVS SDK U3V 传输层控制传输不 claim 接口 → 内核 usbfs 告警并 reset 相机
    # （dmesg "did not claim interface N before use"）。预加载 shim 自动补
    # claim，消除该 reset 触发源。仅作用于经本脚本启动的进程。
    shim="$repo_root/build/liblibusb_claim_shim.so"
    if [[ -f "$shim" ]]; then
        if [[ -n "${LD_PRELOAD-}" ]]; then
            export LD_PRELOAD="$shim:$LD_PRELOAD"
        else
            export LD_PRELOAD="$shim"
        fi
    fi
}
