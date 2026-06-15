#!/usr/bin/env bash
set -euo pipefail

laser_filter_colon_path() {
    local path_list="${1-}"
    local remove_path="${2-}"
    local item
    local filtered=()

    IFS=':' read -r -a _laser_items <<< "$path_list"
    for item in "${_laser_items[@]}"; do
        [ -n "$item" ] || continue
        [ "$item" = "$remove_path" ] && continue
        filtered+=("$item")
    done

    local result=""
    if [ "${#filtered[@]}" -gt 0 ]; then
        local old_ifs="$IFS"
        IFS=':'
        result="${filtered[*]}"
        IFS="$old_ifs"
    fi
    printf '%s\n' "$result"
}

laser_prepend_colon_path() {
    local prepend_path="${1-}"
    local path_list
    path_list="$(laser_filter_colon_path "${2-}" "$prepend_path")"

    if [ -z "$path_list" ]; then
        printf '%s\n' "$prepend_path"
    else
        printf '%s:%s\n' "$prepend_path" "$path_list"
    fi
}

laser_configure_hik_mvs_env() {
    local script_dir repo_root vendor_runtime_dir build_env_file
    local mode sdk_root runtime_dir common_runenv clprotocol_dir allusersprofile
    local vendor_sdk_root system_sdk_root

    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    repo_root="$(cd "$script_dir/.." && pwd)"
    vendor_sdk_root="$repo_root/vendor/hikcamera/src/sdk"
    system_sdk_root="${MVS_SDK_ROOT:-/opt/MVS}"
    vendor_runtime_dir="$repo_root/vendor/hikcamera/src/sdk/lib"
    build_env_file="$repo_root/build/hikcamera-runtime.env"

    mode="${HIKCAMERA_SDK_MODE:-AUTO}"
    if [ -f "$build_env_file" ]; then
        # shellcheck disable=SC1090
        . "$build_env_file"
        if [ -n "${HIKCAMERA_BUILD_SDK_MODE:-}" ]; then
            mode="${HIKCAMERA_BUILD_SDK_MODE#\'}"
            mode="${mode%\'}"
        fi
        if [ -n "${HIKCAMERA_BUILD_SDK_ROOT:-}" ]; then
            sdk_root="${HIKCAMERA_BUILD_SDK_ROOT#\'}"
            sdk_root="${sdk_root%\'}"
        fi
        if [ -n "${HIKCAMERA_BUILD_RUNTIME_DIR:-}" ]; then
            runtime_dir="${HIKCAMERA_BUILD_RUNTIME_DIR#\'}"
            runtime_dir="${runtime_dir%\'}"
        fi
    fi

    case "${mode,,}" in
        auto)
            if [ -e "$vendor_sdk_root/include/MvCameraControl.h" ] \
                && [ -e "$vendor_sdk_root/lib/libMvCameraControl.so" ]; then
                mode="vendor"
            else
                mode="system"
            fi
            ;;
        vendor|system)
            mode="${mode,,}"
            ;;
        *)
            echo "Unsupported HIKCAMERA_SDK_MODE: $mode" >&2
            return 1
            ;;
    esac

    case "$mode" in
        system)
            sdk_root="${sdk_root:-$system_sdk_root}"
            runtime_dir="${runtime_dir:-$sdk_root/lib/64}"
            common_runenv="$sdk_root/lib"
            clprotocol_dir="$sdk_root/lib/CLProtocol"
            allusersprofile="$sdk_root/MVFG"

            if [ ! -e "$runtime_dir/libMvCameraControl.so" ]; then
                echo "Missing system Hik MVS runtime: $runtime_dir/libMvCameraControl.so" >&2
                return 1
            fi

            export HIKCAMERA_SDK_MODE="system"
            export MVS_SDK_ROOT="$sdk_root"
            export MVCAM_SDK_PATH="$sdk_root"
            export MVCAM_COMMON_RUNENV="$common_runenv"
            export MVCAM_GENICAM_CLPROTOCOL="$clprotocol_dir"
            export ALLUSERSPROFILE="$allusersprofile"
            export LD_LIBRARY_PATH="$(
                laser_prepend_colon_path \
                    "$runtime_dir" \
                    "$(laser_filter_colon_path "${LD_LIBRARY_PATH-}" "$vendor_runtime_dir")"
            )"
            ;;
        vendor)
            sdk_root="${sdk_root:-${MVS_SDK_ROOT:-$vendor_sdk_root}}"
            runtime_dir="${runtime_dir:-$sdk_root/lib}"
            common_runenv="$runtime_dir"

            if [ ! -e "$runtime_dir/libMvCameraControl.so" ]; then
                echo "Missing vendor Hik MVS runtime: $runtime_dir/libMvCameraControl.so" >&2
                return 1
            fi

            export HIKCAMERA_SDK_MODE="vendor"
            export MVS_SDK_ROOT="$sdk_root"
            export MVCAM_SDK_PATH="$sdk_root"
            export MVCAM_COMMON_RUNENV="$common_runenv"
            if [ -d "$sdk_root/lib/CLProtocol" ]; then
                export MVCAM_GENICAM_CLPROTOCOL="$sdk_root/lib/CLProtocol"
            fi
            export LD_LIBRARY_PATH="$(laser_prepend_colon_path "$runtime_dir" "${LD_LIBRARY_PATH-}")"
            ;;
        *)
            echo "Unsupported HIKCAMERA_SDK_MODE: $mode" >&2
            return 1
            ;;
    esac
}
