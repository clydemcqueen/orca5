#!/usr/bin/env bash

# common.sh - Shared Docker setup functions for Orca5

# Auto-detect graphics driver if set to 'auto'
detect_graphics() {
    local requested="${1:-auto}"
    if [ "$requested" == "auto" ]; then
        if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
            echo "nvidia"
        elif [ -d "/dev/dri" ]; then
            echo "intel"
        else
            echo "cpu"
        fi
    else
        echo "$requested"
    fi
}

# Return Docker flags for selected graphics
get_graphics_flags() {
    local graphics="$1"
    local flags=""

    case "$graphics" in
        nvidia)
            flags="-e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all --security-opt seccomp=unconfined --gpus all"
            ;;
        intel)
            if [ -d "/dev/dri" ]; then
                flags="-v /dev/dri:/dev/dri --device /dev/dri"
            fi
            ;;
        cpu)
            flags="-e LIBGL_ALWAYS_SOFTWARE=1"
            ;;
        *)
            echo "Warning: unknown graphics mode '$graphics', falling back to cpu" >&2
            flags="-e LIBGL_ALWAYS_SOFTWARE=1"
            ;;
    esac

    echo "$flags"
}

# Return Docker group flags for DRI access
get_dri_group_flags() {
    local flags=""
    if [ -d "/dev/dri" ]; then
        flags="--group-add video"
        local host_render_gid
        host_render_gid=$(getent group render 2>/dev/null | cut -d: -f3)
        if [ -n "$host_render_gid" ] && [ "$host_render_gid" != "65534" ]; then
            flags+=" --group-add $host_render_gid"
        else
            flags+=" --group-add 110"
        fi
        for gid in $(stat -c '%g' /dev/dri/* 2>/dev/null | sort -u); do
            if [ "$gid" != "65534" ] && [ "$gid" != "0" ]; then
                flags+=" --group-add $gid"
            fi
        done
    fi
    echo "$flags"
}

# Ensure the Docker image exists, or build it
ensure_image() {
    local tag="$1"
    local target="$2"
    local dir="$3"

    if ! docker image inspect "$tag" >/dev/null 2>&1; then
        echo "Docker image '$tag' not found. Building image..."
        "$dir/build.sh" "$target" || exit 1
    fi
}
