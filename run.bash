#!/bin/bash

## Adding for convenience, needs to be removed later, allows the script to exit immediately on error
set -e

xhost_enabled=false
if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
    xhost +SI:localuser:"$(whoami)" >/dev/null
    xhost_enabled=true
fi

cleanup_xhost()
{
    if [ "$xhost_enabled" = true ]; then
        xhost -SI:localuser:"$(whoami)" >/dev/null 2>&1 || true
    fi
}
trap cleanup_xhost EXIT INT TERM

container="dtc-jackal-$(hostname)-glider"
if docker container inspect "$container" >/dev/null 2>&1; then
    echo "[GLIDER] Removing existing container '$container'..."
    docker rm -f "$container" >/dev/null
fi

docker run --rm -it --gpus all \
    --privileged \
    --network=host \
    -u $UID \
    -e RUN=true \
    -e USE_SIM_TIME="${USE_SIM_TIME:-false}" \
    -e DISPLAY="${DISPLAY:-}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(cd .. && pwd):/data:ro" \
    --name "$container" \
    dtc-jackal-$(hostname):glider
