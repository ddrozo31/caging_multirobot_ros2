#!/bin/bash
# Variables for docker/compose.yaml. Load them once per terminal, from the repository root:
#   source docker/env.sh
# They give the container your user ID (files you create keep your owner), your screen (DISPLAY,
# XAUTHORITY) and the groups that own the GPU (video, render) and the joystick (input); on Windows
# (WSL2) also the folder of WSLg.
export HOST_UID=$(id -u) HOST_GID=$(id -g)
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
export VIDEO_GID=$(getent group video | cut -d: -f3)
export RENDER_GID=$(getent group render | cut -d: -f3)
export INPUT_GID=$(getent group input | cut -d: -f3)

# Windows (WSL2 + WSLg): where WSLg's sockets are, seen by the Docker daemon.
#   Docker Engine inside WSL2 -> /mnt/wslg          Docker Desktop -> /run/desktop/mnt/host/wslg
if grep -qi microsoft /proc/version 2>/dev/null; then
  if docker info --format '{{.OperatingSystem}}' 2>/dev/null | grep -qi "docker desktop"; then
    export WSLG_DIR=/run/desktop/mnt/host/wslg
  else
    export WSLG_DIR=/mnt/wslg
  fi
  echo "caging docker (WSL2): UID ${HOST_UID}, DISPLAY ${DISPLAY}, WSLg ${WSLG_DIR}; use the gui-wsl service"
else
  echo "caging docker: UID ${HOST_UID}, DISPLAY ${DISPLAY}, video ${VIDEO_GID}, render ${RENDER_GID}, input ${INPUT_GID}"
fi
