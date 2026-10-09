#!/bin/bash
set -e
source /opt/ros/humble/setup.bash

LOG_DIR="${HOME}/.drn_logs"; mkdir -p "$LOG_DIR"
nohup ros2 launch drn_navigation_mapping drn_navigation_localization_launch.py \
  >"${LOG_DIR}/localization.log" 2>&1 &

echo $! > "${LOG_DIR}/localization.pid"


