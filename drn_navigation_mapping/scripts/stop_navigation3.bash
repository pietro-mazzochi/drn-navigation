#!/bin/bash
LOG_DIR="${HOME}/.drn_logs"
for name in localization pathplanner; do
  if [ -f "${LOG_DIR}/${name}.pid" ]; then
    kill "$(cat "${LOG_DIR}/${name}.pid")" 2>/dev/null || true
    rm -f "${LOG_DIR}/${name}.pid"
  fi
done
