#!/usr/bin/env bash
set -Eeuo pipefail

echo "[stop_navigation] Encerrando apenas os processos explicitamente listados..."

EXACT=(
controller_server
planner_server
recoveries_server
bt_navigator
costmap_filter_info_server
filter_mask_server
map_server
map_saver_cli
dummy_pointcloud
speeds_filter_mask_server
speeds_costmap_filter_info
nav2_map_server
nav2_amcl
nav2_lifecycle_manager
nav2_controller
nav2_planner
nav2_behaviors
nav2_bt_navigator
)

REGEX=(
"drn_navigation_localization_launch\.py"
"drn_navigation_pathplanner_launch\.py"
"drn_integration_pointcloud_to_laserscan .*pointcloud2_to_laserscan_launch\.py"
)

DRY_RUN="${DRY_RUN:-0}"

kill_and_wait_pids() {
local label="$1"; shift
local pids=("$@")
[ ${#pids[@]} -eq 0 ] && return 0

echo " - $label: PIDs alvo: ${pids[*]}"

if [ "$DRY_RUN" = "1" ]; then
    echo "   -> DRY_RUN=1 (não será enviado sinal)"
    return 0
fi

kill -TERM "${pids[@]}" 2>/dev/null || true

for _ in {1..50}; do
    sleep 0.1
    local alive=()
    for pid in "${pids[@]}"; do
        kill -0 "$pid" 2>/dev/null && alive+=("$pid") || true
    done
    if [ ${#alive[@]} -eq 0 ]; then
        echo "   -> $label: finalizado."
        return 0
    fi
    done

    echo "   -> $label: ainda em execução, enviando SIGKILL..."
    kill -KILL "${pids[@]}" 2>/dev/null || true
    echo "   -> $label: finalizado à força."
}

for name in "${EXACT[@]}"; do
    [ -z "$name" ] && continue
    mapfile -t pids < <(pgrep -x "$name" || true)
    [ ${#pids[@]} -eq 0 ] && { echo " - $name: nada para parar"; continue; }
    kill_and_wait_pids "$name" "${pids[@]}"
done

for patt in "${REGEX[@]}"; do
    [ -z "$patt" ] && continue
    mapfile -t pids < <(pgrep -f "$patt" || true)
    [ ${#pids[@]} -eq 0 ] && { echo " - $patt: nada para parar"; continue; }
    kill_and_wait_pids "$patt" "${pids[@]}"
done

echo -e "\n[stop_navigation] Verificação opcional (não falha se ausente):"
ros2 node list 2>/dev/null || true

echo "[stop_navigation] Concluído."
