#!/usr/bin/env bash
# Usage: ./run_sim.sh [world: free_flight|tether_rig] [scenario: interactive|roll_steps|pitch_steps|yaw_steps|hold]
#
# Interactive 3D Browser Studio (works natively on Wayland / Hyprland in any browser):
#   ./run_sim.sh
#   (or: ./run_sim.sh free_flight interactive)
#   -> Starts Gazebo physics server + opens http://localhost:8090 with live 3D quadcopter,
#      real-time telemetry strip-chart, environment switcher (free_flight <-> tether_rig),
#      auto scenarios (roll/pitch/yaw steps), manual sliders, wind force, and live PID tuning.
set -e
cd "$(dirname "$0")"
WORLD=${1:-free_flight}; SCEN=${2:-interactive}; shift 2 2>/dev/null || true
cleanup() { pkill -f "gz sim" 2>/dev/null || true; }
trap cleanup EXIT
micromamba run -n gzsim python generate_sdf.py >/dev/null
cleanup; sleep 0.5
if [ "$SCEN" = "interactive" ]; then
  (sleep 2 && xdg-open http://localhost:8090 >/dev/null 2>&1 || true) &
  micromamba run -n gzsim python fc_sim.py --world "$WORLD" --scenario interactive --manage-server --out "results/sim_${WORLD}_${SCEN}.csv" "$@" || true
else
  micromamba run -n gzsim python fc_sim.py --world "$WORLD" --scenario "$SCEN" --manage-server --out "results/sim_${WORLD}_${SCEN}.csv" "$@" || true
fi
