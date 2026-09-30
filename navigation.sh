#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
# Help uses the workspace interpreter after environment setup, like normal runs.
set +u
source "$ROOT/tools/env.sh"
set -u
export ROS_DOMAIN_ID="${LEGBOT_SIM_DOMAIN_ID:-178}"
export ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1 PYTHONUNBUFFERED=1
export IGN_PARTITION="legbot_navigation_${ROS_DOMAIN_ID}_$$"
export LD_LIBRARY_PATH="$ROOT/install/pct_planner/share/pct_planner/planner/lib:${LD_LIBRARY_PATH:-}"
check_only=false
for arg in "$@"; do
  case "$arg" in --help|-h|--check|--prepare-only) check_only=true ;; esac
done
if ! $check_only; then
  mkdir -p "$ROOT/log"
  exec 9>"$ROOT/log/simulation.lock"
  flock -n 9 || { echo '已有仿真/建图/导航在运行，请先退出。' >&2; exit 1; }
fi
exec python3 "$ROOT/tools/navigation.py" "$@"
