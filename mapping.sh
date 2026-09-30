#!/usr/bin/env bash
# Manual GO2 simulation mapping; run from any directory.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
# Help works even before the ROS workspace is built.
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  exec python3 "$ROOT/tools/mapping.py" --help
fi
set +u
source "$ROOT/tools/env.sh"
set -u
export ROS_DOMAIN_ID="${LEGBOT_SIM_DOMAIN_ID:-178}"
export ROS_LOCALHOST_ONLY=1
export IGN_IP=127.0.0.1
export IGN_PARTITION="legbot_mapping_${ROS_DOMAIN_ID}_$$"
export PYTHONUNBUFFERED=1
for arg in "$@"; do
  if [[ "$arg" == --check ]]; then
    exec python3 "$ROOT/tools/mapping.py" "$@"
  fi
done
# Share the navigation lock: never compete for the same robot.
mkdir -p "$ROOT/log"
exec 9>"$ROOT/log/simulation.lock"
flock -n 9 || { echo '本工作区已有 simulation.sh 或 mapping.sh 在运行，请先退出它。' >&2; exit 1; }
exec python3 "$ROOT/tools/mapping.py" "$@"
