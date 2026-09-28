#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
JOBS="${LEGBOT_BUILD_JOBS:-2}"
ARGS=()
while (($#)); do
  case "$1" in
    --jobs)
      [[ $# -ge 2 ]] || { echo '用法：./build.sh [--jobs N] [colcon 参数...]' >&2; exit 2; }
      JOBS="$2"; shift 2 ;;
    -h|--help)
      echo '用法：./build.sh [--jobs N] [--packages-select 包...] [--packages-up-to 包...]'
      echo '默认增量编译全部包，按包顺序执行，单包默认 2 线程；其他参数透传 colcon。'
      exit 0 ;;
    *) ARGS+=("$1"); shift ;;
  esac
done
[[ $EUID -ne 0 ]] || { echo '请不要使用 sudo 编译。' >&2; exit 1; }
[[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || { echo '--jobs 必须是正整数' >&2; exit 2; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo '未安装 ROS Humble，请先执行 ./install.sh。' >&2; exit 1; }
export LEGBOT_BUILD_JOBS="$JOBS"
printf 'Legbot Humble：顺序编译各包，单包最多 %s 线程。\n' "$JOBS"
"$ROOT/tools/build.sh" "${ARGS[@]}"
printf '\n编译完成。使用前执行：source %q/tools/env.sh\n' "$ROOT"
