#!/usr/bin/env bash
# Parse descriptions and run local inference only; never start ROS/Gazebo nodes.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source tools/env.sh
export ROS_LOG_DIR="$PWD/log/offline"
export IGN_GAZEBO_RESOURCE_PATH="$PWD/install/legbot_bringup/share/legbot_bringup/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export SDF_PATH="$IGN_GAZEBO_RESOURCE_PATH${SDF_PATH:+:$SDF_PATH}"
mkdir -p "$ROS_LOG_DIR"
python3 tests/check_launch_descriptions.py
python3 tests/check_model_offline.py
python3 tests/check_estop_offline.py
python3 tests/check_direct_goal_state_machine.py
install/legbot_bringup/lib/legbot_bringup/pct_path_mission --self-test
install/rl_quadruped_controller/lib/rl_quadruped_controller/policy_contract_check src/go2_description/config/himloco
install/rl_quadruped_controller/lib/rl_quadruped_controller/policy_contract_check src/go2_description/config/go2_cts
install/rl_quadruped_controller/lib/rl_quadruped_controller/policy_contract_check src/go2_description/config/legged_gym
install/rl_quadruped_controller/lib/rl_quadruped_controller/onnx_policy_contract_check src/go2_description/config/stairs_trot
install/rl_quadruped_controller/lib/rl_quadruped_controller/onnx_policy_contract_check src/go2_description/config/mjlab_flat
install/rl_quadruped_controller/lib/rl_quadruped_controller/onnx_policy_contract_check src/go2_description/config/moe_cts_77k
python3 tests/check_dependencies_offline.py
python3 - <<'PY_REPORT'
import datetime, json, os, platform
from pathlib import Path
report = {
    'checked_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'platform': platform.platform(),
    'ros_distro': os.environ.get('ROS_DISTRO'),
    'checks_passed': ['launch descriptions', 'model and policy contracts',
                      'emergency-stop logic', 'direct-goal state machine', 'PCT mission self-test',
                      'native dependencies and PCT route'],
    'simulation_started_by_this_check': False,
    'real_robot_connected_or_commanded': False,
    'limitations': ['Offline checks do not certify a Humble simulation or real-robot run.',
                    'Optional Open3D/CuPy tomography dependencies are checked separately.'],
}
Path('log/offline/validation.json').write_text(json.dumps(report, indent=2) + '\n')
print('PASS offline checks; no simulation or robot experiment was started by this check')
PY_REPORT
