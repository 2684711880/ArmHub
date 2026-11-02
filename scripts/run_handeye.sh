#!/usr/bin/env bash
# Start camera/real-feedback FK and RViz; owns no motor connection.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$ROOT/calibration_ws/install/setup.bash" ]]; then
  echo '先运行 bash scripts/build_handeye.sh 编译 RViz 标定插件。' >&2
  exit 1
fi
source "$ROOT/calibration_ws/install/setup.bash"
export PYTHONPATH="$ROOT:${PYTHONPATH:-}"
export ROS_LOCALHOST_ONLY=1
cd "$ROOT"
export ROS_DOMAIN_ID=77
export PANTHERA_HANDEYE_TF_ONLY=1
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/panthera-handeye-${UID}.lock"
if ! flock -n 9; then
  echo 'Hand-eye camera session is already running.' >&2
  exit 1
fi
SESSION_DIR="$ROOT/validation/handeye/$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$SESSION_DIR"
echo "采集窗口日志：$SESSION_DIR"
inputs_pid=""
rviz_pid=""
cleanup() {
  trap - EXIT INT TERM
  [[ -z "$rviz_pid" ]] || kill -TERM "$rviz_pid" 2>/dev/null || true
  [[ -z "$inputs_pid" ]] || kill -INT "$inputs_pid" 2>/dev/null || true
  for _ in {1..30}; do
    if { [[ -z "$rviz_pid" ]] || ! kill -0 "$rviz_pid" 2>/dev/null; } &&
       { [[ -z "$inputs_pid" ]] || ! kill -0 "$inputs_pid" 2>/dev/null; }; then
      break
    fi
    sleep .1
  done
  # These are only this launcher's camera/RViz children, never the motor bridge.
  [[ -z "$rviz_pid" ]] || kill -KILL "$rviz_pid" 2>/dev/null || true
  [[ -z "$inputs_pid" ]] || kill -KILL "$inputs_pid" 2>/dev/null || true
  wait || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
"$ROOT/.venv_dynamics/bin/python" -m apps.handeye_rviz.inputs --joint-source gravity-log --serial 262622274345 9>&- > >(exec 9>&-; tee "$SESSION_DIR/inputs.log") 2>&1 &
inputs_pid=$!
# Allow the camera image topic to appear before RViz loads the topic selector.
sleep 3
if ! kill -0 "$inputs_pid" 2>/dev/null; then
  echo "Camera/FK input failed; see $SESSION_DIR/inputs.log" >&2
  exit 1
fi
rviz2 -d "$ROOT/apps/handeye_rviz/handeye.rviz" --ros-args --log-level moveit_handeye_calibration_target:=warn 9>&- > >(exec 9>&-; tee "$SESSION_DIR/rviz.log") 2>&1 &
rviz_pid=$!
wait -n "$inputs_pid" "$rviz_pid"
