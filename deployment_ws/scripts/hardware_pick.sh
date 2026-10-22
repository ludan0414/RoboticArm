#!/usr/bin/env bash
# 实机代码 (site / real-cell). Not part of the Gazebo simulation stack.
# Real-cell pick graph. Person on e-stop. Ctrl+C stops the launch
# (servo disable + TCP drop; does not BlackOut / cut 48 V).
#
#   ./scripts/hardware_pick.sh
#   ./scripts/hardware_pick.sh semantic_device:=cpu
#   ./scripts/hardware_pick.sh use_rviz:=true
#   ./scripts/hardware_pick.sh preprocessor_config:=<share>/preprocessor_d555_site.yaml
#
# No NVIDIA GPU: semantic_device defaults to cpu unless the caller sets it.
# Profile B (default) is preprocessor_d555_site.yaml (motion_gate off).
# Profile A is preprocessor_d555_live.yaml. Do not pass preprocessor_d555_replay.yaml
# into this live graph (use_sim_time true).
#
# Detect path subscribes raw images. Pass use_compressed:=true only to
# read old JPEG/PNG topics (.../image_raw/compressed).
#
# Overlay is on by default (publish_overlay:=true). yaml keeps it off for
# sim-eval; this launch overrides so /luggage/semantic/overlay is recorded.
#
# --skip-observe / --detect-only / --observe-pose are driver flags, not launch args.
# Other terminal, same overlay:
#   source scripts/env_jazzy_real.sh
# then:
#   ros2 run luggage_planning hardware_pick_driver.py --detect-only
#   ros2 run luggage_planning hardware_pick_driver.py --plan-only --observe-pose current
#   ros2 run luggage_planning hardware_pick_driver.py --observe-pose current
#       # YOLO floor box: stay at live joints (default). Do not send
#       # simulation pickup_observe.
#   OBSERVE_POSE=current ./scripts/run_floor_box_pick.sh plan
#   ros2 run luggage_planning hardware_pick_driver.py --release
#
# ServoJ + Orin sensors (does not change this script's waypoint default):
#   ./scripts/hardware_pick_servo_j.sh
set -euo pipefail

DEPLOY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "$DEPLOY/.." && pwd)"
if [[ -d "${REPO}/src/luggage_planning" ]]; then
  HUMBLE_WS="$REPO"
else
  HUMBLE_WS="${REPO}/elfin_humble_ws"
fi
SDK="${REPO}/third_party/huayan_python_sdk"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "need /opt/ros/jazzy" >&2
  exit 1
fi
if [[ ! -f "$DEPLOY/install/setup.bash" ]]; then
  echo "build deployment_ws: colcon build --packages-select elfin_trajectory_executor" >&2
  exit 1
fi
if [[ ! -f "$HUMBLE_WS/install/setup.bash" ]]; then
  echo "missing ${HUMBLE_WS}/install/setup.bash" >&2
  exit 1
fi
if [[ ! -f "$DEPLOY/livox_ws/env.sh" ]]; then
  echo "missing Livox overlay: $DEPLOY/livox_ws/env.sh" >&2
  echo "  Mid-360 is required. From deployment_ws:" >&2
  echo "  ./scripts/setup_livox_driver.sh" >&2
  exit 1
fi

unset PYTHONPATH COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# local_setup only — setup.bash chains elfin_humble_ws and shadows master.
# shellcheck disable=SC1091
source "$HUMBLE_WS/install/local_setup.bash"
# shellcheck disable=SC1091
source "$DEPLOY/livox_ws/install/local_setup.bash"
# shellcheck disable=SC1091
source "$DEPLOY/install/local_setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
unset ROS_LOCALHOST_ONLY
export PYTHONPATH="${SDK}${PYTHONPATH:+:${PYTHONPATH}}"
export LD_LIBRARY_PATH="/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
if [[ -d "$DEPLOY/livox_ws/sdk_prefix/lib" ]]; then
  export LD_LIBRARY_PATH="$DEPLOY/livox_ws/sdk_prefix/lib:${LD_LIBRARY_PATH}"
fi


if ! python3 -c "import moveit_msgs" >/dev/null 2>&1; then
  echo "WARNING: MoveIt not installed. --detect-only works; plan/pick need:" >&2
  echo "  sudo apt install ros-jazzy-moveit-msgs ros-jazzy-moveit-configs-utils \\" >&2
  echo "    ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-planners-ompl \\" >&2
  echo "    ros-jazzy-moveit-kinematics ros-jazzy-moveit-simple-controller-manager" >&2
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "hardware pick launch. e-stop person required. Ctrl+C stops nodes (no BlackOut)."

_has_nvidia_gpu() {
  command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1
}

# ros2 launch wants foo:=bar, not --foo:=bar. A lone "--" is also dropped.
# Driver-only flags (skip-observe, detect-only, plan-only) are not launch
# arguments; passing them used to look like a silent no-op.
launch_args=()
has_semantic_device=false
has_use_compressed=false
has_scene_tf=false
for arg in "$@"; do
  if [[ "$arg" == "--" ]]; then
    continue
  elif [[ "$arg" == --*:=* ]]; then
    arg="${arg#--}"
  fi
  case "$arg" in
    skip-observe|skip-observe:=*|detect-only|detect-only:=*|plan-only|plan-only:=*|--skip-observe|--detect-only|--plan-only|observe-pose|observe-pose:=*)
      echo "WARNING: '$arg' is a hardware_pick_driver.py flag, not a launch argument." >&2
      echo "  Other terminal: ros2 run luggage_planning hardware_pick_driver.py --observe-pose current --plan-only" >&2
      continue
      ;;
  esac
  launch_args+=("$arg")
  if [[ "$arg" == semantic_device:=* ]]; then
    has_semantic_device=true
  fi
  if [[ "$arg" == use_compressed:=* ]]; then
    has_use_compressed=true
  fi
  if [[ "$arg" == scene_tf_config:=* ]]; then
    has_scene_tf=true
  fi
done

if [[ "$has_scene_tf" == false ]]; then
  launch_args+=("scene_tf_config:=$DEPLOY/config/scene_tf.loading_site.yaml")
fi

if [[ "$has_use_compressed" == false ]]; then
  echo "detect path subscribes raw image_raw (use_compressed:=false)"
  launch_args+=("use_compressed:=false")
fi

if [[ "$has_semantic_device" == false ]] && ! _has_nvidia_gpu; then
  echo "no NVIDIA GPU (nvidia-smi failed); defaulting semantic_device:=cpu" >&2
  launch_args+=("semantic_device:=cpu")
fi

exec ros2 launch luggage_planning hardware_pick.launch.py "${launch_args[@]}"
