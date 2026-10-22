#!/usr/bin/env bash
# Orin Humble perception graph. Planning stays on the ThinkPad.
# Frozen flow: docs/hardware/2026-09-22_orin_deploy_baseline.md
# Later perception edits must keep this entry point and the ThinkPad
# commands below. Do not fold CPS, MoveIt, or a second camera into it.
#
#   ./scripts/orin_perception.sh
#   ./scripts/orin_perception.sh start_livox:=false
#
# This host is JetPack / ROS 2 Humble. Do not source Jazzy. Do not run
# hardware_pick.sh here (x86_64 D555 + Jazzy).
#
# ThinkPad (when 192.168.0.50 is on the robot LAN), other machine:
#   cd /home/adamliao/work/RoboticArm-master/deployment_ws
#   source scripts/env_jazzy_real.sh
#   ./scripts/hardware_pick.sh \
#     start_d555:=false start_perception:=false \
#     start_planning:=true start_executor:=true start_scene:=true \
#     use_moveit:=true use_rviz:=false
#   ros2 run luggage_planning hardware_pick_driver.py --detect-only
set -euo pipefail

DEPLOY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "$DEPLOY/.." && pwd)"
LIVOX_ENV="${LIVOX_WS:-$DEPLOY/livox_ws}/env.sh"
DDS_LIB="/home/hku_reconova/librealsense_dds/lib"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "need /opt/ros/humble (this script is Orin-only)" >&2
  exit 1
fi
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  echo "WARNING: jazzy is also installed; this script sources humble only" >&2
fi
if [[ ! -f "$REPO/install/local_setup.bash" ]]; then
  echo "missing $REPO/install/local_setup.bash" >&2
  echo "  colcon build --symlink-install --packages-select \\" >&2
  echo "    luggage_msgs luggage_description luggage_perception" >&2
  exit 1
fi
if [[ ! -f "$LIVOX_ENV" ]]; then
  echo "missing Livox overlay: $LIVOX_ENV" >&2
  exit 1
fi
if [[ ! -d "$DDS_LIB" ]]; then
  echo "missing D555 DDS librealsense: $DDS_LIB" >&2
  exit 1
fi

# Jetson YOLO-World is live-usable on CUDA only (~45 ms vs ~1.2 s on CPU).
for arg in "$@"; do
  case "$arg" in
    semantic_device:=cpu|semantic_device:=CPU)
      echo "Orin live YOLO is CUDA-only. Do not pass $arg." >&2
      exit 1
      ;;
  esac
done
python3 - <<'PY'
import torch
assert torch.cuda.is_available(), "Orin YOLO-World needs CUDA (CPU is ~1.2s/frame)"
print("torch cuda:", torch.cuda.get_device_name(0))
PY

_ensure_jumbo() {
  local nic="${ORIN_JUMBO_NIC:-eno1}"
  [[ -e "/sys/class/net/$nic" ]] || return 0
  local mtu
  mtu="$(cat "/sys/class/net/$nic/mtu")"
  if [[ "$mtu" == "9000" ]]; then
    echo "$nic mtu=9000"
    return 0
  fi
  echo "$nic mtu=$mtu (need 9000 for D555); applying via /sbin/ip" >&2
  if ! sudo -n /sbin/ip link set "$nic" down; then
    echo "could not bounce $nic (sudo /sbin/ip). D555 frames may stay 0 Hz." >&2
    return 0
  fi
  sudo -n /sbin/ip link set "$nic" mtu 9000 || true
  sudo -n /sbin/ip link set "$nic" up || true
  sleep 2
  echo "$nic mtu=$(cat "/sys/class/net/$nic/mtu")"
}

_ensure_jumbo
if ping -c 1 -W 1 -M do -s 8972 192.168.11.55 >/dev/null 2>&1; then
  echo "D555 jumbo path OK"
else
  echo "WARNING: jumbo ping to 192.168.11.55 failed. Host MTU may be 9000" >&2
  echo "  while a switch/camera hop is 1500. Color/depth may stay 0 Hz." >&2
fi

unset PYTHONPATH COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$REPO/install/local_setup.bash"
# shellcheck disable=SC1091
source "$LIVOX_ENV"
if [[ -f "$DEPLOY/install/local_setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$DEPLOY/install/local_setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
unset ROS_LOCALHOST_ONLY
export ROBOTICARM_ROOT="$REPO"
export LUGGAGE_CLIP_VENDOR_DIR="$REPO/src/luggage_perception/vendor"
if [[ -d "$DDS_LIB" ]]; then
  export LD_LIBRARY_PATH="$DDS_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROBOTICARM_ROOT=${ROBOTICARM_ROOT}"
echo "Orin perception launch. ThinkPad owns planning/CPS/scene TF."

scene_args=()
has_scene_tf=false
for arg in "$@"; do
  if [[ "$arg" == scene_tf_config:=* ]]; then has_scene_tf=true; fi
done
if [[ "$has_scene_tf" == false ]]; then
  scene_args+=("scene_tf_config:=$DEPLOY/config/scene_tf.loading_site.yaml")
fi
exec ros2 launch luggage_perception orin_perception.launch.py "${scene_args[@]}" "$@"
