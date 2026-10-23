#!/usr/bin/env bash
# 实机代码 (site / real-cell). Not part of the Gazebo simulation stack.
# Laptop smooth-control graph: ServoJ (StartServo + PushServoJ) + planning.
# Sensors and YOLO stay on Orin by default (same topic names over DDS).
#
#   ./scripts/hardware_pick_servo_j.sh
#   ./scripts/hardware_pick_servo_j.sh use_rviz:=true
#
# Explicit waypoint compatibility mode:
#   ./scripts/hardware_pick.sh execution_backend:=waypoint
#
# Laptop-only ServoJ (no Orin): keep local D555 + YOLO
#   ./scripts/hardware_pick_servo_j.sh start_d555:=true start_perception:=true
#
# servo_esj is rejected on this S20. Person on e-stop. No BlackOut.
#
# Laptop subscribe (Orin publishes raw images; names unchanged):
#   /camera/d555/color/image_raw
#   /camera/d555/color/camera_info
#   /camera/d555/aligned_depth_to_color/image_raw
#   /camera/d555/aligned_depth_to_color/camera_info
#   /livox/lidar
#   /livox/imu
#   /luggage/preprocessed/camera/color/image
#   /luggage/preprocessed/camera/color/camera_info
#   /luggage/preprocessed/camera/depth/image
#   /luggage/preprocessed/camera/depth/camera_info
#   /luggage/preprocessed/status
#   /luggage/semantic/mask
#   /luggage/semantic/overlay
#   /luggage/semantic/instance_mask
#   /luggage/semantic/yolo_detections
#   /luggage/semantic/cargo_points
#   /luggage/semantic/obstacle_points
# Service: /luggage_detector/detect_luggage
#
# Laptop still publishes /joint_states /tf /tf_static for Orin.
set -euo pipefail

DEPLOY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PICK="$DEPLOY/scripts/hardware_pick.sh"
if [[ ! -f "$PICK" ]]; then
  echo "missing $PICK" >&2
  exit 1
fi

launch_args=()
has_backend=false
has_d555=false
has_perception=false
for arg in "$@"; do
  if [[ "$arg" == "--" ]]; then
    continue
  elif [[ "$arg" == --*:=* ]]; then
    arg="${arg#--}"
  fi
  case "$arg" in
    execution_backend:=servo_esj|execution_backend:=ServoEsJ)
      echo "servo_esj is rejected on this S20. Use servo_j (this script) or waypoint (hardware_pick.sh)." >&2
      exit 1
      ;;
    execution_backend:=*) has_backend=true ;;
    start_d555:=*) has_d555=true ;;
    start_perception:=*) has_perception=true ;;
  esac
  launch_args+=("$arg")
done

injected=()
if [[ "$has_backend" == false ]]; then
  injected+=("execution_backend:=servo_j")
fi
if [[ "$has_d555" == false ]]; then
  injected+=("start_d555:=false")
fi
if [[ "$has_perception" == false ]]; then
  injected+=("start_perception:=false")
fi

echo "hardware_pick_servo_j: backend=servo_j. Default sensors/YOLO off (Orin owns them)."
echo "Waypoint override: ./scripts/hardware_pick.sh execution_backend:=waypoint"
echo "Laptop subscribe: /camera/d555/... /livox/lidar /luggage/preprocessed/... /luggage/semantic/yolo_detections and DetectLuggage."

exec "$PICK" "${injected[@]}" "${launch_args[@]}"
