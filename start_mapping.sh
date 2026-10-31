#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${SENTRY_ROS_SETUP:-/opt/ros/humble/setup.bash}"
INSTALL_SETUP="${SENTRY_INSTALL_SETUP:-$WORKSPACE/install/setup.bash}"
MAP_ROOT="${SENTRY_MAP_ROOT:-$HOME/sentry_maps}"

for arg in "$@"; do
  case "$arg" in
    mode:=*|slam:=*|map:=*|prior_pcd_file:=*)
      echo "Do not override mode or map paths when using start_mapping.sh." >&2
      exit 2
      ;;
  esac
done

if [[ ! -r "$ROS_SETUP" || ! -r "$INSTALL_SETUP" ]]; then
  echo "ROS or workspace setup is missing. Build the workspace first." >&2
  exit 2
fi
source "$ROS_SETUP"
source "$INSTALL_SETUP"
set -u

mkdir -p -- "$MAP_ROOT"
MAP_ROOT="$(cd -- "$MAP_ROOT" && pwd)"
exec 9>"$MAP_ROOT/.navigation.lock"
if ! flock -n 9; then
  echo "Another mapping or navigation session is already running." >&2
  exit 2
fi

SESSION="$(mktemp -d "$MAP_ROOT/map_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
MAP_YAML="$SESSION/map.yaml"
MAP_PCD="$SESSION/map.pcd"
echo "Mapping session: $SESSION"
echo "Press Ctrl+C once and wait for PGM/YAML and PCD to finish saving."

interrupted=0
trap 'interrupted=1' INT TERM
status=0
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  mode:=mapping use_robot_state_pub:=True \
  use_rviz:="${SENTRY_USE_RVIZ:-True}" \
  enable_auto_explore:="${SENTRY_AUTO_EXPLORE:-True}" \
  mapping_enable_mcu_serial:="${SENTRY_ENABLE_MCU_SERIAL:-True}" \
  mcu_serial_port:="${SENTRY_SERIAL_PORT:-/dev/serial_port}" \
  map:="$MAP_YAML" prior_pcd_file:="$MAP_PCD" "$@" || status=$?
trap - INT TERM

if (( status != 0 && interrupted == 0 && status != 130 && status != 143 )); then
  echo "Mapping exited abnormally (status $status); the previous map remains selected." >&2
  exit "$status"
fi

if ! python3 "$WORKSPACE/tools/map_manifest.py" record \
    --manifest "$MAP_ROOT/latest.json" --map "$MAP_YAML" --pcd "$MAP_PCD"; then
  echo "Map files are incomplete; the previous map remains selected." >&2
  exit 1
fi
echo "The next navigation start will use this validated map."
