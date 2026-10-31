#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${SENTRY_ROS_SETUP:-/opt/ros/humble/setup.bash}"
INSTALL_SETUP="${SENTRY_INSTALL_SETUP:-$WORKSPACE/install/setup.bash}"
MAP_ROOT="${SENTRY_MAP_ROOT:-$HOME/sentry_maps}"

for arg in "$@"; do
  case "$arg" in
    mode:=*|slam:=*|map:=*|prior_pcd_file:=*)
      echo "Do not override mode or map paths when using start_navigation.sh." >&2
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

if ! selected="$(python3 "$WORKSPACE/tools/map_manifest.py" select --manifest "$MAP_ROOT/latest.json")"; then
  echo "No complete validated map is available. Run ./start_mapping.sh first." >&2
  exit 1
fi
mapfile -t paths <<< "$selected"
if (( ${#paths[@]} != 2 )); then
  echo "Invalid map selection." >&2
  exit 1
fi
echo "Navigation map: ${paths[0]}"
echo "Navigation PCD: ${paths[1]}"
echo "Check localization in RViz before sending any goal."

exec ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  mode:=navigation use_robot_state_pub:=True \
  use_rviz:="${SENTRY_USE_RVIZ:-True}" \
  enable_mcu_status_nav:="${SENTRY_ENABLE_MCU_SERIAL:-True}" \
  mcu_serial_port:="${SENTRY_SERIAL_PORT:-/dev/serial_port}" \
  map:="${paths[0]}" prior_pcd_file:="${paths[1]}" "$@"
