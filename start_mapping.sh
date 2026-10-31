#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${SENTRY_ROS_SETUP:-/opt/ros/humble/setup.bash}"
INSTALL_SETUP="${SENTRY_INSTALL_SETUP:-$WORKSPACE/install/setup.bash}"
MAP_ROOT="${SENTRY_MAP_ROOT:-$HOME/sentry_maps}"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "Usage: ./start_mapping.sh [--check] [name:=value ...]"
  echo "--check validates installed mapping software/model and map output without starting nodes."
  exit 0
fi
check_only=0
if [[ "${1:-}" == "--check" ]]; then
  check_only=1
  shift
fi

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

if (( check_only )); then
  exec python3 "$WORKSPACE/tools/check_mapping.py" --map-root "$MAP_ROOT"
fi

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

exec python3 "$WORKSPACE/tools/mapping_session.py" \
  --manifest "$MAP_ROOT/latest.json" --map "$MAP_YAML" --pcd "$MAP_PCD" -- \
  ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  mode:=mapping use_robot_state_pub:=True \
  use_rviz:="${SENTRY_USE_RVIZ:-True}" \
  enable_auto_explore:="${SENTRY_AUTO_EXPLORE:-True}" \
  mapping_enable_mcu_serial:="${SENTRY_ENABLE_MCU_SERIAL:-True}" \
  mcu_serial_port:="${SENTRY_SERIAL_PORT:-/dev/serial_port}" \
  map:="$MAP_YAML" prior_pcd_file:="$MAP_PCD" "$@"
