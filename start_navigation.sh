#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${SENTRY_ROS_SETUP:-/opt/ros/humble/setup.bash}"
INSTALL_SETUP="${SENTRY_INSTALL_SETUP:-$WORKSPACE/install/setup.bash}"
MAP_ROOT="${SENTRY_MAP_ROOT:-$HOME/sentry_maps}"
ROUTE_CONFIG="${SENTRY_ROUTE_CONFIG:-$WORKSPACE/config/two_point_route.yaml}"
ROUTE_SCRIPT="$WORKSPACE/src/pb2025_sentry_nav/pb2025_sentry_nav/scripts/rm071_two_point_route.py"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "Usage: ./start_navigation.sh [x1 y1 x2 y2] [name:=value ...]"
  echo "Without coordinates, read first/second from $ROUTE_CONFIG."
  echo "Start navigation, wait for localization, visit point 1 (0.5 m), then point 2 (0.2 m)."
  exit 0
fi

route_args=(--config "$ROUTE_CONFIG")
if (( $# > 0 )) && [[ "$1" != *:=* ]]; then
  if (( $# < 4 )); then
    echo "Provide all four map coordinates: x1 y1 x2 y2." >&2
    exit 2
  fi
  route_args=(-- "$1" "$2" "$3" "$4")
  shift 4
fi

for arg in "$@"; do
  case "$arg" in
    mode:=*|slam:=*|map:=*|prior_pcd_file:=*|enable_two_point_route:=*|route_x1:=*|route_y1:=*|route_x2:=*|route_y2:=*)
      echo "Do not override mode, map paths or route launch arguments in start_navigation.sh." >&2
      exit 2
      ;;
    *:=*) ;;
    *)
      echo "Unexpected argument: $arg. Use x1 y1 x2 y2, followed by name:=value options." >&2
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

if ! coordinates="$(python3 "$ROUTE_SCRIPT" --print-points "${route_args[@]}")"; then
  echo "Set two map coordinates in $ROUTE_CONFIG or pass x1 y1 x2 y2." >&2
  exit 2
fi
mapfile -t points <<< "$coordinates"
if (( ${#points[@]} != 4 )); then
  echo "Invalid two-point route configuration." >&2
  exit 2
fi

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
echo "Two-point route: (${points[0]}, ${points[1]}) within 0.5 m -> (${points[2]}, ${points[3]}) within 0.2 m."
echo "The route starts automatically after localization and Nav2 are ready."

exec ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  mode:=navigation use_robot_state_pub:=True \
  use_rviz:="${SENTRY_USE_RVIZ:-True}" \
  enable_mcu_status_nav:="${SENTRY_ENABLE_MCU_SERIAL:-True}" \
  mcu_serial_port:="${SENTRY_SERIAL_PORT:-/dev/serial_port}" \
  map:="${paths[0]}" prior_pcd_file:="${paths[1]}" \
  enable_two_point_route:=True \
  route_x1:="${points[0]}" route_y1:="${points[1]}" \
  route_x2:="${points[2]}" route_y2:="${points[3]}" "$@"
