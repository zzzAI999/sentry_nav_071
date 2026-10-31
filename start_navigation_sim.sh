#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${SENTRY_ROS_SETUP:-/opt/ros/humble/setup.bash}"
INSTALL_SETUP="${SENTRY_INSTALL_SETUP:-$WORKSPACE/install/setup.bash}"
MAP_ROOT="${SENTRY_MAP_ROOT:-$HOME/sentry_maps}"
ROUTE_SCRIPT="$WORKSPACE/src/pb2025_sentry_nav/pb2025_sentry_nav/scripts/rm071_two_point_route.py"
route_enabled=False
points=("" "" "" "")
route_args=()

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "Usage: ./start_navigation_sim.sh [--two-point | x1 y1 x2 y2] [use_rviz:=False]"
  echo "Load the latest complete map; place the virtual robot with RViz 2D Pose Estimate."
  echo "By default use RViz goals; --two-point reads config/two_point_route.yaml."
  exit 0
fi
if [[ "${1:-}" == "--two-point" ]]; then
  route_enabled=True
  route_args=(--config "${SENTRY_ROUTE_CONFIG:-$WORKSPACE/config/two_point_route.yaml}")
  shift
elif (( $# > 0 )) && [[ "$1" != *:=* ]]; then
  if (( $# < 4 )); then
    echo "Provide x1 y1 x2 y2, or use --two-point." >&2
    exit 2
  fi
  route_enabled=True
  route_args=(-- "$1" "$2" "$3" "$4")
  shift 4
fi
for arg in "$@"; do
  case "$arg" in
    use_rviz:=True|use_rviz:=False) ;;
    *) echo "Unsupported simulation option: $arg" >&2; exit 2 ;;
  esac
done
if [[ ! -r "$ROS_SETUP" || ! -r "$INSTALL_SETUP" ]]; then
  echo "ROS or workspace setup is missing. Build the workspace first." >&2
  exit 2
fi
source "$ROS_SETUP"
source "$INSTALL_SETUP"
set -u

if [[ "$route_enabled" == True ]]; then
  coordinates="$(python3 "$ROUTE_SCRIPT" --print-points "${route_args[@]}")" || exit 2
  mapfile -t points <<< "$coordinates"
fi
mkdir -p -- "$MAP_ROOT"
MAP_ROOT="$(cd -- "$MAP_ROOT" && pwd)"
exec 9>"$MAP_ROOT/.navigation.lock"
if ! flock -n 9; then
  echo "Another mapping, navigation or simulation session is already running." >&2
  exit 2
fi
selected="$(python3 "$WORKSPACE/tools/map_manifest.py" select --manifest "$MAP_ROOT/latest.json")" || exit 1
mapfile -t paths <<< "$selected"
if (( ${#paths[@]} != 2 )); then
  echo "No complete saved map is available. Run ./start_mapping.sh first." >&2
  exit 1
fi

simulation_domain="${SENTRY_SIM_DOMAIN_ID:-71}"
if [[ ! "$simulation_domain" =~ ^[0-9]{1,3}$ ]] || (( 10#$simulation_domain > 232 )); then
  echo "SENTRY_SIM_DOMAIN_ID must be an integer from 0 to 232." >&2
  exit 2
fi
simulation_domain="$((10#$simulation_domain))"
if [[ "$simulation_domain" == "${ROS_DOMAIN_ID:-0}" ]]; then
  echo "Choose a simulation DDS domain different from the real robot's ROS_DOMAIN_ID." >&2
  exit 2
fi
export ROS_DOMAIN_ID="$simulation_domain"
echo "Simulation map: ${paths[0]}"
echo "Simulation DDS domain: $ROS_DOMAIN_ID; velocity output: sim/cmd_vel."
echo "Use RViz 2D Pose Estimate to place the virtual robot, then send navigation goals."

exec ros2 launch pb2025_nav_bringup map_simulation_launch.py \
  map:="${paths[0]}" use_rviz:="${SENTRY_USE_RVIZ:-True}" \
  enable_two_point_route:="$route_enabled" \
  route_x1:="${points[0]}" route_y1:="${points[1]}" \
  route_x2:="${points[2]}" route_y2:="${points[3]}" "$@"
