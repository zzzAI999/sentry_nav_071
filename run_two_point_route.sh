#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${SENTRY_ROS_SETUP:-/opt/ros/humble/setup.bash}"
INSTALL_SETUP="${SENTRY_INSTALL_SETUP:-$WORKSPACE/install/setup.bash}"

if [[ ! -r "$ROS_SETUP" || ! -r "$INSTALL_SETUP" ]]; then
  echo "ROS or workspace setup is missing. Build the workspace first." >&2
  exit 2
fi
source "$ROS_SETUP"
source "$INSTALL_SETUP"
set -u

MAP_ROOT="${SENTRY_MAP_ROOT:-$HOME/sentry_maps}"
mkdir -p -- "$MAP_ROOT"
exec 8>"$MAP_ROOT/.two_point_route.lock"
if ! flock -n 8; then
  echo "Another two-point route is already running." >&2
  exit 2
fi

exec ros2 run pb2025_sentry_nav rm071_two_point_route.py "$@"
