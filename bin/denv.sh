#!/usr/bin/env bash
# Source this file to prepare the environment for the AI/drone CLI.
#
#   source bin/denv.sh
#
# It is safe to source multiple times.  Override any value from the outside:
#   ROS_DOMAIN_ID=0        # target a different ROS graph (e.g. real hardware)
#   DRONE_WS=/path/to/ws   # point at a different drone workspace
#
# shellcheck shell=bash

DRONE_AI_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export DRONE_AI_ROOT

# Local SITL uses domain 42 (see run_sim.sh).  Real hardware usually uses 0.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# The workspace that provides drone_px4_node and the /drone/* interfaces.
export DRONE_WS="${DRONE_WS:-/home/nvidia/drone_ws}"

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

# ROS 2 Humble.
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

# The drone_ws overlay (provides drone_interfaces and drone_px4_node).
if [ -f "${DRONE_WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "${DRONE_WS}/install/setup.bash"
else
  echo "denv: warning: ${DRONE_WS}/install/setup.bash not found; run colcon build in drone_ws" >&2
fi

# Make lib/drone_api.py importable.
export PYTHONPATH="${DRONE_AI_ROOT}/lib:${PYTHONPATH:-}"
