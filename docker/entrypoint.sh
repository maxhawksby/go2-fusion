#!/bin/bash
# Builds the mounted workspace, then runs the container's command (by default the
# full robot launch). The build is incremental: build/ and install/ live in Docker
# volumes, and --symlink-install links Python files instead of copying them, so
# Python edits take effect on the next node restart without rebuilding.
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
cd /ros2_ws

if [ ! -f src/go2_robot_sdk/setup.py ]; then
    echo "No source in /ros2_ws/src. Start the container with 'docker compose up'," \
         "which mounts the repo there." >&2
    exit 1
fi
if [ -z "$(ls -A src/go2_robot_sdk/external_lib/aioice 2>/dev/null)" ]; then
    echo "The aioice submodule is missing. On the host, run:" \
         "git submodule update --init --recursive" >&2
    exit 1
fi

# Capped parallelism: an uncapped build used all 7.4 GB of a laptop's WSL and froze it.
# Raise COLCON_WORKERS and MAKEFLAGS on a machine with more memory.
export MAKEFLAGS="${MAKEFLAGS:--j4}"
colcon build --symlink-install --parallel-workers "${COLCON_WORKERS:-2}"
source install/setup.bash

exec "$@"
