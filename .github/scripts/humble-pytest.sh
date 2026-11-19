#!/usr/bin/env bash
# Runs pytest inside ROS 2 Humble (Ubuntu 22.04, Python 3.10), the software Oak
# runs, instead of the GitHub runner's newer Python. Installs the package's
# rosdep keys and builds go2_interfaces first, so tests that import ROS
# messages can run. The package is taken from PYTHONPATH.
# Usage: .github/scripts/humble-pytest.sh [PYTHONPATH=src/<pkg>] <pytest args>
set -euo pipefail

pythonpath=""
if [[ "${1:-}" == PYTHONPATH=* ]]; then
  pythonpath="${1#PYTHONPATH=}"
  shift
fi

printf -v args '%q ' "$@"
exec docker run --rm -v "$PWD":/ws -w /ws -e PYTHONPATH="$pythonpath" ros:humble-ros-base \
  bash -c "apt-get update -qq >/dev/null \
    && apt-get install -y -qq python3-pytest python3-numpy python3-yaml python3-colcon-common-extensions >/dev/null \
    && rosdep update -q >/dev/null \
    && rosdep install -q -y -r --ignore-src --from-paths src/go2_interfaces ${pythonpath//:/ } >/dev/null \
    && source /opt/ros/humble/setup.bash \
    && colcon --log-base /tmp/log build --base-paths src/go2_interfaces --build-base /tmp/build --install-base /tmp/install >/dev/null \
    && source /tmp/install/setup.bash \
    && export PYTHONPATH=\"\$PYTHONPATH:$pythonpath\" \
    && python3 --version \
    && python3 -m pytest -o addopts='' $args"
