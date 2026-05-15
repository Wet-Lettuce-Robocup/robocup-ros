#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f /underlay_ws/install/setup.bash ] && source /underlay_ws/install/setup.bash
[ -f /overlay_ws/install/setup.bash ] && source /overlay_ws/install/setup.bash

exec "$@"
