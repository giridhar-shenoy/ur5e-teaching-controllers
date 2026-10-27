#!/bin/bash
# Source ROS 2 Lyrical (and the workspace overlay when it has been built), then run the command.
set -e
source /opt/ros/lyrical/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
