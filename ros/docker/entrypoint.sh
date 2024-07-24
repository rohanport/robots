#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

SHELL_TYPE=$(ps -hp $$ | awk '{print $5}' | xargs basename)

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.$SHELL_TYPE

# Source the base workspace, if built
if [ -f /ros_ws/install/setup.$SHELL_TYPE ]
then
  source /ros_ws/install/setup.$SHELL_TYPE
fi

# Grant permissions and source the overlay workspace, if built
if [ -f /overlay_ws/install/setup.$SHELL_TYPE ]
then
  source /overlay_ws/install/setup.$SHELL_TYPE
fi

# Execute the command passed into this entrypoint
exec "$@"
