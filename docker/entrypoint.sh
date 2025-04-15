#!/bin/bash

set -e

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "/home/ros/ros2-ws/install/setup.bash" ]; then
    source /home/ros/ros2-ws/install/setup.bash
fi

echo "Provided arguments: $@"

exec $@
