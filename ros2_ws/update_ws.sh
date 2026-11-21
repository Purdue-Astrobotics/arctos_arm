#!/bin/bash

# Usage: source ./update_ws.sh

# Sources ros2 and applies whatever you changes you made:

source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash

colcon build --symlink-install # write files into install/
source install/setup.bash
