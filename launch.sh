#!/bin/bash

cd ~

source /opt/ros/humble/setup.bash

DIR=~/workspace-heimdall

if [ -d "$DIR" ]; then
    cd workspace-heimdall
else
    cd heimdall_gui
fi

# colcon build --packages-select heimdall_gui
source install/setup.bash
export ROS_DOMAIN_ID=69

ros2 launch heimdall_gui gui_bringup.launch.py 
