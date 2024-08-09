#!/bin/bash

arg=$1
workspace=$2

export ROS_DOMAIN_ID=69

cd ~/$workspace

source install/setup.bash

ros2 launch launches/$arg