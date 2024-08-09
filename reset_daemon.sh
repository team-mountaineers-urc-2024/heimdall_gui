#!bin/bash
source /opt/ros/humble/setup.bash
ros2 daemon stop
sleep 3
ros2 daemon start
