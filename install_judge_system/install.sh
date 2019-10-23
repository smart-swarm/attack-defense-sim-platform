#!/bin/bash
echo "stat installing the judge system..."
sudo dpkg -i ros-kinetic-judge-system-msgs_0.0.0-0xenial_amd64.deb
sh load_custom_msgs_to_ros_env.sh
sudo dpkg -i ros-kinetic-judge-system_0.0.0-0xenial_amd64.deb
echo "judge system installed."

