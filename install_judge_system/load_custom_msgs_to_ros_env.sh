#!/bin/bash
cd ~
user_name=$(pwd)
echo $user_name
cd -
cp rosdep.yaml ~/.ros/rosdep.yaml
echo "yaml file://$user_name/.ros/rosdep.yaml" > 10_judge_system_msg_tf_ros_kinetic.list
sudo cp 10_judge_system_msg_tf_ros_kinetic.list /etc/ros/rosdep/sources.list.d/
rosdep update

