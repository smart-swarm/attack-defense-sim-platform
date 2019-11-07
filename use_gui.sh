#!/bin/bash

cd gui_executor
cmake .
make
cd -
cp gui_executor/gui_executor catkin_ws/
cd catkin_ws
./gui_executor


