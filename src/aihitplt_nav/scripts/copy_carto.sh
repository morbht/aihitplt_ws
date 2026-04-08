#!/bin/bash

echo "start copy aihitplt_cartographer.launch to /opt/ros/melodic/share/cartographer_ros/launch"
sudo cp aihitplt_cartographer.launch /opt/ros/melodic/share/cartographer_ros/launch
echo ""
echo "start copy aihitplt.lua to /opt/ros/melodic/share/cartographer_ros/configuration_files"
sudo cp aihitplt.lua /opt/ros/melodic/share/cartographer_ros/configuration_files
echo "finish !!!"

