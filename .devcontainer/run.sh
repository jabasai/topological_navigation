#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
sudo apt-get update
rosdep --rosdistro=humble update

cd /home/lcas/ws
rosdep install --from-paths ./src -i -y
colcon build --symlink-install
echo "source /home/lcas/ws/install/setup.bash" >> ~/.bashrc

