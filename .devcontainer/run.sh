#!/bin/bash

set -e


sudo mkdir -p /ws/src && sudo chown -R vscode /ws
sudo ln -s /models /engagement_detector/models && sudo chown -R vscode /models

ln -s /engagement_detector /ws/src/

source /opt/ros/${ROS_DISTRO}/setup.bash
# # sudo apt update

rosdep --rosdistro=${ROS_DISTRO} update 

cd /ws
rosdep install --from-paths src --ignore-src -r -y 
colcon build 

echo "source /ws/install/setup.bash" >> ~/.bashrc

cd /engagement_detector
mkdir -p models

if [ -f models/lstm_10_50_runsigm_runsigm.h5 ]; then
    echo "File exists"
else
    echo "Downloading model"
    wget -O models/lstm_10_50_runsigm_runsigm.h5 'http://lcas.lincoln.ac.uk/owncloud/index.php/s/UHWU1g5qXU9RiAp/download'
    sudo ln -s /models /engagement_detector/models
fi
