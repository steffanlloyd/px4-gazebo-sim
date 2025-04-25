#!/bin/bash

# Clone the PX4 Autopilot repository to the /libs directory
git clone https://github.com/PX4/PX4-Autopilot.git --recursive $(dirname "$0")/../libs/PX4-Autopilot

# Put in the modified lidar file
cp $(dirname "$0")/../resources/models/lidar_2d_v2/model.sdf $(dirname "$0")/../libs/PX4-Autopilot/Tools/simulation/gz/models/lidar_2d_v2/model.sdf

# Download QGroundControl
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
wget -P $(dirname "$0")/../resources https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x $(dirname "$0")/../resources/QGroundControl.AppImage

# Build the docker
$(dirname "$0")/docker_build.sh