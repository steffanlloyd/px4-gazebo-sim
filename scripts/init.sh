#!/bin/bash

# Clone the PX4 Autopilot repository to the /libs directory
git clone https://github.com/PX4/PX4-Autopilot.git --recursive $(dirname "$0")/../libs/PX4-Autopilot

# Download QGroundControl
wget -P $(dirname "$0")/../resources https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x $(dirname "$0")/../resources/QGroundControl.AppImage

# Build the docker
$(dirname "$0")/docker_build.sh