# gazebo_px4_sim

This repository gives the scaffolding for running a PX4 drone simulation in a forest environment. It makes use of a repository by ARK-Electronics called ROS_PX4_Offboard_Example, but makes some corrections to get everything to launch properly, use the correct map, and dockerizes the whole setup to make the system portable.

## Requirements

This software is designed to be run on any Ubuntu 20.04 or higher host machine. The software itself will run under a Docker image.

## Cloning the repository

You need to clone the repository with submodules to get all the files. Use the command:
```bash
git clone https://github.com/steffanlloyd/px4-gazebo-sim.git --recursive
```

## How to set up

If you don't already have docker install, install it now. There is a convenience script in this repository to help:
```bash
./scripts/docker_install.sh
```

To set up the repository, run the `scripts/init.sh` script:
```bash
./scripts/init.sh
```
This will download the PX4 autopilot software to `libs/PX4-Autopilot`, build the docker image, and download QGroundControl executable to your machine. It will take 5-6 minutes to run, depending on your internet speed.

Then, run the docker. To access the GUI, this command MUST be run from within an active display (e.g., you can't run the run command from an SSH connection).
```bash
./scripts/docker_run.sh
```
After the docker is running however, you can issue commands over SSH and the GUI will show on the display you were on when you started the Docker.

## Connect to the Docker
You can connect to the Docker in a number of ways. At the command line, there is a convenience script defined:
```bash
./scripts/docker_connect.sh
```
In VS code, you can also open the IDE in the Docker which is often more convenient.

## Building the code

Once in the docker, build the ros directory:
```bash
cd ros2-ws
colcon build
```
The first time this runs, it will take a while to build the px4_msgs directory. However, this only needs to run once.

The .bashrc file of this docker has already been initialized to source the ros installation for each new window. However, you will still need to source the ROS build with:
```bash
source install/setup.bash # run from your ros2-ws folder after building.
```
There is an alias for this command also defined within the .bashrc file of the docker file to make this easier, you can just type:
```bash
sis
```

## How to run

To run the simulation, you will need to run three executables.

### QGroundControl
First, **on the host machine**, you need to run QGroundControl. The init script already downloaded this when you ran it. You can run the software by navigating to `resources/QGroundControl` in your file browser and double clicking on the icon. Alternatively, you can just run it as `./QGroundControl.AppImage`.

### Gazebo
Second, **in the docker image**, run the Gazebo simulation. Use this command:
```bash
cd ~/libraries/PX4-Autopilot
PX4_GZ_WORLD=baylands make px4_sitl gz_x500_lidar_down
```

### MicroDDS
Lastly, you need to run MicroDDS, **in the docker image** to allow the simulation to communicate with ROS. Use this code:
```bash
MicroXRCEAgent udp4 -p 8888
```

### Manual control example
As a starting example, we have provided some basic code that will allow you to navigate the drone using the arrow keys and WASD. To start up this software, you will need to open two more **in the docker image**.

Terminal 1, run the launch file for the control example:
```bash
sis # source the install
ros2 launch px4_offboard offboard_velocity_control.launch.py 
```

Terminal 2: run the control input window:
```bash
sis # source the install
ros2 run px4_offboard control
```

This will allow you to control the simulation from ROS. Press the space bar to take off, then you can use the arrow keys and WASD to navigate the drone around.

For the challenge, you will need to make a new ROS package (or modify this existing one), to read in the ROS messages, and automatically take actions based on the result to navigate through the forest, and map the surrounding environment.