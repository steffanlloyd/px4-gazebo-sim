# gazebo_px4_sim

To set up the repository, run the `scripts/init.sh` script:
```bash
./scripts/init.sh
```
This will download the PX4 autopilot software to `libs/PX4-Autopilot`, then build the docker image. It will take 5-6 minutes to run, depending on your internet speed.

Then, run the docker. To access the GUI, this command MUST be run from within an active display (e.g., you can't run the run command from an SSH connection).
```bash
./scripts/docker_run.sh
```
After the docker is running however, you can issue commands over ssh and the GUI will show on the display you were on when you started the Docker.

Once in the docker, build the ros directory:
```bash
cd ros2-ws
colcon build
```

To do:
- Bring in a forest map environment
- Get the ROS messaging working