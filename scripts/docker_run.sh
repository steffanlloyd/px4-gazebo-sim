# Connect display, if there is a display
if [ -n "$DISPLAY" ]; then
    xhost +local:root
fi

# Check if the Docker container is running
if [ "$(docker ps -q -f name=gazebo-px4-sim)" ]; then
    # Prompt the user
    read -p "The Docker container 'gazebo-px4-sim' is already running. Do you want to restart it? (y/n) " answer
    case ${answer:0:1} in
        y|Y )
            # Stop the Docker container
            echo "Stopping docker..."
            docker stop gazebo-px4-sim
            echo "Done."
            ;;
        * )
            # Exit the script
            echo "Exiting."
            exit
            ;;
    esac
fi

# Start the docker
echo "Starting docker gazebo-px4-sim..."
docker run -itd \
  --name="gazebo-px4-sim" \
  --rm \
  --privileged --network host --ipc host --user ros \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(dirname "$0")/../ros2-ws:/home/ros/ros2-ws \
  -v $(dirname "$0")/../save:/home/ros/save \
  -v $(dirname "$0")/../libs/PX4-Autopilot:/home/ros/libraries/PX4-Autopilot \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  gazebo-px4-sim:latest \
  tail -f /dev/null

echo "Done."

docker ps