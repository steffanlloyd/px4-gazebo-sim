# Build
docker build \
    --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) \
    -t gazebo-px4-sim:latest -f Dockerfile .
