
# FROM dustynv/ros:humble-desktop-l4t-r35.4.1 
FROM ros:humble-ros-base-jammy

# Install prereqs. This adds the full install
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Add extra programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \ 
    ros-dev-tools \
    i2c-tools \
    nano \
    build-essential \
    libffi-dev \
    python3-pip \
    python3-dev \ 
    sudo \
    wget \
    gdb \ 
    tree \
    screen \ 
    minicom \ 
    socat \
    lsof \
    libfuse2 \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev \
    && rm -rf /var/lib/apt/lists/*

# Setup user. The USER_UID needs to match the user of the host computer! That way the files won't have access issues.
ARG USERNAME=ros
ARG USER_UID=1002
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \ 
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Add sudo privelidges
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Install python requirements
COPY ./docker/python-requirements.txt python-requirements.txt
RUN pip3 install --no-cache-dir -r python-requirements.txt

# Set privelidges for the user
RUN usermod -aG dialout ros

# Set the container's environment variables to enable rviz and others
ENV QT_X11_NO_MITSHM=1

# Build libraries
# COPY ./libraries /home/${USERNAME}/libraries
# RUN chown -R ${USERNAME} /home/${USERNAME}/libraries
USER ros
WORKDIR /home/${USERNAME}
RUN mkdir libraries
WORKDIR /home/${USERNAME}/libraries

# PX4-Autopilot
RUN git clone https://github.com/PX4/PX4-Autopilot.git
USER root
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
# we will link our own version of repository
RUN rm -rf /home/${USERNAME}/PX4-Autopilot

# Qgroundcontrol
RUN apt-get update && apt-get remove modemmanager -y \
    && rm -rf /var/lib/apt/lists/*
RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
RUN chmod +x ./QGroundControl.AppImage

# Micro XRCE-DDS
USER ros
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR /home/${USERNAME}/libraries/Micro-XRCE-DDS-Agent
RUN mkdir build
WORKDIR /home/${USERNAME}/libraries/Micro-XRCE-DDS-Agent/build
USER root
RUN cmake .. && make -j$(nproc) && make install && ldconfig /usr/local/lib

# Set up entrypoints
COPY ./docker/bashrc.txt /home/${USERNAME}/.bashrc
COPY ./docker/entrypoint.sh /entrypoint.sh

# Update ROS dependencies
USER ros
RUN rosdep update
USER root

WORKDIR /home/ros/

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

CMD ["bash"]