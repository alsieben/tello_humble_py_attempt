FROM osrf/ros:humble-desktop
WORKDIR /ros2_ws

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config & chown $USER_UID

RUN apt-get update \
    && u apt-get isntall -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rem-rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y python3-pip \
        libboost-python-dev \
    && pip3 install numpy \
        djitellopy \
        opencv-contrib-python \
        pyyaml \
        pygame
    && pip3 install --force-reinstall numpy==1.21.5
    
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    && source /opt/ros/humble/setup.bash

RUN cd src \
    && git clone https://github.com/ros-perception/vision_opencv.git\
    && cd .. \
    && colcon build --symlink-install
