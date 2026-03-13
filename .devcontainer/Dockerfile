ARG BASE_IMAGE=lcas.lincoln.ac.uk/ros:humble-staging
ARG USERNAME=ros
ARG ROS_DISTRO=humble

FROM ${BASE_IMAGE} AS devcontainer

COPY . /home/$USERNAME/workspace

USER root

RUN rm -rf /var/lib/apt/lists/* \
    && apt-get update --fix-missing \
    && apt-get -y install --no-install-recommends --fix-missing \
        ros-${ROS_DISTRO}-desktop \
        bash-completion \
        python3-colcon-common-extensions \
        python3-pip \
        python3-vcstool \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*
    
USER ${USERNAME}
    
RUN rosdep update --rosdistro ${ROS_DISTRO} &&\
    sudo apt-get update && \
    rosdep install --from-paths /home/ros/workspace/src --ignore-src -r -y

WORKDIR /home/ros/workspace
RUN colcon build --symlink-install
