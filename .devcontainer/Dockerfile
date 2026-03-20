ARG BASE_IMAGE=lcas.lincoln.ac.uk/ros:humble-staging
ARG ROS_DISTRO=humble

FROM ${BASE_IMAGE} AS devcontainer

USER root

RUN rm -rf /var/lib/apt/lists/* \
    && apt-get update --fix-missing \
    && apt-get -y install --no-install-recommends --fix-missing \
    ros-${ROS_DISTRO}-rviz2 \
    bash-completion \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

USER ros

COPY ./ /home/ros/workspace/

WORKDIR /home/ros/workspace
RUN sudo rm -rf log/ build/ install/  && \    
    sudo chown -R ros:ros /home/ros/workspace

ENV ROS2_WS=/home/ros/workspace

RUN rosdep update --rosdistro ${ROS_DISTRO} &&\
    sudo apt-get update && \
    rosdep install --from-paths ./src --ignore-src -r -y

# Do an initial build to speed up development
RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/ros/.bashrc && colcon build --continue-on-error"


# ----------------------------------------------------------------------------------------------------
FROM devcontainer AS final

RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/ros/.bashrc && colcon build"

ENV MAP_FILE=/home/ros/workspace/src/topological_navigation/config/test_simple_tmap2.yaml 
CMD ["/bin/bash", "-lc", "source /home/ros/workspace/install/setup.bash && exec ros2 launch topological_navigation topological_navigation.launch.py map_path:=$MAP_FILE"]