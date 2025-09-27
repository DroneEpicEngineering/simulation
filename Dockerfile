FROM --platform=amd64 ghcr.io/droneepicengineering/base:latest

ARG USERNAME=dee
ARG GAZEBO_VERSION=fortress
ARG ROS_DISTRO=humble

USER ${USERNAME}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN sudo apt-get update && \
    sudo apt-get -y --quiet --no-install-recommends install \
    ros-${ROS_DISTRO}-ros-gz \
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /home/${USERNAME}
RUN sudo usermod -aG dialout "${USERNAME}" && \
    sudo apt-get update && \
    sudo apt-get -y --quiet --no-install-recommends install \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    libfuse2 \
    libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev \
    && sudo rm -rf /var/lib/apt/lists/*

RUN wget -q https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage && \
    chmod +x ./QGroundControl.AppImage

WORKDIR /home/${USERNAME}
RUN git clone "https://github.com/PX4/PX4-Autopilot.git" --branch v1.15.4 --recursive && \
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
ENV PX4_PATH=/home/${USERNAME}/PX4-Autopilot
WORKDIR ${PX4_PATH}
RUN make "-j$(nproc)" px4_sitl

ENV ROS_WORKSPACE=/home/${USERNAME}/ws
WORKDIR ${ROS_WORKSPACE}/src
RUN git clone "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git" --branch v2.4.2 && \
    git clone "https://github.com/PX4/px4_msgs.git" --branch "release/1.15"

WORKDIR $ROS_WORKSPACE
RUN rosdep update && \
    rosdep install --from-paths ${ROS_WORKSPACE} -r -y --ignore-src

WORKDIR $ROS_WORKSPACE
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build

WORKDIR $ROS_WORKSPACE/src
COPY . simulation

WORKDIR $ROS_WORKSPACE
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash"

RUN echo "source \"/opt/ros/${ROS_DISTRO}/setup.bash\"" >> "/home/${USERNAME}/.bashrc" && \
    echo "source \"${ROS_WORKSPACE}/install/setup.bash\"" >> "/home/${USERNAME}/.bashrc"

CMD ["/bin/bash"]
