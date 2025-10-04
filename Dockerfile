FROM ghcr.io/droneepicengineering/base:latest

ARG USERNAME=dee
ARG GAZEBO_VERSION=harmonic
ARG ROS_DISTRO=humble

USER ${USERNAME}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN sudo apt-get update && \
    sudo apt-get -y --quiet --no-install-recommends install \
    gz-${GAZEBO_VERSION} \
    ros-${ROS_DISTRO}-ros-gz \
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /home/${USERNAME}
RUN sudo usermod -aG dialout "${USERNAME}" && \
    sudo apt-get update && \
    sudo apt-get -y --quiet --no-install-recommends install \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libqt5gui5 \
    libfuse2 \
    fuse \
    libpulse-mainloop-glib0 \
    libmosquitto-dev \
    mosquitto \
    && sudo rm -rf /var/lib/apt/lists/*

RUN curl -L -o QGroundControl-x86_64.AppImage https://github.com/mavlink/qgroundcontrol/releases/download/v5.0.7/QGroundControl-x86_64.AppImage && \
    chmod +x ./QGroundControl-x86_64.AppImage

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
