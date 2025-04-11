FROM ros:humble-ros-base

ARG USERNAME=dee
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ARG GAZEBO_VERSION=fortress
ARG ROS_DISTRO=humble

# Install general dependencies
RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
    build-essential \
    cmake \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user with sudo privileges
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install Gazebo
RUN sudo apt-get update && \
    sudo apt-get -y --quiet --no-install-recommends install \
    ros-${ROS_DISTRO}-ros-gz \
    && sudo rm -rf /var/lib/apt/lists/*

# Install QGroundControl
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

# Install PX4
WORKDIR /home/${USERNAME}
RUN git clone "https://github.com/PX4/PX4-Autopilot.git" --branch v1.15.4 --recursive && \
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
ENV PX4_PATH=/home/${USERNAME}/PX4-Autopilot
WORKDIR ${PX4_PATH}
RUN make "-j$(nproc)" px4_sitl

# Clone repositories to workspace
ENV ROS_WORKSPACE=/home/${USERNAME}/ws
WORKDIR ${ROS_WORKSPACE}/src
RUN git clone "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git" --branch v2.4.2 && \
    git clone "https://github.com/PX4/px4_msgs.git" --branch "release/1.15"

# Install ROS dependencies
WORKDIR $ROS_WORKSPACE
RUN rosdep update && \
    rosdep install --from-paths ${ROS_WORKSPACE} -r -y --ignore-src

# Build
WORKDIR $ROS_WORKSPACE
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build

# Copy simulation package to workspace
WORKDIR $ROS_WORKSPACE/src
COPY . simulation

WORKDIR $ROS_WORKSPACE
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash"

RUN echo "source \"/opt/ros/${ROS_DISTRO}/setup.bash\"" >> "/home/${USERNAME}/.bashrc" && \
    echo "source \"${ROS_WORKSPACE}/install/setup.bash\"" >> "/home/${USERNAME}/.bashrc"

CMD ["/bin/bash"]
