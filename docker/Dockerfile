# ─────────────────────────────────────────────────────────────────────────────
# AeroSAR — ROS 2 Humble (Ubuntu 22.04 Jammy)
# Migrated from Jazzy/Gazebo to Humble/AirSim + MAVROS
# AirSim runs on the HOST machine; this container is the ROS 2 workspace.
# ─────────────────────────────────────────────────────────────────────────────
FROM osrf/ros:humble-desktop

ARG USERNAME=aerosar
ARG USER_UID=1000
ARG USER_GID=1000

# ── Base system packages ──────────────────────────────────────────────────────
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
    python3-pip \
    sudo \
    wget curl gnupg2 ca-certificates \
    git git-lfs \
    build-essential cmake \
    nano vim \
    # AirSim C++ client deps
    libasio-dev libtinyxml2-dev libssl-dev \
    libboost-filesystem-dev libboost-system-dev \
    # GStreamer (kept for image transport)
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libfuse2 \
    astyle \
    # GeographicLib (required by MAVROS)
    libgeographic-dev geographiclib-tools \
    && rm -rf /var/lib/apt/lists/*

# ── ROS 2 Humble core packages ────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-std-msgs \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rviz2 \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# ── MAVROS (replaces Micro XRCE-DDS; needed for PX4-Avoidance & AirSim) ──────
# mavros-extras ships the GeographicLib dataset installer script
RUN apt-get update && apt-get install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-mavros-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets (mandatory for MAVROS EKF)
RUN wget -q https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && bash install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

# ── AirSim ROS 2 wrapper build deps ──────────────────────────────────────────
# The airsim_ros_pkgs source is cloned into src/ at workspace level.
# We only pre-install the apt-level deps here so the colcon build finds them.
RUN apt-get update && apt-get install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-geographic-msgs \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# ── Python packages ───────────────────────────────────────────────────────────
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --ignore-installed -r /tmp/requirements.txt \
    # msgpack is required by the AirSim Python client
    && rm /tmp/requirements.txt

# ── rosdep init ───────────────────────────────────────────────────────────────
RUN rosdep update

# ── Dev user ──────────────────────────────────────────────────────────────────
RUN if id -u $USER_UID 2>/dev/null; then userdel $(id -un $USER_UID); fi \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ── Shell setup ───────────────────────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /home/$USERNAME/ws/install/setup.bash 2>/dev/null || true" >> /home/$USERNAME/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> /home/$USERNAME/.bashrc \
    # AIRSIM_HOST is set at runtime via docker-compose; default to localhost
    && echo 'export AIRSIM_HOST="${AIRSIM_HOST:-127.0.0.1}"' >> /home/$USERNAME/.bashrc \
    && echo 'alias sds="source install/setup.bash"' >> /home/$USERNAME/.bashrc \
    && echo 'alias cb="colcon build --symlink-install"' >> /home/$USERNAME/.bashrc \
    && echo 'alias mavros_up="ros2 launch bringup mavros.launch.py"' >> /home/$USERNAME/.bashrc

ENV SHELL=/bin/bash

USER $USERNAME
WORKDIR /home/$USERNAME/ws

COPY --chown=$USERNAME:$USERNAME entrypoint.sh /home/$USERNAME/entrypoint.sh
RUN chmod +x /home/$USERNAME/entrypoint.sh

ENTRYPOINT ["/home/aerosar/entrypoint.sh"]
CMD ["/bin/bash"]