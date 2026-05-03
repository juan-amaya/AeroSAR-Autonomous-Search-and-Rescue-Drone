FROM osrf/ros:jazzy-desktop

ARG USERNAME=aerosar
ARG USER_UID=1000
ARG USER_GID=1000

# ── Base system packages ───────────────────────────────────────────────────────
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
    python3-pip \
    sudo \
    wget curl gnupg2 ca-certificates \
    git git-lfs \
    build-essential cmake \
    nano vim \
    libasio-dev libtinyxml2-dev libssl-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libfuse2 \
    astyle \
    && rm -rf /var/lib/apt/lists/*

# ── ROS 2 extra packages ──────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-rviz2 \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# ── Gazebo Harmonic ───────────────────────────────────────────────────────────
RUN wget https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y \
    gz-harmonic \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    && rm -rf /var/lib/apt/lists/*

# ── Micro XRCE-DDS Agent ──────────────────────────────────────────────────────
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce-agent \
    && cd /tmp/xrce-agent \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) \
    && make install \
    && rm -rf /tmp/xrce-agent

# ── Python packages ───────────────────────────────────────────────────────────
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --break-system-packages --ignore-installed -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# ── rosdep init (must run as root before switching user) ─────────────────────
RUN rosdep update

# ── Dev user ──────────────────────────────────────────────────────────────────
RUN if id -u $USER_UID 2>/dev/null; then userdel $(id -un $USER_UID); fi \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ── Shell setup ───────────────────────────────────────────────────────────────
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /home/$USERNAME/ws/install/setup.bash 2>/dev/null || true" >> /home/$USERNAME/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> /home/$USERNAME/.bashrc \
    && echo 'alias sds="source install/setup.bash"' >> /home/$USERNAME/.bashrc \
    && echo 'alias cb="colcon build --symlink-install"' >> /home/$USERNAME/.bashrc

ENV SHELL=/bin/bash

USER $USERNAME
WORKDIR /home/$USERNAME/ws

CMD ["/bin/bash"]