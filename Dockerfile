FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

WORKDIR /workspace

RUN apt-get update && apt-get install -y \
    terminator \
    gir1.2-aravis-0.8 \
    iproute2 \
    python3-gi \
    python3-opencv \
    ros-jazzy-cv-bridge \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-moveit \
    ros-jazzy-moveit-py \
    ros-jazzy-moveit-servo \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    sudo \
    usbutils \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# YOLO for arctos_cv. numpy stays below 2, cv_bridge is built against 1.26
RUN pip3 install --break-system-packages --no-cache-dir ultralytics "numpy<2"

# Run as a user matching the host's UID/GID so files created in the mounted
# workspace are owned by the host user (set UID/GID in .env, see .env.example)
ARG USERNAME=dev
ARG UID=1000
ARG GID=1000

RUN if id ubuntu &>/dev/null; then userdel -r ubuntu; fi \
    && groupadd -o -g ${GID} ${USERNAME} \
    && useradd -o -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc

# Sets up the camera's network port at start, see entrypoint.sh. Called as
# camera-net-setup it only does the setup
COPY entrypoint.sh /entrypoint.sh
RUN ln -s /entrypoint.sh /usr/local/bin/camera-net-setup
ENTRYPOINT ["/entrypoint.sh"]

USER ${USERNAME}

CMD ["bash"]
