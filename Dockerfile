# Start from ROS Jazzy + CUDA base
FROM dtcpronto/ros-jazzy:cuda

USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
    vim \
    tmux \
    cmake \
    gcc \
    g++ \
    git \
    build-essential \
    sudo \
    wget \
    curl \
    zip \
    unzip \
    ros-jazzy-gtsam \
    ros-jazzy-gps-msgs \
    python3-colcon-common-extensions \
    libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/*

# Switch to dtc user
USER dtc
WORKDIR /home/dtc/ws

# Clone glider
COPY --chown=dtc:dtc ./glider /home/dtc/ws/src/glider
COPY --chown=dtc:dtc ./dgps_msgs /home/dtc/ws/src/dgps_msgs

# Build workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install"

COPY --chown=dtc:dtc ./entrypoint.bash /home/dtc/entrypoint.bash
RUN chmod +x /home/dtc/entrypoint.bash

ENTRYPOINT ["/home/dtc/entrypoint.bash"]
