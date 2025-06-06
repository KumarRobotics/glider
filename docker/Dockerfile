FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

# install the basics
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
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
 unzip

# add a user
ARG user_id
ARG USER jammy
RUN useradd -U --uid ${user_id} -ms /bin/bash $USER \
 && echo "$USER:$USER" | chpasswd \
 && adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# add groups to get access to sensors
USER $USER
WORKDIR /home/$USER

# imu driver
RUN sudo apt-get update \
 && sudo apt-get install -y ros-humble-gtsam

RUN sudo apt-get install -y python3-colcon-common-extensions

# install visualizer deps
RUN sudo apt-get install -y python3-pip
RUN pip3 install flask folium flask_socketio utm

# build ros env
RUN mkdir ws

# make life nice inside docker
RUN sudo chown $USER:$USER ~/.bashrc \
 && /bin/sh -c 'echo ". /opt/ros/humble/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo "source ~/ws/install/setup.bash" >> ~/.bashrc' \
 && echo 'export PS1="\[$(tput setaf 2; tput bold)\]\u\[$(tput setaf 7)\]@\[$(tput setaf 3)\]\h\[$(tput setaf 7)\]:\[$(tput setaf 4)\]\W\[$(tput setaf 7)\]$ \[$(tput sgr0)\]"' >> ~/.bashrc

