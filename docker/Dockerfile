FROM osrf/ros:noetic-desktop-full

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
ARG USER fossa
RUN useradd -U --uid ${user_id} -ms /bin/bash $USER \
 && echo "$USER:$USER" | chpasswd \
 && adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# add groups to get access to sensors
USER $USER
WORKDIR /home/$USER
RUN sudo adduser $USER dialout
RUN sudo adduser $USER tty 
RUN sudo adduser $USER plugdev

# ublox gps driver and deps
RUN mkdir -p ws/src \
 && cd ws/src \
 && git clone https://github.com/KumarRobotics/ublox \
 && git clone https://github.com/tilk/rtcm_msgs \
 && git clone https://github.com/LORD-MicroStrain/ntrip_client

# imu driver
RUN sudo apt-get update \
 && sudo apt-get install -y ros-noetic-microstrain-inertial-driver ros-noetic-gtsam

RUN sudo apt-get install -y python3-catkin-tools

# install visualizer deps
RUN sudo apt-get install -y python3-pip
RUN pip3 install flask folium flask_socketio utm

# build ros env
RUN cd ~/ws \
 && catkin config --extend /opt/ros/noetic \
 && catkin build --no-status -DCMAKE_BUILD_TYPE=Release

# make life nice inside docker
RUN sudo chown $USER:$USER ~/.bashrc \
 && /bin/sh -c 'echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo "source ~/ws/devel/setup.bash" >> ~/.bashrc' \
 && echo 'export PS1="\[$(tput setaf 2; tput bold)\]\u\[$(tput setaf 7)\]@\[$(tput setaf 3)\]\h\[$(tput setaf 7)\]:\[$(tput setaf 4)\]\W\[$(tput setaf 7)\]$ \[$(tput sgr0)\]"' >> ~/.bashrc

