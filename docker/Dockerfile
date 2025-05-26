FROM osrf/ros:jazzy-desktop-full

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

RUN userdel ubuntu

# add a user
ARG user_id
ARG USER jalesco
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
RUN mkdir -p ws/src
# imu driver
RUN sudo apt-get update \
 && sudo apt-get install -y ros-jazzy-gtsam

RUN sudo chown -R $USER:$USER ws/

RUN sudo chown $USER:$USER ~/.bashrc \
 && /bin/sh -c 'echo ". /opt/ros/jazzy/setup.bash" >> ~/.bashrc' \
 && echo 'export PS1="\[$(tput setaf 2; tput bold)\]\u\[$(tput setaf 7)\]@\[$(tput setaf 3)\]\h\[$(tput setaf 7)\]:\[$(tput setaf 4)\]\W\[$(tput setaf 7)\]$ \[$(tput sgr0)\]"' >> ~/.bashrc
