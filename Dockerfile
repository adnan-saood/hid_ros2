FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ="Europe/Paris"

# Create user 'robot' and set up home directory
# Set the working directory to /dev

RUN useradd -m -s /bin/bash robot && echo "robot:robot" | chpasswd && adduser robot sudo
RUN usermod -aG sudo robot
RUN echo 'robot ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN newgrp sudo

RUN mkdir /catkin_ws && chown -R robot:robot /catkin_ws
WORKDIR /robot

USER robot

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ="Europe/Paris"

# install ros2 packages
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1*


RUN sudo apt-get install ros-humble-rviz2 -y
RUN sudo apt install git -y
RUN sudo apt-get install python3-colcon-common-extensions -y
RUN sudo apt-get install python3-rosdep -y
RUN sudo apt-get install python3-rosinstall -y
RUN sudo apt-get install python3-rosinstall-generator -y
RUN sudo apt-get install python3-wstool -y
RUN sudo apt-get install python3-vcstool -y
RUN sudo apt-get install python3-argcomplete -y
RUN sudo apt-get install python3-pip -y
RUN sudo apt-get install python3-setuptools -y
RUN sudo apt-get install python3-pyqt5 -y



RUN sudo apt-get install python3-pygraphviz -y
RUN sudo apt-get install ros-humble-ros2controlcli -y
RUN sudo apt-get install ros-humble-ros2-control -y
RUN sudo apt-get install ros-humble-ros2-controllers -y
RUN sudo apt-get install ros-humble-pid-controller -y
# Install RTW
RUN cd /home/robot/ && git clone https://github.com/StoglRobotics/ros_team_workspace.git
RUN cd /home/robot/ros_team_workspace/rtwcli/ && pip3 install -r requirements.txt
RUN echo "source /home/robot/ros_team_workspace/setup.bash" >> /home/robot/.bashrc
