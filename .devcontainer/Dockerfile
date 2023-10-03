#FROM osrf/ros:noetic-desktop-full
FROM drexel_labs
 
# # Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales
 
# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update 
RUN apt-get install -y --no-install-recommends python3-rosdep

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install turtlebot stuff



# install git
RUN apt-get update && apt-get install -y git


# # Build the Catkin workspace and ensure it's sourced
RUN source /opt/ros/noetic/setup.bash \
 && cd /turtlebot3_ws \
 && catkin_make \
 && source /turtlebot3_ws/devel/setup.bash
RUN echo "source /turtlebot3_ws/devel/setup.bash" >> ~/.bashrc


# Set the working folder at startup
# RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

WORKDIR catkin_ws

# Change the default shell to Bash

SHELL [ "/bin/bash" , "-c" ]

