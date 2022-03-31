FROM osrf/ros:melodic-desktop-bionic

ARG ssh_prv_key

# install ros
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'

# install further ROS dependencies
RUN apt-get update && apt-get install -y \
        python-rosdep  \
        python-rosinstall  \
        python-rosinstall-generator  \
        python-wstool  \
        build-essential  \
        python-rosdep \
        ros-melodic-catkin  \
        python-catkin-tools  \
        ros-melodic-rotors-simulator

# install other packages
RUN apt-get update && apt-get install -y \
        nano  \
        git

WORKDIR /home
RUN git clone https://github.com/ydschnappi/UAV_search_and_rescue.git
RUN cd UAV_search_and_rescue/src &&\
    git submodule init &&\
    git submodule update


# set working directory to project
WORKDIR /home/UAV_search_and_rescue

RUN catkin config --extend /opt/ros/melodic
RUN catkin build
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'
RUN /bin/bash -c 'source devel/setup.bash'
