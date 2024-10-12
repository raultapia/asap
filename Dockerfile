ARG ROS_DISTRO=noetic
FROM osrf/ros:$ROS_DISTRO-desktop
ARG ROS_VERSION=1

RUN apt update
RUN apt install -y wget
RUN apt install -y git
RUN apt install -y ros-$ROS_DISTRO-vision-opencv
RUN apt install -y ros-$ROS_DISTRO-cv-bridge

# cmake
RUN wget https://github.com/Kitware/CMake/releases/download/v3.20.0/cmake-3.20.0.tar.gz
RUN tar -xzvf cmake-3.20.0.tar.gz
WORKDIR /cmake-3.20.0
RUN ./bootstrap
RUN make
RUN make install
WORKDIR /

# openev
RUN apt install -y autoconf
RUN apt install -y libtool
RUN apt install -y libudev-dev
RUN git clone https://github.com/raultapia/openev
RUN mkdir -p /openev/build
WORKDIR /openev/build
RUN cmake .. -DBUILD_MODULE_CONTAINERS=ON -DBUILD_MODULE_CORE=ON -DBUILD_MODULE_DEVICES=ON -DBUILD_MODULE_READERS=OFF -DBUILD_MODULE_REPRESENTATIONS=OFF -DBUILD_MODULE_UTILS=ON -DBUILD_EXAMPLES=OFF
RUN make
RUN make install

# dvs_msgs
RUN mkdir -p /asap_ws/src
WORKDIR /asap_ws/src
RUN git clone https://github.com/ros-event-camera/dvs_msgs

# asap
RUN git clone https://github.com/raultapia/asap
WORKDIR /asap_ws
RUN if [ $ROS_VERSION = 1 ]; then \
    apt install -y python3-catkin-tools; \
    . /opt/ros/$ROS_DISTRO/setup.sh && catkin build; \
    elif [ $ROS_VERSION = 2 ]; then \
    . /opt/ros/$ROS_DISTRO/setup.sh && colcon build; \
    else \
    exit 1; \
    fi
