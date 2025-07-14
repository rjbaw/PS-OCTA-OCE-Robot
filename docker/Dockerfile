FROM osrf/ros:jazzy-desktop-full

RUN apt-get update &&\
    apt-get install -y \
    libopencv-dev \
    libglfw3-dev \
    ros-$ROS_DISTRO-ur \
    vim \
    tmux \
    curl \
    wget \
    htop \
    sudo \
    iproute2 \
    iputils-ping \
    ros-$ROS_DISTRO-ros2-control \ 
    ros-$ROS_DISTRO-ros2-controllers

RUN git clone https://github.com/isl-org/Open3D &&\
    cd Open3D &&\
    git checkout v0.19.0 &&\
    yes | bash util/install_deps_ubuntu.sh &&\
    mkdir build &&\ 
    cd build &&\
    cmake .. &&\
    make -j $(nproc) &&\
    make install &&\
    cd ../../ &&\
    rm -r Open3D

RUN deluser ubuntu
RUN adduser ubuntu --disabled-password --home /workspace/
RUN passwd -d ubuntu && usermod -aG sudo ubuntu

RUN chmod +x /ros_entrypoint.sh

ARG CACHE=0

RUN git clone https://github.com/rjbaw/PS-OCTA-OCE-Robot.git /workspace/ps-oce-robot/
RUN chown -R ubuntu:ubuntu /workspace/ps-oce-robot

USER ubuntu
WORKDIR /workspace/ps-oce-robot/

RUN ulimit -c 0

RUN cd /workspace/ps-oce-robot &&\
    bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install" &&\
    chmod +x launch.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["/workspace/ps-oce-robot/launch.sh"]
