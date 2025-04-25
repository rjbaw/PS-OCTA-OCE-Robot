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

USER ubuntu
WORKDIR /workspace

RUN ulimit -c 0

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
