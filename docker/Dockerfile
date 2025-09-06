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
    clang-format \
    clang-tidy \
    python3-typing-extensions \
    ros-$ROS_DISTRO-ros2-control \ 
    ros-$ROS_DISTRO-ros2-controllers

RUN git clone -b v0.19.0 https://github.com/isl-org/Open3D.git &&\
    cd Open3D &&\
    yes | bash util/install_deps_ubuntu.sh &&\
    mkdir build &&\ 
    cd build &&\
    cmake -G Ninja -DBUILD_WEBRTC=OFF .. &&\
    cmake --build . --target install &&\
    cd ../../ &&\
    rm -r Open3D

RUN git clone -b v2.8.0 --recurse-submodule https://github.com/pytorch/pytorch.git &&\
    cd pytorch &&\
    python3 tools/build_libtorch.py &&\
    cmake --install build --prefix /usr/local &&\
    cd ../../ &&\
    rm -r pytorch

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
    chmod +x tools/* &&\
    bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install" &&\
    chmod +x launch.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["/workspace/ps-oce-robot/launch.sh"]
