FROM ros:jazzy-ros-base

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    # Core dev tools
    git build-essential cmake ninja-build \
    # Build system and colcon
    python3-colcon-common-extensions \
    # C++ toolchain helpers
    clang-format clang-tidy \
    # Networking + utils
    sudo iproute2 iputils-ping curl wget htop vim tmux \
    # ROS control and UR drivers
    ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-ur \
    # MoveIt and tf2 extras used by this repo
    ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-tf2-ros ros-$ROS_DISTRO-tf2-eigen ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-controller-manager-msgs ros-$ROS_DISTRO-ur-dashboard-msgs \
    # OpenCV (incl. contrib for img_hash)
    libopencv-dev libopencv-contrib-dev \
    # Open3D and math libs used across deps
    libglfw3-dev libglew-dev libeigen3-dev \
    # Python bits used by libtorch build script
    python3 python3-dev python3-pip python3-typing-extensions \
    # BLAS/LAPACK and Protobuf often required by libtorch/Open3D
    libopenblas-dev liblapack-dev protobuf-compiler libprotobuf-dev zlib1g-dev libjpeg-dev libpng-dev && \
    rm -rf /var/lib/apt/lists/*

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

# Create non-root user for runtime (no GUI tools like RViz installed here)
RUN id -u ubuntu >/dev/null 2>&1 || useradd -m -d /workspace -s /bin/bash ubuntu && \
    passwd -d ubuntu || true && usermod -aG sudo ubuntu && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu && chmod 0440 /etc/sudoers.d/ubuntu

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
