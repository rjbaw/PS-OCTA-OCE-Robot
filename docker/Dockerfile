# syntax=docker/dockerfile:1.7-labs
FROM ubuntu:24.04 AS open3d-builder

USER root
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        git build-essential cmake ninja-build sudo \
        python3 curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/src
RUN git clone -b v0.19.0 https://github.com/isl-org/Open3D.git \
    && cd Open3D \
    && yes | bash util/install_deps_ubuntu.sh \
    && mkdir build \
    && cd build \
    && cmake -G Ninja -DBUILD_WEBRTC=OFF -DCMAKE_INSTALL_PREFIX=/opt/open3d-install .. \
    && cmake --build . --target install

FROM openvino/ubuntu24_dev:2025.3.0 AS base

USER root
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        curl gnupg2 lsb-release software-properties-common \
        git git-lfs build-essential cmake ninja-build \
        clang-format clang-tidy \
        sudo iproute2 iputils-ping wget htop vim tmux traceroute ripgrep \
        ocl-icd-libopencl1 clinfo \
        libopencv-dev libopencv-contrib-dev \
        libglfw3-dev libglew-dev libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/src
RUN git clone --depth 1 -b v1.22.2 https://github.com/microsoft/onnxruntime.git && \
    bash -lc 'source /opt/intel/openvino/setupvars.sh; \
      cd /opt/src/onnxruntime; \
      ./build.sh \
        --allow_running_as_root \
        --use_openvino HETERO:NPU,GPU \
        --parallel \
        --skip_tests \
        --update \
        --build \
        --cmake_generator Ninja \
        --compile_no_warning_as_error \
        --config Release \
        --build_shared_lib \
        --cmake_extra_defines CMAKE_INSTALL_RPATH="\$ORIGIN"; \
        cmake --install build/Linux/Release --prefix /usr/local' && \
    rm -rf /opt/src/onnxruntime

ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

RUN set -eux; \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list; \
    apt-get update; \
    apt-get install -y \
      ros-$ROS_DISTRO-ros-base \
      ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers \
      ros-$ROS_DISTRO-ur \
      ros-$ROS_DISTRO-trac-ik-kinematics-plugin \
      ros-$ROS_DISTRO-launch-testing-ament-cmake \
      ros-$ROS_DISTRO-ament-cmake-clang-format \
      python3-colcon-common-extensions \
      libc++-dev libc++abi-dev libgmock-dev && \
    rm -rf /var/lib/apt/lists/*

COPY --from=open3d-builder /opt/open3d-install/ /usr/local/

RUN id -u ubuntu >/dev/null 2>&1 || useradd -m -d /workspace -s /bin/bash ubuntu \
    && passwd -d ubuntu || true && usermod -aG sudo ubuntu \
    && echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/ubuntu && chmod 0440 /etc/sudoers.d/ubuntu

RUN pip install --no-cache-dir empy catkin_pkg lark pyyaml packaging && rm -rf /root/.cache/pip || true


FROM base AS app-builder

USER ubuntu

WORKDIR /workspace/source
COPY --chown=ubuntu:ubuntu \
     --exclude=.git \
     --exclude=.github \
     --exclude=.cache \
     --exclude=.ruff_cache \
     --exclude=**/data \
     --exclude=**/build \
     --exclude=**/install \
     --exclude=**/log \
     --exclude=cad \
     --exclude=result \
     --exclude=VI \
     --exclude=core* \
     . /workspace/source

RUN bash -lc 'source /opt/ros/$ROS_DISTRO/setup.bash; \
    colcon build --merge-install --install-base /workspace/app' \
    && rm -rf /workspace/source
COPY --chown=ubuntu:ubuntu launch.sh /workspace/app/launch.sh

FROM base AS test
USER ubuntu
WORKDIR /workspace/repo
COPY --chown=ubuntu:ubuntu \
     --exclude=.git \
     --exclude=.github \
     --exclude=.cache \
     --exclude=.ruff_cache \
     --exclude=**/data \
     --exclude=**/build \
     --exclude=**/install \
     --exclude=**/log \
     --exclude=cad \
     --exclude=result \
     --exclude=VI \
     --exclude=core* \
     . /workspace/repo
RUN bash -lc 'make clean && make test-ci'

FROM base AS run

USER ubuntu
WORKDIR /workspace/app
COPY --from=app-builder --chown=ubuntu:ubuntu /workspace/app /workspace/app
RUN chmod +x /workspace/app/launch.sh
USER ubuntu

CMD ["/workspace/app/launch.sh"]
