FROM osrf/ros:jazzy-desktop-full
#image: ros:jazzy

RUN apt-get update &&\
    apt-get install -y \
    libopencv-dev \
    ros-jazzy-ur \
    vim \
    tmux \
    curl \
    wget \
    htop

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

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
