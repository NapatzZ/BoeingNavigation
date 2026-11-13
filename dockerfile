FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    software-properties-common \
    apt-transport-https \
    wget \
    curl \
    git \
    vim \
    byobu \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    libeigen3-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update 

RUN apt-get update && apt-get install -y \ 
    ros-humble-slam-toolbox \ 
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-msgs \
    ros-humble-kobuki-ros-interfaces \
    ros-humble-kobuki-velocity-smoother \
    ros-humble-sophus \
    ros-humble-teleop-twist-keyboard \
    ros-humble-joy-teleop \
    ros-humble-teleop-twist-joy \
    ros-humble-twist-mux \
    ros-humble-spatio-temporal-voxel-layer \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws
RUN mkdir -p src

WORKDIR /ros_ws/src

RUN git clone https://github.com/NapatzZ/skuba-kobuki-driver.git

RUN git clone https://github.com/Slamtec/sllidar_ros2.git

COPY boeing_description /ros_ws/src/boeing_description
COPY boeing_bringup /ros_ws/src/boeing_bringup
COPY boeing_navigation /ros_ws/src/boeing_navigation

WORKDIR /ros_ws
RUN apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    colcon build \
      --symlink-install \
      --executor sequential \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
    "

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros_ws/install/setup.bash" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /ros_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]




