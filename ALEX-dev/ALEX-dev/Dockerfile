FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    git \
    iputils-ping \
    nmap \
    # Package installation
    libpoco-dev \
    libyaml-cpp-dev \
    ros-humble-control-msgs \
    ros-humble-realtime-tools \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-msgs \
    ros-humble-moveit-msgs \
    dbus-x11 \
    libignition-gazebo6-dev \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-moveit-msgs \
    ros-humble-ros-gz-sim \
    ros-humble-ign-ros2-control

RUN /bin/bash -c 'printf "source /opt/ros/humble/setup.bash \nsource /ros2_ws/install/setup.bash \nexport ROS_DOMAIN_ID=101 \n" >> ~/.bashrc;'

RUN mkdir -p /ros2_ws/src/ALEX
WORKDIR /ros2_ws/src

RUN git clone -b humble-devel https://github.com/doosan-robotics/doosan-robot2.git \
    && git clone -b ros2-master https://github.com/IntelRealSense/realsense-ros.git 

RUN rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y \
    && rm -rf /var/lib/apt/lists/* 

RUN /bin/bash -c '. /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build'

COPY . /ros2_ws/src/ALEX

RUN /bin/bash -c '. /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --symlink-install --packages-select-regex alex_*'

ENTRYPOINT ["/bin/bash", "/ros2_ws/src/ALEX/docker_entrypoint.sh"]
