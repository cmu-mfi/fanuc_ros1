FROM ros:noetic-ros-core

# Set working directory
WORKDIR /root

# Fix GPG key before ANY apt-get update
RUN rm -f /etc/apt/sources.list.d/ros1-latest.list /usr/share/keyrings/ros1-latest-archive-keyring.gpg

# Add the new key and updated source entry
RUN apt-get update && apt-get install -y curl gnupg ca-certificates && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1.list

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-wstool \
    python3-pybind11 \
    python3-catkin-tools \
    build-essential \
    libjsoncpp-dev \
    vim tmux \
    ros-noetic-eigenpy \
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    ros-noetic-urdf \
    ros-noetic-tf-conversions \
    ros-noetic-tf2-ros \
    ros-noetic-code-coverage \
    ros-noetic-rviz \
    ros-noetic-roslint \
    ros-noetic-tf2-eigen \
    ros-noetic-interactive-markers \
    ros-noetic-actionlib \
    ros-noetic-tf2-kdl \
    ros-noetic-controller-manager-msgs \
    ros-noetic-nodelet \
    ros-noetic-tf \
    ros-noetic-industrial-robot-client \
    ros-noetic-object-recognition-msgs \
    ros-noetic-franka-description \
    ros-noetic-resource-retriever \
    ros-noetic-octomap \
    ros-noetic-octomap-msgs \
    ros-noetic-eigen-stl-containers \
    liboctomap-dev \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    ros-noetic-graph-msgs \
    libfcl-dev \
    ros-noetic-random-numbers \
    libqhull-dev \
    libbenchmark-dev \
    libblas-dev liblapack-dev \
    ros-noetic-eigen-conversions \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-ompl \
    ros-noetic-moveit-planners-ompl \
    ros-noetic-warehouse-ros-mongo \
    libglew-dev \
    freeglut3-dev \
    ros-noetic-cv-bridge \
    ros-noetic-control-toolbox \
    ros-noetic-rosparam-shortcuts \
    ros-noetic-joint-limits-interface \
    ros-noetic-industrial-robot-simulator \
    ros-noetic-catkin \
    ros-noetic-joint-state-publisher\
    python3-pip \
    ros-noetic-pluginlib \
    ros-noetic-rosunit \
    ros-noetic-rostest \
    ros-noetic-std-msgs \
    ros-noetic-moveit-resources-panda-description \
    ros-noetic-roscpp \
    ros-noetic-pilz-industrial-motion \
    ros-noetic-joint-state-publisher-gui \
    spacenavd \
    ros-noetic-spacenav-node \
    ros-noetic-trac-ik-kinematics-plugin \
    ros-noetic-joy \
    ros-noetic-joy-teleop \
    && rm -rf /var/lib/apt/lists/*
RUN pip install ruckig 
RUN pip install -U pip
RUN pip install -U wheel setuptools
RUN pip install https://github.com/gavanderhoorn/comet_rpc/archive/0.2.4.tar.gz
#More 

# Enable universe repo and fix sources
RUN sed -i 's/^# deb/deb/' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y \
        qtbase5-dev \
        qtchooser \
        qt5-qmake \
        qtbase5-dev-tools
ENV PKG_CONFIG_PATH=/usr/lib/pkgconfig:/usr/lib/x86_64-linux-gnu/pkgconfig:/usr/share/pkgconfig


# Initialize rosdep
RUN rosdep init && rosdep update

# Set environment
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=noetic

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Create and build your workspace (preserved from your original setup)
RUN mkdir -p /root/ros1_ws/src

# COPY your full workspace (optional: uncomment if not using volume mount)
COPY ./ /root/ros1_ws/src/fanuc_ros1

# MoveIt setup
RUN cd /root/ros1_ws/src && \
    wstool init . && \
    wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall && \
    wstool remove moveit_tutorials && \
    wstool update -t .

# Install dependencies
RUN cd /root/ros1_ws && \
    rosdep install --from-paths src --ignore-src -r -y

# Build
WORKDIR /root/ros1_ws
RUN bash -c "source /opt/ros/noetic/setup.bash"

# Source workspace
#RUN catkin config --init && catkin build
RUN echo "source /root/ros1_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /root/
SHELL ["/bin/bash", "-c"]

