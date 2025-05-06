# This is an auto generated Dockerfile for ros:ros-core
# generated from docker_images_ros2/create_ros_core_image.Dockerfile.em
FROM ubuntu:jammy

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN set -eux; \
       key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
       export GNUPGHOME="$(mktemp -d)"; \
       gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
       mkdir -p /usr/share/keyrings; \
       gpg --batch --export "$key" > /usr/share/keyrings/ros2-latest-archive-keyring.gpg; \
       gpgconf --kill all; \
       rm -rf "$GNUPGHOME"

# setup sources.list
RUN echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO humble

# install ros2 packages 
RUN apt-get update && apt-get install -y --no-install-recommends --fix-missing\
    ros-humble-ros-core \
    && rm -rf /var/lib/apt/lists/*

# INSTALL GAZEBO
# Install curl and required dependencies
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    lsb-release \
    gnupg \
    sudo \
    ca-certificates \
    curl \
    && rm -rf /var/lib/apt/lists/*
# Install Gazebo 11 using the one-liner install script
RUN curl -sSL http://get.gazebosim.org | sh

# INSTALL PX4 TOOLCHAIN
RUN apt-get update && apt-get install -y\
    git \
    gnupg2 \
    python3-pip \
    python3-jinja2 \
    python3-numpy \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    python3-future \
    python3-toml \
    python3-empy \
    python3-yaml \
    python3-click \
    python3-pygments \
    ninja-build \
    exiftool \
    && rm -rf /var/lib/apt/lists/*
# Download the PX4 Source code
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive /PX4-Autopilot
# Run the PX4 setup script 
RUN bash /PX4-Autopilot/Tools/setup/ubuntu.sh

# INSTALL uXRCE-DDS AGENT
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    make \
    libssl-dev \
    libboost-all-dev \
    libasio-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*
# Clone the uXRCE-DDS Agent repository
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /Micro-XRCE-DDS-Agent
# Build and install the uXRCE-DDS Agent
WORKDIR /Micro-XRCE-DDS-Agent
RUN mkdir build && cd build && cmake .. && make && make install
# Update the linker cache
RUN sudo ldconfig /usr/local/lib/

# INSTALL PX4-ROS2 BRIDGE
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-eigen3-cmake-module \
# Install Python dependencies
RUN pip3 install -U "empy==3.3.4" pyros-genmsg "setuptools<=65.5.1"

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
