# Use ROS base image
FROM ros:humble

# Set the user name and id
ENV USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ENV BASH_SETUP_REQUIRED=true

# Basic information about the image
LABEL Name="${USERNAME}-$ROS_DISTRO-orin" \
      Version="1.0" \
      Maintainer="Rasmus Ohert"

# Set HOME environment variable
ENV HOME /home/${USERNAME}

# Set the display environment variable to local host
ENV DISPLAY=:0

# Create a root user
RUN groupadd --gid $USER_GID $USERNAME \
    && adduser --uid $USER_UID --gid $USER_GID --disabled-password --gecos "" ${USERNAME} \
    && usermod -a -G dialout $USERNAME

# Install basic tools, Python 3.10.XX, minimalmodbus, and ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    nano \
    wget \
    python3.10 \
    python3-pip \
    libmodbus-dev \
    net-tools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update  apt-get install -y --no-install-recommends \
    ros-humble-rqt* \
    ros-humble-rviz2 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install minimalmodbus \
    FastDebugger \
    evdev \
    setuptools==58.2.0 \
    plotly \
    pandas \
    Jetson.GPIO \
    adafruit-circuitpython-bno055 
# setuptools==58.2.0 is the latest version to work with ros2 python packages without any warnings

# Set up sudo for the user
RUN echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Copy the bashrc scripts to the container
COPY bashrc ${HOME}/.bashrc

# Copy the startup script to the container

# Create a mount directory (mainly for testing)
# ENV ROS2_MOUNT_DIR_PATH /ros2_ws
# RUN mkdir -p ${ROS2_MOUNT_DIR_PATH} && chown -R ${USER_UID}:${USER_GID} ${ROS2_MOUNT_DIR_PATH}
# WORKDIR ${ROS2_MOUNT_DIR_PATH}


# Set the user
USER ros

# Default command to start the container
CMD [ "bash" ]
