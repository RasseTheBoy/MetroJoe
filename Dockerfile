FROM ros:humble

ARG ROS_PAKCAGE_NAME=new_package

# Set the user name and id
ENV USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Basic information about the image
LABEL   Name="${USERNAME}-$ROS_DISTRO-orin" \
        Version="1.0" \
        Maintainer="Rasmus Ohert"

# Set HOME environment variable
ENV HOME /home/${USERNAME}

# Create a non-root user
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}\
    && mkdir ${HOME}/.config && chown ${USER_UID}:${USER_GID} ${HOME}/.config

# Install some basic tools and Python 3.10.XX
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    nano \
    wget \
    python3.10 \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set up sudo for the user
RUN echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Copy the bashrc scripts to the container
COPY bashrc ${HOME}/.bashrc

# Set the workspace as the working directory
WORKDIR ${HOME}/ros_ws/src

CMD [ "bash" ]