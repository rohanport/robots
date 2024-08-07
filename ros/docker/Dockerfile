# ROS distribution to use
ARG ROS_DISTRO=iron

##################################
# Base Image for ROS Environment #
##################################
FROM osrf/ros:${ROS_DISTRO}-desktop AS base
ARG GAZEBO_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV GZ_VERSION=${GAZEBO_DISTRO}
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
 git lsb-release wget gnupg python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine

# Install required setuptools version
RUN pip3 install setuptools==58.2.0 transforms3d

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rosbridge-server \
 ros-${ROS_DISTRO}-ros-gz \
 ros-${ROS_DISTRO}-ros2-control \
 ros-${ROS_DISTRO}-ros2-controllers \
 ros-${ROS_DISTRO}-gz-ros2-control \
 ros-${ROS_DISTRO}-tf-transformations

# Install Gazebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y --no-install-recommends \
 gz-${GAZEBO_DISTRO}

# Create Colcon workspace with external dependencies
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws/src

COPY ./ros/dependencies.repos .
RUN vcs import < dependencies.repos

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /ros_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Set up the entrypoint
WORKDIR /ros_ws
COPY ./ros/docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#####################################
# Overlay Image for ROS Environment #
#####################################
FROM base AS overlay

# Create an overlay Colcon workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws

RUN source /ros_ws/install/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

# Set up the entrypoint
COPY ./ros/docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#####################
# Development Image #
#####################
FROM overlay AS dev

# Dev container arguments
ARG USERNAME=ros-dev
ARG UID=1000
ARG GID=${UID}

# Install extra tools for development
RUN apt-get update && apt-get install -y --no-install-recommends \
 gdb gdbserver nano

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /overlay_ws/

# Used to persist bash history as per https://code.visualstudio.com/remote/advancedcontainers/persist-bash-history
RUN SNIPPET="export PROMPT_COMMAND='history -a' && sudo chown -R $USERNAME /commandhistory && export HISTFILE=/commandhistory/.bash_history" \
    && mkdir /commandhistory \
    && echo "$SNIPPET" >> "/home/$USERNAME/.bashrc"

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc
