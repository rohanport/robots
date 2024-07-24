##########################
#  Dashboard Environment #
##########################
FROM oven/bun:debian AS dashboard
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y sudo git

# Setup the dashboard environment
RUN mkdir /workspace
RUN mkdir /workspace/dashboard
WORKDIR /workspace/dashboard
COPY ./dashboard/ ./

# Set the ownership of the overlay workspace to the new user
RUN chown -R bun /workspace/dashboard

USER bun
WORKDIR /workspace/dashboard

#####################
# Development Image #
#####################
FROM dashboard AS dev

USER root

# Dev container arguments
ARG USERNAME=dashboard
ARG UID=1000
ARG GID=1000

# Switch the bun user's uid to 999 as vscode binds our user to 1000
RUN groupmod --gid ${GID} bun \
    && usermod --uid 999 --gid ${GID} bun \
    && chown -R 999:${GID} /home/bun

# Create new user and home directory
RUN useradd --uid ${GID} --gid ${GID} --create-home ${USERNAME} \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the workspace to the new user
RUN chown -R ${UID}:${GID} /workspace/dashboard

USER ${USERNAME}
WORKDIR /workspace/dashboard