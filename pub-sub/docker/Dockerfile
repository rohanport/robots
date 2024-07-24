##########################
#  pub-sub Environment #
##########################
FROM oven/bun:debian AS pub-sub
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y sudo git

# Setup the pub-sub environment
RUN mkdir /workspace
RUN mkdir /workspace/pub-sub
WORKDIR /workspace/pub-sub
COPY ./pub-sub/ ./

# Set the ownership of the overlay workspace to the new user
RUN chown -R bun /workspace/pub-sub

USER bun
WORKDIR /workspace/pub-sub

#####################
# Development Image #
#####################
FROM pub-sub AS dev

USER root

# Dev container arguments
ARG USERNAME=pub-sub
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
RUN chown -R ${UID}:${GID} /workspace/pub-sub

USER ${USERNAME}
WORKDIR /workspace/pub-sub