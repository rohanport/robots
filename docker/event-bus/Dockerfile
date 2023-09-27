##########################
#  Event Bus Environment #
##########################
FROM oven/bun:latest as event-bus
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
 sudo

# Setup the event bus environment
RUN mkdir /event-bus
WORKDIR /event-bus
COPY ./event-bus/ ./

# Set up the entrypoint
COPY ./docker/event-bus/entrypoint.sh /

ARG USERNAME=bun

RUN chown -R $USERNAME /event-bus/

# Set the user and source entrypoint in the user's .bashrc file
COPY ./docker/event-bus/entrypoint.sh /
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc
USER ${USERNAME}