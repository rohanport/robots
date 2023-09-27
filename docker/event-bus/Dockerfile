##########################
#  Event Bus Environment #
##########################
FROM oven/bun:latest as event-bus
SHELL ["/bin/bash", "-c"]

# Setup the event bus environment
RUN mkdir /event-bus
WORKDIR /event-bus
COPY ./event-bus/ ./

RUN chown -R bun /event-bus/
USER bun
