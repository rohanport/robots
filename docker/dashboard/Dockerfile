##########################
#  Dashboard Environment #
##########################
FROM oven/bun:latest as dashboard
SHELL ["/bin/bash", "-c"]

# Setup the event bus environment
RUN mkdir /dashboard
WORKDIR /dashboard
COPY ./dashboard/ ./

RUN chown -R bun /dashboard/
USER bun
