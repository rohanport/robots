##########################
#  Dashboard Environment #
##########################
FROM oven/bun:latest as dashboard
SHELL ["/bin/bash", "-c"]

# Setup the dashboard environment
RUN mkdir /dashboard
WORKDIR /dashboard
COPY ./dashboard/ ./

RUN chown -R bun /dashboard/
USER bun
