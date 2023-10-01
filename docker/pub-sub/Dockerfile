##########################
#  pub-sub Environment #
##########################
FROM oven/bun:latest as pub-sub
SHELL ["/bin/bash", "-c"]

# Setup the pub-sub environment
RUN mkdir /pub-sub
WORKDIR /pub-sub
COPY ./pub-sub/ ./

RUN chown -R bun /pub-sub/
USER bun
WORKDIR /pub-sub
RUN bun install