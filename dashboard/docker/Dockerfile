##########################
#  Dashboard Environment #
##########################
FROM oven/bun:debian AS dashboard
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y curl

# Setup the dashboard environment
RUN mkdir /dashboard
WORKDIR /dashboard
COPY ./dashboard/ ./

RUN chown -R bun /dashboard/
USER bun
WORKDIR /dashboard

RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash
ENV NVM_DIR=/home/bun/.nvm
ENV NODE_VERSION=22.4.0
RUN . "$NVM_DIR/nvm.sh" && nvm install ${NODE_VERSION}
RUN . "$NVM_DIR/nvm.sh" && nvm use v${NODE_VERSION}
RUN . "$NVM_DIR/nvm.sh" && nvm alias default v${NODE_VERSION}
ENV PATH="$NVM_DIR/versions/node/v${NODE_VERSION}/bin/:${PATH}"