########################
#  RxInfer Environment #
########################
FROM julia:latest AS rxinfer
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
 git sudo

# Setup the rxinfer julia environment
RUN mkdir /rxinfer
WORKDIR /rxinfer
COPY ./rxinfer/ ./

# Set up the entrypoint
COPY ./docker/rxinfer/entrypoint.sh /

ARG USERNAME=rxinfer
ARG UID=1000
ARG GID=${UID}

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /rxinfer/

# Set the user and source entrypoint in the user's .bashrc file
COPY ./docker/rxinfer/entrypoint.sh /
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc
USER ${USERNAME}

# Permanently set the JULIA_PROJECT
ENV JULIA_PROJECT "/rxinfer"

# Install project dependencies
WORKDIR /rxinfer
RUN julia -e 'using Pkg; Pkg.instantiate()'